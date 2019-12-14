// For loading the data
#include "ReadTrajectory.h"

// Camera observations of landmarks (i.e. pixel coordinates) will be stored as Point2 (x, y).
#include <gtsam/geometry/Point2.h>

// Inference and optimization
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

// SFM-specific factors
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/ProjectionFactorCalibrationFull.h> // does calibration !
#include <gtsam/slam/ProjectionFactorCalibrationRotationOnly.h> // does calibration !

// Standard headers
#include <cmath>
#include <vector>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
int main(int argc, char* argv[]) {

  // USAGE:
  // ./StereoSelfCalibrationAPI trial_exp 0 rot data/StereoSelfCalibrationFeatures.txt data/StereoSelfCalibrationPoses.txt data/StereoSelfCalibrationPosesDeviated.txt data/StereoSelfCalibrationExtrinsics.txt data/StereoSelfCalibrationExtrinsicsDeviated.txt pose
  // or
  // ./StereoSelfCalibrationAPI trial_exp 0 pose

  // Check command line args
  if (argc < 4) {
    cout << "Provide the experiment name, the experiment ID and optimization mode (See README.md). For example:\n";
    cout << "./StereoSelfCalibrationAPI trial_exp 0 rot\n OR\n";
    cout << "./StereoSelfCalibrationAPI trial_exp 0 pose\n";
    exit(1);
  }

  string exp_name = argv[1];
  int counter = atoi(argv[2]);
  string opt_mode = argv[3];

  if (opt_mode.compare("rot") != 0 && opt_mode.compare("pose") != 0){
    cout << "Third argument can only be one of 'pose' or 'rot'\n";
    exit(1);
  }

  // Create experiment directory
  const int dir_err = system(("mkdir -p " + exp_name).c_str());
  if (-1 == dir_err) {
      printf("Error creating directory!n");
      exit(1);
  }

  // Specify file paths
  string feature_filepath, pose_filepath, pose_deviated_filepath, extrinsic_filepath, extrinsic_deviated_filepath, fixed_translation_path;
  if (argc > 4) {
    feature_filepath = argv[4];
    pose_filepath = argv[5];
    pose_deviated_filepath = argv[6];
    extrinsic_filepath = argv[7];
    extrinsic_deviated_filepath = argv[8];
  }
  else {
    feature_filepath = "../../examples/Data/StereoSelfCalibration/Features.txt";
    pose_filepath = "../../examples/Data/StereoSelfCalibration/Poses.txt";
    pose_deviated_filepath = "../../examples/Data/StereoSelfCalibration/PosesDeviated.txt";
    extrinsic_filepath = "../../examples/Data/StereoSelfCalibration/Extrinsics.txt";
    extrinsic_deviated_filepath = "../../examples/Data/StereoSelfCalibration/ExtrinsicsDeviated.txt";
  }
  fixed_translation_path = "../../examples/Data/StereoSelfCalibration/FixedTranslationVector.txt";

  // Create the set of ground-truth
  vector<Point3> points = readPoints(feature_filepath);
  vector<Pose3> poses = readPoses(pose_filepath);
  vector<Pose3> deviatedPoses = readPoses(pose_deviated_filepath);
  vector<Pose3> extrinsic = readPoses(extrinsic_filepath);
  vector<Pose3> deviatedExtrinsics = readPoses(extrinsic_deviated_filepath);
  Point3 extrinsicTranslation =  readTranslation(fixed_translation_path);

  // Some flags
  bool feature_rounding = true;

  // Create the factor graph
  NonlinearFactorGraph graph;

  // Add a prior on pose x1
  noiseModel::Diagonal::shared_ptr poseNoise = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(0.3), Vector3::Constant(0.1)).finished()); // 30cm std on x,y,z 0.1 rad on roll,pitch,yaw
  graph.emplace_shared<PriorFactor<Pose3> >(Symbol('x', 0), poses[0], poseNoise);

  // Add a prior on the extrinsic calibration parameters
  if (opt_mode.compare("pose") == 0){
    noiseModel::Diagonal::shared_ptr extrinsicNoise = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(0.3), Vector3::Constant(0.1)).finished()); // 30cm std on x,y,z 0.1 rad on roll,pitch,yaw
    graph.emplace_shared<PriorFactor<Pose3> >(Symbol('R', 0), extrinsic[0], extrinsicNoise);
  }
  else{
    noiseModel::Diagonal::shared_ptr extrinsicNoise = noiseModel::Diagonal::Sigmas((Vector(3) << Vector3::Constant(0.3)).finished()); // 30cm std on x,y,z 0.1 rad on roll,pitch,yaw
    graph.emplace_shared<PriorFactor<Rot3> >(Symbol('R', 0), extrinsic[0].rotation(), extrinsicNoise);
  }

  // Define intrinsics
  Cal3_S2::shared_ptr KLeft(new Cal3_S2(350.0, 350.0, 0.0, 336.0, 188.0));
  Cal3_S2::shared_ptr KRight(new Cal3_S2(350.0, 350.0, 0.0, 336.0, 188.0));

  // Simulated measurements from each camera pose, adding them to the factor graph
  noiseModel::Isotropic::shared_ptr measurementNoise = noiseModel::Isotropic::Sigma(2, 1.0);

  // Keep flag list for selecting which landmarks have been seen
  vector<int> visible_landmarks(points.size(), 0);

  // Iterate over all poses and register measurments and their valid factors
  float mean_feature_count = 0;
  for (size_t i = 0; i < poses.size(); ++i) {
    int feature_count = 0;
    for (size_t j = 0; j < points.size(); ++j) {
      Pose3 poseLeft = deviatedPoses[i];
      Pose3 poseRight = poseLeft.compose(deviatedExtrinsics[i]);
      SimpleCamera cameraLeft(poseLeft, *KLeft);
      SimpleCamera cameraRight(poseRight, *KRight);

      Point2 measurementLeft, measurementRight;

      try{
        measurementLeft = cameraLeft.project(points[j]);
      }
      catch (...){
        continue;
      }
      try{
        measurementRight = cameraRight.project(points[j]);
      }
      catch (...){
        continue;
      }

      if (feature_rounding) {
        measurementLeft[0] = round(measurementLeft[0]);
        measurementLeft[1] = round(measurementLeft[1]);
        measurementRight[0] = round(measurementRight[0]);
        measurementRight[1] = round(measurementRight[1]);
      }
      bool lower_bound = (measurementLeft[0] > 0.0 && measurementLeft[1] > 0.0 && measurementRight[0] > 0.0 && measurementRight[1] > 0.0);
      bool upper_bound = (measurementLeft[0] < 672.0 && measurementLeft[1] < 376.0 && measurementRight[0] < 672.0 && measurementRight[1] < 376.0);
      if (lower_bound && upper_bound){
        visible_landmarks[j] = 1;
        feature_count++;
        graph.emplace_shared<GenericProjectionFactor<Pose3, Point3, Cal3_S2> >(measurementLeft, measurementNoise, Symbol('x', i), Symbol('l', j), KLeft);
        if (opt_mode.compare("pose") == 0){
          graph.emplace_shared<ProjectionFactorCalibrationFull<Pose3, Point3, Cal3_S2> >(measurementRight, measurementNoise, Symbol('x', i), Symbol('R', i), Symbol('l', j), KRight);
        }
        else{
          graph.emplace_shared<ProjectionFactorCalibrationRotationOnly<Pose3, Point3, Rot3, Cal3_S2> >(measurementRight, measurementNoise, Symbol('x', i), Symbol('R', i), Symbol('l', j), KRight);
        }
      }
    }
    mean_feature_count = mean_feature_count + feature_count;
  }

  mean_feature_count = mean_feature_count / poses.size();

  // Create the initial estimate to the solution
  // now including an estimate on the camera calibration parameters
  Values initialEstimate;
  if (opt_mode.compare("pose") == 0){
    for (size_t i = 0; i < poses.size(); ++i)
      initialEstimate.insert(Symbol('R', i), extrinsic[i]);
  }
  else{
    for (size_t i = 0; i < poses.size(); ++i)
      initialEstimate.insert(Symbol('R', i), extrinsic[i].rotation());
  }
  for (size_t i = 0; i < poses.size(); ++i)
    initialEstimate.insert(Symbol('x', i), poses[i]);
  for (size_t j = 0; j < points.size(); ++j){
    if (visible_landmarks[j] == 1) {
      initialEstimate.insert<Point3>(Symbol('l', j), points[j] + Point3(-0.025, 0.020, 0.015));
    }
  }

  // Optimize the graph and get results
  Values result = LevenbergMarquardtOptimizer(graph, initialEstimate).optimize();
  Values pose_filtered = result.filter(Symbol::ChrTest('R'));

  // Extract estimated extrinsics
  Pose3 estimated_extrinsic_pose;
  Values extrinsics_filtered = result.filter(Symbol::ChrTest('R'));
  Key Rkey = symbol('R', poses.size()-1);
  if(opt_mode.compare("pose") == 0){
    estimated_extrinsic_pose = extrinsics_filtered.at(Rkey).cast<Pose3>();
  }
  else{
    estimated_extrinsic_pose = Pose3(extrinsics_filtered.at(Rkey).cast<Rot3>(), extrinsicTranslation);
  }
  Vector3 estimated_extrinsic_rot = estimated_extrinsic_pose.rotation().ypr();
  Vector3 estimated_extrinsic_trans = estimated_extrinsic_pose.translation();

  // Compute errors in extrinsic rotation and translation
  Pose3 error_pose = estimated_extrinsic_pose.compose(deviatedExtrinsics[poses.size()-1].inverse());
  Vector3 rot_final_error = error_pose.rotation().ypr();
  Vector3 tr_final_error = error_pose.translation();

  // Write estimated extrinsics to result file
  ofstream result_file;
  string result_filename;
  result_filename = exp_name + "/result_";
  result_filename += to_string(counter);
  result_filename += ".txt";
  result_file.open(result_filename);
  result_file << estimated_extrinsic_rot[0]<< " ";
  result_file << estimated_extrinsic_rot[1]<< " ";
  result_file << estimated_extrinsic_rot[2]<< " ";
  result_file << estimated_extrinsic_trans[0]<< " ";
  result_file << estimated_extrinsic_trans[1]<< " ";
  result_file << estimated_extrinsic_trans[2]<< "\n";
  result_file.close();

  // Write rotation and translation errors
  ofstream error_file;
  string error_filename;
  error_filename = exp_name + "/error_";
  error_filename += to_string(counter);
  error_filename += ".txt";
  error_file.open(error_filename);
  error_file << rot_final_error << "\n";
  error_file << tr_final_error << "\n";
  error_file << mean_feature_count << "\n";
  error_file.close();

  return 0;
}

