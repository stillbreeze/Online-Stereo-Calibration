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
#include <gtsam/slam/ProjectionFactorCalibration.h> // does calibration !

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
  // ./StereoSelfCalibrationAPI trial_exp 0 data/StereoSelfCalibrationFeatures.txt data/StereoSelfCalibrationPoses.txt data/StereoSelfCalibrationPosesDeviated.txt data/StereoSelfCalibrationExtrinsics.txt data/StereoSelfCalibrationExtrinsicsDeviated.txt
  // or
  // ./StereoSelfCalibrationAPI trial_exp 0

  // Check command line args
  if (argc < 3) {
    cout << "Provide the experiment name and the experiment ID. For example:\n";
    cout << "./StereoSelfCalibrationAPI trial_exp 0\n";
    exit(1);
  }

  string exp_name = argv[1];
  int counter = atoi(argv[2]);

  // Create experiment directory
  const int dir_err = system(("mkdir -p " + exp_name).c_str());
  if (-1 == dir_err) {
      printf("Error creating directory!n");
      exit(1);
  }

  // Specify file paths
  string feature_filepath, pose_filepath, pose_deviated_filepath, extrinsic_filepath, extrinsic_deviated_filepath;
  if (argc > 3) {
    feature_filepath = argv[3];
    pose_filepath = argv[4];
    pose_deviated_filepath = argv[5];
    extrinsic_filepath = argv[6];
    extrinsic_deviated_filepath = argv[7];
  }
  else {
    feature_filepath = "../../examples/Data/StereoSelfCalibration/Features.txt";
    pose_filepath = "../../examples/Data/StereoSelfCalibration/Poses.txt";
    pose_deviated_filepath = "../../examples/Data/StereoSelfCalibration/PosesDeviated.txt";
    extrinsic_filepath = "../../examples/Data/StereoSelfCalibration/Extrinsics.txt";
    extrinsic_deviated_filepath = "../../examples/Data/StereoSelfCalibration/ExtrinsicsDeviated.txt";
  }

  // Create the set of ground-truth
  vector<Point3> points = readPoints(feature_filepath);
  vector<Pose3> poses = readPoses(pose_filepath);
  vector<Pose3> deviatedPoses = readPoses(pose_deviated_filepath);
  vector<Pose3> extrinsic = readPoses(extrinsic_filepath);
  vector<Pose3> deviatedExtrinsics = readPoses(extrinsic_deviated_filepath);

  // Some flags
  bool feature_rounding = true;

  // Create the factor graph
  NonlinearFactorGraph graph;

  // Add a prior on pose x1
  noiseModel::Diagonal::shared_ptr poseNoise = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(0.3), Vector3::Constant(0.1)).finished()); // 30cm std on x,y,z 0.1 rad on roll,pitch,yaw
  graph.emplace_shared<PriorFactor<Pose3> >(Symbol('x', 0), poses[0], poseNoise);

  // Add a prior on the extrinsic calibration parameters
  noiseModel::Diagonal::shared_ptr extrinsicNoise = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(0.3), Vector3::Constant(0.1)).finished()); // 30cm std on x,y,z 0.1 rad on roll,pitch,yaw
  graph.emplace_shared<PriorFactor<Pose3> >(Symbol('R', 0), extrinsic[0], extrinsicNoise);

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
        graph.emplace_shared<ProjectionFactorCalibration<Pose3, Point3, Cal3_S2> >(measurementRight, measurementNoise, Symbol('x', i), Symbol('R', i), Symbol('l', j), KRight);
      }
    }
    mean_feature_count = mean_feature_count + feature_count;
  }

  mean_feature_count = mean_feature_count / poses.size();

  // Create the initial estimate to the solution
  // now including an estimate on the camera calibration parameters
  Values initialEstimate;
  for (size_t i = 0; i < poses.size(); ++i)
    initialEstimate.insert(Symbol('R', i), extrinsic[i]);
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

  int i = 0;
  Vector3 rot_final_error;
  Vector3 tr_final_error;

  ofstream result_file;
  string result_filename;
  result_filename = exp_name + "/result_";
  result_filename += to_string(counter);
  result_filename += ".txt";
  ifstream infile(result_filename);
  if (infile.good())
    remove(result_filename.c_str());

  for(const Values::Filtered<Value>::KeyValuePair& key_value: pose_filtered.filter(Symbol::ChrTest('R'))) {
    Pose3 estimated_extrinsic = key_value.value.cast<Pose3>();
    Pose3 error_pose = estimated_extrinsic.compose(deviatedExtrinsics[i].inverse());

    Vector3 result_r = estimated_extrinsic.rotation().ypr();
    Point3 result_t = estimated_extrinsic.translation();
    result_file.open(result_filename, ios_base::app);
    result_file << result_r[0]<< " ";
    result_file << result_r[1]<< " ";
    result_file << result_r[2]<< " ";
    result_file << result_t[0]<< " ";
    result_file << result_t[1]<< " ";
    result_file << result_t[2]<< "\n";
    result_file.close();

    Vector3 error_rot = error_pose.rotation().ypr();
    Point3 error_tr = error_pose.translation();

    rot_final_error = error_rot;
    tr_final_error = error_tr;
    ++i;
  }

  ofstream error_file;
  string error_filename;
  error_filename = exp_name + "/error_";
  error_filename += to_string(counter);
  error_filename += ".txt";
  error_file.open (error_filename);
  error_file << rot_final_error << "\n";
  error_file << tr_final_error << "\n";
  error_file << mean_feature_count << "\n";
  error_file.close();

  return 0;
}

