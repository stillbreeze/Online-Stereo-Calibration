// For loading the data
// #include "StereoCalibrationData.h"
#include "StraightTrajectory.h"
#include "ReadTrajectory.h"

// Camera observations of landmarks (i.e. pixel coordinates) will be stored as Point2 (x, y).
#include <gtsam/geometry/Point2.h>

// Post optimization covariance matrices
#include <gtsam/nonlinear/Marginals.h>

// Inference and optimization
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
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
#include <cstdlib>
#include <fstream>
#include <typeinfo>
#include <iostream>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
int main(int argc, char* argv[]) {

  // USAGE:
  // Solve only for extrinsic rotation
  // ./StereoSelfCalibration 0 true 0.02 1000 1.0 rounding

  // Solve for complete extrinsic pose
  // ./StereoSelfCalibration 0 true 0.02 1000 1.0 rounding true

  // Some constants
  int counter = atoi(argv[1]);
  string rounding_flag = argv[2];
  float extrinsic_deviation = atof(argv[3]);
  int total_pts = atoi(argv[4]);
  float measurement_noise = atof(argv[5]);
  string exp_name = argv[6];
  string opt_translation_flag;
  if (argc > 7){
    opt_translation_flag = argv[7];
  }
  else{
    opt_translation_flag = "false";
  }



  // Create experiment directory
  const int dir_err = system(("mkdir -p " + exp_name).c_str());
  if (-1 == dir_err)
  {
      printf("Error creating directory!n");
      exit(1);
  }


  bool print_log = false;
  bool print_log_final = false;
  bool feature_rounding = false;
  bool opt_translation = false;

  if (rounding_flag == "true"){
    feature_rounding = true;
  }

  Point3 fixed_extrinsic_translation;
  if (opt_translation_flag == "true"){
    opt_translation = true;
  }
  else{
    string fixed_translation_path = "../../examples/Data/StereoSelfCalibration/FixedTranslationVector.txt";
    fixed_extrinsic_translation =  readTranslation(fixed_translation_path);
  }

  // Create the set of ground-truth
  vector<Point3> points = createPoints(total_pts);
  vector<Pose3> poses = createPoses();
  vector<Pose3> deviatedPoses = createPosesDeviated();
  vector<Pose3> extrinsic = createExtrinsics();
  vector<Pose3> deviatedExtrinsics = createExtrinsicsDeviated(extrinsic, extrinsic_deviation);


  // Create the factor graph
  NonlinearFactorGraph graph;

  // Add a prior on pose x1
  noiseModel::Diagonal::shared_ptr poseNoise = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(0.3), Vector3::Constant(0.1)).finished()); // 30cm std on x,y,z 0.1 rad on roll,pitch,yaw
  graph.emplace_shared<PriorFactor<Pose3> >(Symbol('x', 0), poses[0], poseNoise);

  // Add a prior on the extrinsic calibration parameters
  if (opt_translation){
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
  noiseModel::Isotropic::shared_ptr measurementNoise = noiseModel::Isotropic::Sigma(2, measurement_noise);

  // Keep flag list for selecting which landmarks have been seen
  vector<int> visible_landmarks(points.size(), 0);


  float mean_feature_count = 0;

  for (size_t i = 0; i < poses.size(); ++i) {
    int feature_count = 0;
    for (size_t j = 0; j < points.size(); ++j) {
      Pose3 poseLeft = deviatedPoses[i];
      Pose3 poseRight = poseLeft.compose(deviatedExtrinsics[i]);
      SimpleCamera cameraLeft(poseLeft, *KLeft);
      SimpleCamera cameraRight(poseRight, *KRight);

      // cout << "-----\n";
      // cout<< "\n";
      // points[j].print();
      // cout<< "\n";
      // poseLeft.print();
      // cout<<"\n";
      // poseRight.print();
      // cout<<"\n";
      // if (points[j][1] - poses[i].translation()[1] < 3.0){
      //   continue;
      // }
      Point2 measurementLeft, measurementRight;

      try{
        measurementLeft = cameraLeft.project(points[j]);
      }
      catch (...){
        if (print_log)
          // cout << "Left projection exception\n";
        continue;
      }
      try{
        measurementRight = cameraRight.project(points[j]);
      }
      catch (...){
        if (print_log)
          // cout << "Right projection exception\n";
        continue;
      }

      // measurementLeft.print();
      // cout<<"\n";
      // measurementRight.print();
      // cout << "-----\n";
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
        if (opt_translation){
          graph.emplace_shared<ProjectionFactorCalibrationFull<Pose3, Point3, Cal3_S2> >(measurementRight, measurementNoise, Symbol('x', i), Symbol('R', i), Symbol('l', j), KRight);
        }
        else{
          graph.emplace_shared<ProjectionFactorCalibrationRotationOnly<Pose3, Point3, Rot3, Cal3_S2> >(measurementRight, measurementNoise, Symbol('x', i), Symbol('R', i), Symbol('l', j), KRight);
        }
      }
    }
    if (print_log){
      cout << feature_count << "\n";
      cout << "*********" << "\n";
    }
    mean_feature_count = mean_feature_count + feature_count;
  }

  mean_feature_count = mean_feature_count / poses.size();

// // --------------------------------------------

  /* Create the initial estimate to the solution */
  /* now including an estimate on the camera calibration parameters */
  Values initialEstimate;
  if (opt_translation){
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


  /* Optimize the graph and print results */
  // Values result = DoglegOptimizer(graph, initialEstimate).optimize();
  Values result = LevenbergMarquardtOptimizer(graph, initialEstimate).optimize();

  Values rot_filtered = result.filter(Symbol::ChrTest('R'));
  Key Rkey = symbol('R', poses.size()-1);
  Pose3 estimated_extrinsic_pose;
  if (opt_translation){
    estimated_extrinsic_pose = rot_filtered.at(Rkey).cast<Pose3>();
  }
  else{
    estimated_extrinsic_pose = Pose3(rot_filtered.at(Rkey).cast<Rot3>(), fixed_extrinsic_translation);
  }
  
  Pose3 error_pose = estimated_extrinsic_pose.compose(deviatedExtrinsics[poses.size()-1].inverse());
  Vector3 rot_final_error = error_pose.rotation().ypr();
  Vector3 tr_final_error = error_pose.translation();


  ofstream file;
  string filename;
  filename = exp_name + "/error_";
  filename += to_string(counter);
  filename += ".txt";
  file.open (filename);
  file << rot_final_error << "\n";
  file << tr_final_error << "\n";
  file << mean_feature_count << "\n";
  file.close();

  // Computing the mean epipolar error and reprojection error of the batch segment
  float total_count = 0;
  float reprojection_error_sum = 0.0;
  float epipolar_error_sum = 0.0;
  for (size_t i = 0; i < poses.size(); ++i) {
    for (size_t j = 0; j < points.size(); ++j) {
      Pose3 poseLeft = deviatedPoses[i];
      Pose3 poseRight = poseLeft.compose(estimated_extrinsic_pose);
      Pose3 poseRightGroundTruth = poseLeft.compose(deviatedExtrinsics[i]);
      SimpleCamera cameraLeft(poseLeft, *KLeft);
      SimpleCamera cameraRight(poseRight, *KRight);
      SimpleCamera cameraRightGroundTruth(poseRightGroundTruth, *KRight);
      Point2 measurementLeft, measurementRight, measurementRightGroundTruth;
      try{
        measurementLeft = cameraLeft.project(points[j]);
      }
      catch (...){
        if (print_log)
        continue;
      }
      try{
        measurementRight = cameraRight.project(points[j]);
      }
      catch (...){
        if (print_log)
        continue;
      }
      try{
        measurementRightGroundTruth = cameraRightGroundTruth.project(points[j]);
      }
      catch (...){
        if (print_log)
        continue;
      }

      bool lower_bound = (measurementLeft[0] > 0.0 && measurementLeft[1] > 0.0 && measurementRight[0] > 0.0 && measurementRight[1] > 0.0  && measurementRightGroundTruth[0] > 0.0 && measurementRightGroundTruth[1] > 0.0);
      bool upper_bound = (measurementLeft[0] < 672.0 && measurementLeft[1] < 376.0 && measurementRight[0] < 672.0 && measurementRight[1] < 376.0 && measurementRightGroundTruth[0] < 672.0 && measurementRightGroundTruth[1] < 376.0);
      if (lower_bound && upper_bound){
        total_count++; 
        float distance = sqrt(pow(measurementRight[0] - measurementRightGroundTruth[0], 2) + pow(measurementRight[1] - measurementRightGroundTruth[1], 2));
        reprojection_error_sum += distance;
        float t1 = estimated_extrinsic_pose.translation()[0];
        float t2 = estimated_extrinsic_pose.translation()[1];
        float t3 = estimated_extrinsic_pose.translation()[2];
        Matrix3 essential_matrix_estimated = skewSymmetric(t1, t2, t3) * estimated_extrinsic_pose.rotation().matrix();
        Vector3 xleft = (*KLeft).matrix_inverse() * Vector3(measurementLeft[0], measurementLeft[1], 1);
        Vector3 xright = (*KRight).matrix_inverse() * Vector3(measurementRight[0], measurementRight[1], 1);
        epipolar_error_sum += fabs(xright.transpose() * essential_matrix_estimated * xleft);
      }
    }
  }
  float mean_reprojection_error = reprojection_error_sum / total_count;
  float mean_epipolar_error = epipolar_error_sum / total_count;

  // Estimating the information theoretic metrics of the batch segment
  try{
    Marginals marginals(graph, result);
    Matrix marginal_covariance_extrinsics = marginals.marginalCovariance(Symbol('R', poses.size()-1));
    Matrix fisher_information_matrix = marginal_covariance_extrinsics.inverse();
    float a_optimality = fisher_information_matrix.trace();
    float d_optimality = fisher_information_matrix.determinant();
    float e_optimality = fisher_information_matrix.eigenvalues()(0).real();

    ofstream file;
    string filename;
    filename = exp_name + "/information_metrics_";
    filename += to_string(counter);
    filename += ".txt";
    file.open (filename);
    file << a_optimality << "\n";
    file << d_optimality << "\n";
    file << e_optimality << "\n";
    file << mean_reprojection_error << "\n";
    file << mean_epipolar_error << "\n";
    file.close();
  }
  catch (...){
  }


  if (print_log_final){
    cout << "\n";
    cout << "\n";
    cout << "Final rotation error (z,y,x)\n";
    cout << rot_final_error;
    cout << "\n";
    cout << "Final translation error (x,y,z)\n";
    cout << tr_final_error;
    cout << "\n";
    cout << "\n";
  }
  // VectorValues retracted = pose_filtered.retract(pose_filtered);
  // pose_filtered.print("Final results:\n");



// // --------------------------------------------

  return 0;
}
/* ************************************************************************* */

