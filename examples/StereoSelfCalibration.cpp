// For loading the data
// #include "StereoCalibrationData.h"
#include "StraightTrajectory.h"

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
#include <cstdlib>
#include <fstream>
#include <iostream>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
int main(int argc, char* argv[]) {

  // USAGE:
  // ./StereoSelfCalibration 0 true 0.02 1000 1.0 rounding

  // Some constants
  int counter = atoi(argv[1]);
  string rounding_flag = argv[2];
  float extrinsic_deviation = atof(argv[3]);
  int total_pts = atoi(argv[4]);
  float measurement_noise = atof(argv[5]);
  string exp_name = argv[6];

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

  if (rounding_flag == "true"){
    feature_rounding = true;
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
  noiseModel::Diagonal::shared_ptr extrinsicNoise = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(0.3), Vector3::Constant(0.1)).finished()); // 30cm std on x,y,z 0.1 rad on roll,pitch,yaw
  graph.emplace_shared<PriorFactor<Pose3> >(Symbol('R', 0), extrinsic[0], extrinsicNoise);

  // Define intrinsics
  Cal3_S2::shared_ptr KLeft(new Cal3_S2(350.0, 350.0, 0.0, 336.0, 188.0));
  Cal3_S2::shared_ptr KRight(new Cal3_S2(350.0, 350.0, 0.0, 336.0, 188.0));

  // Simulated measurements from each camera pose, adding them to the factor graph
  noiseModel::Isotropic::shared_ptr measurementNoise = noiseModel::Isotropic::Sigma(2, measurement_noise);
  // Keep flag list for selecting which landmarks have been seen
  vector<int> visible_landmarks(points.size(), 0);




// ****************************************************************************************************

  // cout << "*********" << "\n";

  // Pose3 init = Pose3(gtsam::Rot3::Ypr(M_PI/2,0,0), gtsam::Point3(30, 0, 0));
  // Point3 pt3 = Point3(30,5,2);
  // SimpleCamera cam(init, *KLeft);

  // cout<<"\n";
  // init.print();
  // cout<<"\n";
  // pt3.print();
  // cout<<"\n";

  // try
  // {
  //   Point2 pt2 = cam.project(pt3);
  //   pt2.print();
  //   cout<<"\n";
  // }
  // catch (int e)
  // {
  //   cout << "Projection error" << e << '\n';
  // }

  // cout << "*********" << "\n";

  // Pose3 init1 = Pose3(gtsam::Rot3::Ypr(M_PI/2,0,0), gtsam::Point3(30, 0, 0));
  // Point3 pt31 = Point3(30,5,10);
  // SimpleCamera cam1(init1, *KLeft);

  // cout<<"\n";
  // init1.print();
  // cout<<"\n";
  // pt31.print();
  // cout<<"\n";

  // try
  // {
  //   Point2 pt21 = cam1.project(pt31);
  //   pt21.print();
  //   cout<<"\n";
  // }
  // catch (int e)
  // {
  //   cout << "Projection error" << e << '\n';
  // }

  // cout << "*********" << "\n";


  // Pose3 init2 = Pose3(gtsam::Rot3::Ypr(M_PI/2,0,0), gtsam::Point3(30, 0, 0));
  // Point3 pt32 = Point3(30,5,15);
  // SimpleCamera cam2(init2, *KLeft);

  // cout<<"\n";
  // init2.print();
  // cout<<"\n";
  // pt32.print();
  // cout<<"\n";

  // try
  // {
  //   Point2 pt22 = cam2.project(pt32);
  //   pt22.print();
  //   cout<<"\n";
  // }
  // catch (int e)
  // {
  //   cout << "Projection error" << e << '\n';
  // }

  // cout << "*********" << "\n";


// ****************************************************************************************************




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
        graph.emplace_shared<ProjectionFactorCalibration<Pose3, Point3, Cal3_S2> >(measurementRight, measurementNoise, Symbol('x', i), Symbol('R', i), Symbol('l', j), KRight);
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
  for (size_t i = 0; i < poses.size(); ++i)
    initialEstimate.insert(Symbol('R', i), extrinsic[i]);
  for (size_t i = 0; i < poses.size(); ++i)
    // initialEstimate.insert(Symbol('x', i), poses[i].compose(Pose3(Rot3::Rodrigues(-0.1, 0.2, 0.25), Point3(0.05, -0.10, 0.20))));
    initialEstimate.insert(Symbol('x', i), poses[i]);
  for (size_t j = 0; j < points.size(); ++j){
    if (visible_landmarks[j] == 1) {
      initialEstimate.insert<Point3>(Symbol('l', j), points[j] + Point3(-0.025, 0.020, 0.015));
    }
  }

  /* Optimize the graph and print results */
  // Values result = DoglegOptimizer(graph, initialEstimate).optimize();
  Values result = LevenbergMarquardtOptimizer(graph, initialEstimate).optimize();

  Values pose_filtered = result.filter(Symbol::ChrTest('R'));

  int i = 0;
  Vector3 rot_final_error;
  Vector3 tr_final_error;

  for(const Values::Filtered<Value>::KeyValuePair& key_value: pose_filtered.filter(Symbol::ChrTest('R'))) {
    Pose3 estimated_extrinsic = key_value.value.cast<Pose3>();
    Pose3 error_pose = estimated_extrinsic.compose(deviatedExtrinsics[i].inverse());

    // Vector3 rot = estimated_extrinsic.rotation().ypr();
    // Point3 tr = estimated_extrinsic.translation();

    Vector3 error_rot = error_pose.rotation().ypr();
    Point3 error_tr = error_pose.translation();

    rot_final_error = error_rot;
    tr_final_error = error_tr;
    if (print_log) {
      cout << error_rot;
      cout << "\n";
      cout << error_tr;
      cout << "\n";
      cout << "\n";
    }
    // if (print_log) {
    //   cout << estimated_extrinsic;
    //   cout << "\n";
    //   cout << extrinsic[i];
    //   cout << "\n";
    //   cout << deviatedExtrinsics[i];
    //   cout << "\n";
    //   cout << "\n";
    // }
    ++i;
  }

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

