/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    StereoCalibrationData.h
 * @brief   Simulated data for online stereo camera calibration
 */

#include <ctime>
#include <random>
#include <cstdlib>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/SimpleCamera.h>

/* ************************************************************************* */
std::vector<gtsam::Point3> createPoints(int& total_pts) {

  // Create the set of ground-truth landmarks
  std::vector<gtsam::Point3> points;
  srand (static_cast <unsigned> (time(0)));

  int i = 0;
  int steps = total_pts;
  for(; i < steps; ++i) {
    float x = 20 + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(40-20)));
    float y = 5 + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(30-5)));
    float z = 0 + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(10-0)));
    points.push_back(gtsam::Point3(x,y,z));
    // std::cout << x << " ";
    // std::cout << y << " ";
    // std::cout << z << "\n";
    // points[i].print();
  }

  // points.push_back(gtsam::Point3(25.5147,7.4793,0.105779));
  // points.push_back(gtsam::Point3(20.5147,7.4793,5.105779));
  // points.push_back(gtsam::Point3(25.5147,7.4793,5.105779));
  // points.push_back(gtsam::Point3(20.5147,7.4793,5.105779));
  // points.push_back(gtsam::Point3(20.5147,7.4793,5.105779));  
  // points.push_back(gtsam::Point3(20.5147,7.4793,0.105779));
  // points.push_back(gtsam::Point3(35.0,30.0,1.0));
  // points.push_back(gtsam::Point3(35.0,12.0,10.0));
  // points.push_back(gtsam::Point3(35.0,30.0,10.0));
  // points.push_back(gtsam::Point3(25.0,12.0,1.0));
  // points.push_back(gtsam::Point3(25.0,30.0,1.0));
  // points.push_back(gtsam::Point3(25.0,12.0,10.0));
  // points.push_back(gtsam::Point3(25.0,30.0,10.0));

  return points;
}

/* ************************************************************************* */
std::vector<gtsam::Pose3> createPoses(
            const gtsam::Pose3& init = gtsam::Pose3(gtsam::Rot3::Ypr(M_PI/2,0,0), gtsam::Point3(30, 0, 0)),
            const gtsam::Pose3& delta = gtsam::Pose3(gtsam::Rot3::Ypr(0,0,0), gtsam::Point3(0.5, 0, 0)), 
            int steps = 16) {
  
  // Create the set of ground-truth poses
  std::vector<gtsam::Pose3> poses;

  int i = 1;
  poses.push_back(init);
  for(; i < steps; ++i) {
    poses.push_back(poses[i-1].compose(delta));
    // poses[i].print();
  }
  
  return poses;
}

/* ************************************************************************* */

std::vector<gtsam::Pose3> createPosesDeviated(
            const gtsam::Pose3& init = gtsam::Pose3(gtsam::Rot3::Ypr(M_PI/2,0,0), gtsam::Point3(30, 0, 0)),
            int steps = 16) {
  
  // Create the set of ground-truth poses
  std::vector<gtsam::Pose3> deviatedPoses;
  // Random no generator for noise on odometry
  std::random_device rd;
  std::mt19937 gen(rd());

  int i = 1;
  gtsam::Pose3 delta_drifty;
  deviatedPoses.push_back(init);
  for(; i < steps; ++i) {
    std::normal_distribution<float> drift_rot(0.0, 0.02);
    std::normal_distribution<float> drift_t(0.0, 0.3);
    float rot_x = drift_rot(gen);
    float rot_y = drift_rot(gen);
    float rot_z = drift_rot(gen);
    float t_x = drift_t(gen);
    float t_y = drift_t(gen);
    float t_z = drift_t(gen);
    delta_drifty = gtsam::Pose3(gtsam::Rot3::Ypr(0+rot_x, 0+rot_y, 0+rot_z), gtsam::Point3(0.5+t_x, 0+t_y, 0+t_z));
    deviatedPoses.push_back(deviatedPoses[i-1].compose(delta_drifty));
    // deviatedPoses[i].print();
  }
  
  return deviatedPoses;
}


/* ************************************************************************* */
std::vector<gtsam::Pose3> createExtrinsics(
            const gtsam::Pose3& init = gtsam::Pose3(gtsam::Rot3::Ypr(0,0,0), gtsam::Point3(0, 0.1, 0)),
            const gtsam::Pose3& delta = gtsam::Pose3(gtsam::Rot3::Ypr(0,0,0), gtsam::Point3(0, 0, 0)), 
            int steps = 16) {
  
  // Create the set of ground-truth poses
  std::vector<gtsam::Pose3> poses;
  int i = 1;
  poses.push_back(init);
  for(; i < steps; ++i) {
    poses.push_back(poses[i-1].compose(delta));
    // poses[i].print();
  }
  
  return poses;
}

/* ************************************************************************* */
std::vector<gtsam::Pose3> createExtrinsicsDeviated(std::vector<gtsam::Pose3>& factoryExtrinsics, float R) {
  
  // Create the set of ground-truth poses
  std::vector<gtsam::Pose3> deviatedExtrinsics;

  // Set random deviation from factory values
  // srand (static_cast <unsigned> (time(0)));
  // float low = -0.02;
  // float high = 0.02;
  // float Rx = low + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(high-low)));
  // float Ry = low + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(high-low)));
  // float Rz = low + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(high-low)));
  // float tx = low + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(high-low)));
  // float ty = low + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(high-low)));
  // float tz = low + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(high-low)));
  // std::cout << Rx;
  // std::cout << Ry;
  // std::cout << Rz;

  float Rx = R;
  float Ry = R;
  float Rz = R;

  gtsam::Pose3 deviation = gtsam::Pose3(gtsam::Rot3::Ypr(Rz, Ry, Rx), gtsam::Point3(0, 0, 0));

  int i = 0;
  int steps = factoryExtrinsics.size();
  for(; i < steps; ++i) {
    deviatedExtrinsics.push_back(factoryExtrinsics[i].compose(deviation));
    // deviatedExtrinsics[i].print();
  }
  
  return deviatedExtrinsics;
}

