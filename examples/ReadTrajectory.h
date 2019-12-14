/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

#include <ctime>
#include <random>
#include <cstdlib>
#include <fstream>
#include <iostream>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/SimpleCamera.h>

/* ************************************************************************* */
std::vector<gtsam::Point3> readPoints(std::string const &feature_filepath) {

  // Create the set of ground-truth landmarks
  std::vector<gtsam::Point3> points;

  // read file
  std::ifstream file(feature_filepath);
  std::string str;
  if (!file.is_open()) {
      std::cout << "Error while opening feature file " + feature_filepath + "\n";
      exit(1);
  }
  // Write to gtsam point vector
  while (std::getline(file, str)) {
    std::string::size_type sz;

    float x = std::stof(str, &sz);
    str = str.substr(sz);
    float y = std::stof(str, &sz);
    str = str.substr(sz);
    float z = std::stof(str, &sz);
    // std::cout << x << " ";
    // std::cout << y << " ";
    // std::cout << z << "\n";
    points.push_back(gtsam::Point3(x,y,z));
  }
  return points;
}

/* ************************************************************************* */
gtsam::Point3 readTranslation(std::string const &fixed_translation_file_path) {
  
  // Create the fixed ground truth for the translation vector
  gtsam::Point3 extrinsic_translation;

  // read file
  std::ifstream file(fixed_translation_file_path);
  std::string str;
  if (!file.is_open()) {
      std::cout << "Error while opening feature file " + fixed_translation_file_path + "\n";
      exit(1);
  }
  // Write to gtsam point vector
  while (std::getline(file, str)) {
    std::string::size_type sz;

    float x = std::stof(str, &sz);
    str = str.substr(sz);
    float y = std::stof(str, &sz);
    str = str.substr(sz);
    float z = std::stof(str, &sz);
    extrinsic_translation = gtsam::Point3(x,y,z);
  }
  return extrinsic_translation;
}

/* ************************************************************************* */
std::vector<gtsam::Pose3> readPoses(std::string const &pose_filepath) {
  
  // Create the set of ground-truth poses
  std::vector<gtsam::Pose3> poses;

  // read file
  std::ifstream file(pose_filepath);
  std::string str;
  if (!file.is_open()) {
      std::cout << "Error while opening pose file " + pose_filepath + "\n";
      exit(1);
  }
  // Write to gtsam poses vector
  // std::cout<< "\n\n";
  while (std::getline(file, str)) {
    std::string::size_type sz;

    float yaw = std::stof(str, &sz);
    str = str.substr(sz);
    float pitch = std::stof(str, &sz);
    str = str.substr(sz);
    float roll = std::stof(str, &sz);
    str = str.substr(sz);
    float x = std::stof(str, &sz);
    str = str.substr(sz);
    float y = std::stof(str, &sz);
    str = str.substr(sz);
    float z = std::stof(str, &sz);
    gtsam::Pose3 pose = gtsam::Pose3(gtsam::Rot3::Ypr(yaw, pitch, roll), gtsam::Point3(x, y, z));
    // pose.print();
    poses.push_back(pose);
  }
  return poses;
}
/* ************************************************************************* */
