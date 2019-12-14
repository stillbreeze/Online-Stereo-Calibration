#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <boost/optional.hpp>

// #include "examples/ReadTrajectory.h"

namespace gtsam {

  /**
   * Non-linear factor for a constraint derived from a 2D measurement. The calibration is known here.
   * i.e. the main building block for visual SLAM.
   * @addtogroup SLAM
   */
  template<class POSE, class LANDMARK, class ROT, class CALIBRATION = Cal3_S2>
  class ProjectionFactorCalibrationRotationOnly: public NoiseModelFactor3<POSE, ROT, LANDMARK> {
  protected:

    // Keep a copy of measurement and calibration for I/O
    Point2 measured_;                    ///< 2D measurement
    boost::shared_ptr<CALIBRATION> K_;  ///< shared pointer to calibration object

    // verbosity handling for Cheirality Exceptions
    bool throwCheirality_; ///< If true, rethrows Cheirality exceptions (default: false)
    bool verboseCheirality_; ///< If true, prints text for Cheirality exceptions (default: false)

  public:

    /// shorthand for base class type
    typedef NoiseModelFactor3<POSE, ROT, LANDMARK> Base;

    /// shorthand for this class
    typedef ProjectionFactorCalibrationRotationOnly<POSE, LANDMARK, ROT, CALIBRATION> This;

    /// shorthand for a smart pointer to a factor
    typedef boost::shared_ptr<This> shared_ptr;

    /// Default constructor
  ProjectionFactorCalibrationRotationOnly() :
      measured_(0.0, 0.0), throwCheirality_(false), verboseCheirality_(false) {
  }

    /**
     * Constructor
     * TODO: Mark argument order standard (keys, measurement, parameters)
     * @param measured is the 2 dimensional location of point in image (the measurement)
     * @param model is the standard deviation
     * @param poseKey is the index of the camera
     * @param transformKey is the index of the body-camera transform
     * @param pointKey is the index of the landmark
     * @param K shared pointer to the constant calibration
     */
    ProjectionFactorCalibrationRotationOnly(const Point2& measured, const SharedNoiseModel& model,
        Key poseKey, Key transformKey,  Key pointKey,
        const boost::shared_ptr<CALIBRATION>& K) :
          Base(model, poseKey, transformKey, pointKey), measured_(measured), K_(K),
          throwCheirality_(false), verboseCheirality_(false) {}

    /**
     * Constructor with exception-handling flags
     * TODO: Mark argument order standard (keys, measurement, parameters)
     * @param measured is the 2 dimensional location of point in image (the measurement)
     * @param model is the standard deviation
     * @param poseKey is the index of the camera
     * @param pointKey is the index of the landmark
     * @param K shared pointer to the constant calibration
     * @param throwCheirality determines whether Cheirality exceptions are rethrown
     * @param verboseCheirality determines whether exceptions are printed for Cheirality
     */
    ProjectionFactorCalibrationRotationOnly(const Point2& measured, const SharedNoiseModel& model,
        Key poseKey, Key transformKey, Key pointKey,
        const boost::shared_ptr<CALIBRATION>& K,
        bool throwCheirality, bool verboseCheirality) :
          Base(model, poseKey, transformKey, pointKey), measured_(measured), K_(K),
          throwCheirality_(throwCheirality), verboseCheirality_(verboseCheirality) {}

    /** Virtual destructor */
    virtual ~ProjectionFactorCalibrationRotationOnly() {}

    /// @return a deep copy of this factor
    virtual NonlinearFactor::shared_ptr clone() const {
      return boost::static_pointer_cast<NonlinearFactor>(
          NonlinearFactor::shared_ptr(new This(*this))); }

    /**
     * print
     * @param s optional string naming the factor
     * @param keyFormatter optional formatter useful for printing Symbols
     */
    void print(const std::string& s = "", const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
      std::cout << s << "ProjectionFactorCalibrationRotationOnly, z = ";
      traits<Point2>::Print(measured_);
      Base::print("", keyFormatter);
    }

    /// equals
    virtual bool equals(const NonlinearFactor& p, double tol = 1e-9) const {
      const This *e = dynamic_cast<const This*>(&p);
      return e
          && Base::equals(p, tol)
          && traits<Point2>::Equals(this->measured_, e->measured_, tol)
          && this->K_->equals(*e->K_, tol);
    }

    /// Evaluate error h(x)-z and optionally derivatives
    // !!Translation value hardcoded!!
    Vector evaluateError(const Pose3& pose, const Rot3& transform, const Point3& point,
        boost::optional<Matrix&> H1 = boost::none,
        boost::optional<Matrix&> H2 = boost::none,
        boost::optional<Matrix&> H3 = boost::none) const {
      try {
          // HARDCODED PATH FOR FIXED TRANSLATION VECTOR
          std::string fixed_translation_path = "../../examples/Data/StereoSelfCalibration/FixedTranslationVector.txt";
          Point3 fixed_extrinsic_translation =  readTranslation(fixed_translation_path);

          if(H1 || H2 || H3) {
            Matrix H0, H02, HRt;
            Pose3 extrinsics = Pose3(transform, fixed_extrinsic_translation);
            PinholeCamera<CALIBRATION> camera(pose.compose(extrinsics, H0, H02), *K_);
            Point2 reprojectionError(camera.project(point, H1, H3, boost::none) - measured_);
            HRt = *H1 * H02;
            *H2 = HRt.block<2,3>(0,0);
            *H1 = *H1 * H0;
            return reprojectionError;
          } else {
            Pose3 extrinsics = Pose3(transform, fixed_extrinsic_translation);
            PinholeCamera<CALIBRATION> camera(pose.compose(extrinsics), *K_);
            return camera.project(point, H1, H3, boost::none) - measured_;
          }
      } catch( CheiralityException& e) {
        if (H1) *H1 = Matrix::Zero(2,6);
        if (H2) *H2 = Matrix::Zero(2,3);
        if (H3) *H3 = Matrix::Zero(2,3);
        if (verboseCheirality_)
          std::cout << e.what() << ": Landmark "<< DefaultKeyFormatter(this->key2()) <<
              " moved behind camera " << DefaultKeyFormatter(this->key1()) << std::endl;
        if (throwCheirality_)
          throw e;
      }
      return Vector::Ones(2) * 2.0 * K_->fx();
    }

    /** return the measurement */
    const Point2& measured() const {
      return measured_;
    }

    /** return the calibration object */
    inline const boost::shared_ptr<CALIBRATION> calibration() const {
      return K_;
    }

    /** return verbosity */
    inline bool verboseCheirality() const { return verboseCheirality_; }

    /** return flag for throwing cheirality exceptions */
    inline bool throwCheirality() const { return throwCheirality_; }

  private:

    /// Serialization function
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
      ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
      ar & BOOST_SERIALIZATION_NVP(measured_);
      ar & BOOST_SERIALIZATION_NVP(K_);
      ar & BOOST_SERIALIZATION_NVP(throwCheirality_);
      ar & BOOST_SERIALIZATION_NVP(verboseCheirality_);
    }
  };

  /// traits
  template<class POSE, class LANDMARK, class ROT, class CALIBRATION>
  struct traits<ProjectionFactorCalibrationRotationOnly<POSE, LANDMARK, ROT, CALIBRATION> > :
      public Testable<ProjectionFactorCalibrationRotationOnly<POSE, LANDMARK, ROT, CALIBRATION> > {
  };

} // \ namespace gtsam
