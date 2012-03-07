#include <stdexcept>
#include <string>
#include <vector>
#include <boost/assign/list_of.hpp>
#include <boost/date_time.hpp>
#include <boost/foreach.hpp>
#include <boost/math/constants/constants.hpp>

#include <Eigen/LU>
#include <LinearMath/btMatrix3x3.h>

#include "jrl_walkgen_bridge/morisawa.hh"

#include <angles/angles.h>
#include <jrl/dynamics/urdf/parser.hh>
#include <walk_interfaces/util.hh>
#include <walk_msgs/conversion.hh>
#include <walk_msgs/Footprint2d.h>

namespace jrlWalkgenBridge
{
  // Use jrl_dynamics_urdf to load the robot.
  // This will throw if the robot loading fail.
  static CjrlHumanoidDynamicRobot*
  loadRobot (const std::string& robotDescription)
  {
    jrl::dynamics::urdf::Parser parser;
    return parser.parseStream (robotDescription, "root_joint");
  }

  namespace
  {
    static std::string makeCommand (const std::string& key, double value)
    {
      boost::format fmt (":%1% %2%");
      fmt % key % value;
      return fmt.str ();
    }
  } // end of anonymous namespace.

  Morisawa2007::Morisawa2007 (const std::string& robotDescription,
			      const double& step)
    : walk::PatternGenerator2d(),
      step_ (step),
      robot_ (loadRobot (robotDescription)),
      pgi_ (PatternGeneratorJRL::patternGeneratorInterfaceFactory
	    (&*robot_))
  {
    ROS_ASSERT(pgi_);
  }

  Morisawa2007::Morisawa2007(const Morisawa2007& pg)
    : walk::PatternGenerator2d(),
      step_ (pg.step_),
      robot_ (pg.robot_),
      pgi_ (pg.pgi_)
  {}

  Morisawa2007::~Morisawa2007()
  {}

  Morisawa2007&
  Morisawa2007::operator= (const Morisawa2007& rhs)
  {
    if (this == &rhs)
      return *this;
    step_ = rhs.step_;
    robot_ = rhs.robot_;
    pgi_ = rhs.pgi_;
    return *this;
  }
  void
  Morisawa2007::computeTrajectories()
  {
    ROS_ASSERT (pgi_);

    // Common pattern generator initialization.
    using boost::assign::list_of;
    std::vector<std::string> buffer =
      list_of
      (makeCommand
       ("comheight", initialCenterOfMassPosition()[2]).c_str ())
      (makeCommand ("samplingperiod", step_).c_str ())
      (":previewcontroltime 1.6")
      (":omega 0.0")
      (":stepheight 0.07")
      (":singlesupporttime 0.78")
      (":doublesupporttime 0.02")
      (":armparameters 0.5")
      (":LimitsFeasibility 0.0")
      (":ZMPShiftParameters 0.015 0.015 0.015 0.015")
      (":TimeDistributionParameters 2.0 3.7 1.7 3.0")
      (":UpperBodyMotionParameters -0.1 -1.0 0.0")

      // Dedicated initialization.
      (":SetAlgoForZmpTrajectory Morisawa")
      (":SetAutoFirstStep true");

    BOOST_FOREACH(const std::string& s, buffer)
      {
	std::istringstream stream(s);
	pgi_->ParseCmd(stream);
      }

    //FIXME: in jrl-walkgen test, evaluateStartPosition
    // is called here.

    // Compute step sequence.
    std::string stepSequence;

    walk::HomogeneousMatrix3d world_M_footprint;
    if (startWithLeftFoot())
      {
	world_M_footprint = initialRightFootPosition ();
	stepSequence += (boost::format("%1% %2% %3% ")
			 % initialRightFootPosition () (0, 3)
			 % initialRightFootPosition () (1, 3)
			 % std::atan2 (initialRightFootPosition () (1, 0),
				       initialRightFootPosition () (0, 0))
			 ).str ();
      }
    else
      {
	world_M_footprint = initialLeftFootPosition ();
	stepSequence += (boost::format("%1% %2% %3% ")
			 % initialLeftFootPosition () (0, 3)
			 % initialLeftFootPosition () (1, 3)
			 % std::atan2 (initialLeftFootPosition () (1, 0),
				       initialLeftFootPosition () (0, 0))
			 ).str ();
      }

    typedef walk::PatternGenerator2d::footprint_t footprint_t;
    BOOST_FOREACH (footprint_t footprint, footprints ())
      {
	walk::HomogeneousMatrix3d world_M_newFootprint;
	world_M_newFootprint.setIdentity ();
	for (unsigned i = 0; i < 2; ++i)
	  world_M_newFootprint (i, 3) = footprint.position (i);

	walk::HomogeneousMatrix3d footprint_M_newFootprint =
	  world_M_footprint.inverse () * world_M_newFootprint;

	boost::format fmt ("%1% %2% %3% ");
	fmt
	  % footprint_M_newFootprint (0, 3)
	  % footprint_M_newFootprint (1, 3)
	  % std::atan2 (footprint_M_newFootprint (1, 0),
			footprint_M_newFootprint (0, 0));
	stepSequence += fmt.str ();

	world_M_footprint = world_M_newFootprint;
      }

    ROS_DEBUG_STREAM ("step sequence: " << stepSequence);

    stepSequence = ":stepseq " + stepSequence;
    buffer.push_back (stepSequence.c_str ());
    BOOST_FOREACH(const std::string& s, buffer)
      {
	std::istringstream stream(s);
	pgi_->ParseCmd(stream);
      }

    MAL_VECTOR_DIM (configuration, double, robot_->numberDof ());
    MAL_VECTOR_DIM (velocity, double, robot_->numberDof ());
    MAL_VECTOR_DIM (acceleration, double, robot_->numberDof ());
    MAL_VECTOR_DIM (zmp, double, 3);
    PatternGeneratorJRL::COMState com;
    PatternGeneratorJRL::FootAbsolutePosition leftFoot;
    PatternGeneratorJRL::FootAbsolutePosition rightFoot;

    using boost::posix_time::milliseconds;

    getCenterOfMassTrajectory ().data ().clear ();
    while (pgi_->RunOneStepOfTheControlLoop
	   (configuration, velocity, acceleration, zmp, com,
	    leftFoot, rightFoot))
      {
	walk::Trajectory3d::element_t leftFootElement;
	btMatrix3x3 leftFootRotation;
	walk::Trajectory3d::element_t rightFootElement;
	btMatrix3x3 rightFootRotation;
	walk::TrajectoryV3d::element_t comElement;
	walk::TrajectoryV2d::element_t zmpElement;

	double yaw = 0.;
	double pitch = 0.;
	double roll = 0.;

	// Left foot.
	leftFootElement.duration = milliseconds (step_ * 1e3);
	leftFootElement.position.setIdentity ();
	leftFootElement.position (0, 3) =
	  leftFoot.x;
	leftFootElement.position (1, 3) =
	  leftFoot.y;
	leftFootElement.position (2, 3) =
	  leftFoot.z;
	yaw = angles::from_degrees (leftFoot.theta);
	pitch = angles::from_degrees (leftFoot.omega2);
	roll = angles::from_degrees (leftFoot.omega);
	leftFootRotation.setEulerYPR (yaw, pitch, roll);
	for (unsigned i = 0; i < 3; ++i)
	  for (unsigned j = 0; j < 3; ++j)
	    leftFootElement.position (i, j) = leftFootRotation[i][j];
	getLeftFootTrajectory().data ().push_back (leftFootElement);

	// Right foot.
	rightFootElement.duration = milliseconds (step_ * 1e3);
	rightFootElement.position.setIdentity ();
	rightFootElement.position (0, 3) =
	  rightFoot.x;
	rightFootElement.position (1, 3) =
	  rightFoot.y;
	rightFootElement.position (2, 3) =
	  rightFoot.z;
	yaw = angles::from_degrees (rightFoot.theta);
	pitch = angles::from_degrees (rightFoot.omega2);
	roll = angles::from_degrees (rightFoot.omega);
	rightFootRotation.setEulerYPR (yaw, pitch, roll);
	for (unsigned i = 0; i < 3; ++i)
	  for (unsigned j = 0; j < 3; ++j)
	    rightFootElement.position (i, j) = rightFootRotation[i][j];
	getRightFootTrajectory().data ().push_back (rightFootElement);

	// Center of mass.
	comElement.duration = milliseconds (step_ * 1e3);
	comElement.position[0] = com.x[0];
	comElement.position[1] = com.y[0];
	comElement.position[2] = com.z[0];
	getCenterOfMassTrajectory ().data ().push_back (comElement);

	// ZMP
	zmpElement.duration = milliseconds (step_ * 1e3);
	zmpElement.position[0] = zmp (0);
	zmpElement.position[1] = zmp (1);
	getZmpTrajectory ().data ().push_back (zmpElement);

	// FIXME: posture.
      }
  }
} // end of namespace jrlWalkgenBridge.
