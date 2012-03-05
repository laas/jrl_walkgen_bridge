#include <stdexcept>
#include <string>
#include <vector>
#include <boost/assign/list_of.hpp>
#include <boost/date_time.hpp>
#include <boost/foreach.hpp>
#include <boost/math/constants/constants.hpp>

#include <Eigen/LU>

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

  Morisawa2007::Morisawa2007 (const std::string& robotDescription)
    : walk::PatternGenerator2d(),
      robot_ (loadRobot (robotDescription)),
      pgi_ (PatternGeneratorJRL::patternGeneratorInterfaceFactory
	    (&*robot_))
  {
    ROS_ASSERT(pgi_);

    // Common pattern generator initialization.
    using boost::assign::list_of;
    std::vector<std::string> buffer =
      list_of (":comheight 0.8078")
      (":samplingperiod 0.005")
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
      (":onlinechangestepframe relative")
      (":SetAutoFirstStep false");

    BOOST_FOREACH(const std::string& s, buffer)
      {
	std::istringstream stream(s);
	pgi_->ParseCmd(stream);
      }

    //FIXME: in jrl-walkgen test, evaluateStartPosition
    // is called here.
  }

  Morisawa2007::Morisawa2007(const Morisawa2007& pg)
    : walk::PatternGenerator2d(),
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
    robot_ = rhs.robot_;
    pgi_ = rhs.pgi_;
    return *this;
  }
  void
  Morisawa2007::computeTrajectories()
  {
    ROS_ASSERT (pgi_);
    //FIXME: don't bother with reading the input for now.
    std::istringstream stream
      (":StartOnLineStepSequencing"
       " 0.0 -0.095 0.0 0.0 0.19 0.0 0.0 -0.19 0.0 0.0 0.19 0.0");
    pgi_->ParseCmd(stream);
    std::cout << "OK OK OK" << std::endl;
  }
} // end of namespace jrlWalkgenBridge.
