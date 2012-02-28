#include <stdexcept>

#include <ros/ros.h>

#include <jrl_walkgen_bridge/kajita.hh>

#include <walk_interfaces/yaml.hh>
#include <walk_msgs/abstract-node.hh>

using walk::HomogeneousMatrix3d;
using walk::Posture;


std::string getParam (const std::string& param,
		      const std::string& defaultValue)
{
  std::string result;
  ros::param::param(param, result, defaultValue);
  return result;
}

class KajitaTrajectoryServer :
  public walk_msgs::AbstractNode<jrlWalkgenBridge::Kajita2003,
				 walk_msgs::Footprint2d,
				 walk_msgs::GetPath>
{
public:
  explicit KajitaTrajectoryServer ();
  ~KajitaTrajectoryServer ();

  virtual void convertFootprint
  (patternGenerator_t::footprints_t& dst,
   const std::vector<footprintRosType_t>& src);

  virtual void
  setupPatternGenerator (walk_msgs::GetPath::Request& req);
};

KajitaTrajectoryServer::KajitaTrajectoryServer ()
  : walk_msgs::AbstractNode<jrlWalkgenBridge::Kajita2003,
			    walk_msgs::Footprint2d,
			    walk_msgs::GetPath>
    ("", getParam ("~world_frame_id", "/world"),
     jrlWalkgenBridge::Kajita2003 (getParam ("robot_description", "")),
     true)
{}

KajitaTrajectoryServer::~KajitaTrajectoryServer ()
{}

void
KajitaTrajectoryServer::convertFootprint
(patternGenerator_t::footprints_t& dst,
 const std::vector<footprintRosType_t>& src)
{
  using boost::posix_time::seconds;
  using boost::posix_time::milliseconds;

  dst.clear();
  std::vector<walk_msgs::Footprint2d>::const_iterator it = src.begin();
  for (; it != src.end(); ++it)
    {
      patternGenerator_t::footprint_t footprint;
      footprint.beginTime = (it->beginTime).toBoost();
      footprint.duration =
	seconds(it->duration.sec) + milliseconds(it->duration.nsec * 1000);
      footprint.position(0) = it->x;
      footprint.position(1) = it->y;
      footprint.position(2) = it->theta;
      dst.push_back(footprint);
    }
}

void
KajitaTrajectoryServer::setupPatternGenerator (walk_msgs::GetPath::Request& req)
{
}


int main(int argc, char **argv)
{
  try
    {
      ros::init(argc, argv, "jrl_walkgen_bridge");

      KajitaTrajectoryServer node;
      if (ros::ok())
	node.spin();
    }
  catch (std::exception& e)
    {
      std::cerr << "fatal error: " << e.what() << std::endl;
      ROS_ERROR_STREAM("fatal error: " << e.what());
      return 1;
    }
  catch (...)
    {
      ROS_ERROR_STREAM("unexpected error");
      return 2;
    }
  return 0;
}
