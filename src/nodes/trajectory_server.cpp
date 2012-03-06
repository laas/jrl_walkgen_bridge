#include <stdexcept>

#include <ros/ros.h>

#include <jrl_walkgen_bridge/kajita.hh>
#include <jrl_walkgen_bridge/morisawa.hh>

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

double getParam (const std::string& param,
		 const double& defaultValue)
{
  double result;
  ros::param::param<double> (param, result, defaultValue);
  return result;
}

template <typename T>
class TrajectoryServer :
  public walk_msgs::AbstractNode<T,
				 walk_msgs::Footprint2d,
				 walk_msgs::GetPath>
{
public:
  typedef walk_msgs::AbstractNode<T,
				  walk_msgs::Footprint2d,
				  walk_msgs::GetPath> parent_t;

  explicit TrajectoryServer (T& patternGenerator);
  ~TrajectoryServer ();

  virtual void convertFootprint
  (typename parent_t::patternGenerator_t::footprints_t& dst,
   const std::vector<typename parent_t::footprintRosType_t>& src);

  virtual void
  setupPatternGenerator (walk_msgs::GetPath::Request& req);
};

template <typename T>
TrajectoryServer<T>::TrajectoryServer (T& patternGenerator)
  : walk_msgs::AbstractNode<T,
			    walk_msgs::Footprint2d,
			    walk_msgs::GetPath>
    ("", getParam ("~world_frame_id", "/world"),
     patternGenerator,
     true)
{}

template <typename T>
TrajectoryServer<T>::~TrajectoryServer ()
{}

template <typename T>
void
TrajectoryServer<T>::convertFootprint
(typename parent_t::patternGenerator_t::footprints_t& dst,
 const std::vector<typename parent_t::footprintRosType_t>& src)
{
  using boost::posix_time::seconds;
  using boost::posix_time::milliseconds;

  dst.clear();
  std::vector<walk_msgs::Footprint2d>::const_iterator it = src.begin();
  for (; it != src.end(); ++it)
    {
      typename parent_t::patternGenerator_t::footprint_t footprint;
      footprint.beginTime = (it->beginTime).toBoost();
      footprint.duration =
	seconds(it->duration.sec) + milliseconds(it->duration.nsec * 1000);
      footprint.position(0) = it->x;
      footprint.position(1) = it->y;
      footprint.position(2) = it->theta;
      dst.push_back(footprint);
    }
}

template <typename T>
void
TrajectoryServer<T>::setupPatternGenerator (walk_msgs::GetPath::Request& req)
{
}


int main(int argc, char **argv)
{
  try
    {
      ros::init(argc, argv, "jrl_walkgen_bridge");

      std::string algorithm = getParam("algorithm", "Morisawa2007");
      if (algorithm == "Morisawa2007")
	{
	  ROS_INFO_STREAM
	    ("starting trajectory server using algorithm " << algorithm);
	  jrlWalkgenBridge::Morisawa2007 patternGenerator
	    (getParam ("robot_description", ""),
	     getParam ("~step", 0.005));
	  TrajectoryServer<jrlWalkgenBridge::Morisawa2007> node
	    (patternGenerator);
	  if (ros::ok())
	    node.spin();
	}
      else if (algorithm == "Kajita2003")
	{
	  ROS_INFO_STREAM
	    ("starting trajectory server using algorithm " << algorithm);
	  jrlWalkgenBridge::Kajita2003 patternGenerator
	    (getParam ("robot_description", ""));
	  TrajectoryServer<jrlWalkgenBridge::Kajita2003> node
	    (patternGenerator);
	  if (ros::ok())
	    node.spin();
	}
      else
	ROS_FATAL_STREAM("unsupported algorithm: " << algorithm);
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
