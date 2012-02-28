#include <stdexcept>
#include <boost/date_time.hpp>
#include <boost/math/constants/constants.hpp>

#include <Eigen/LU>

#include "jrl_walkgen_bridge/kajita.hh"

#include <angles/angles.h>
#include <walk_interfaces/util.hh>
#include <walk_msgs/conversion.hh>
#include <walk_msgs/Footprint2d.h>

namespace jrlWalkgenBridge
{
  Kajita2003::Kajita2003 ()
    : walk::PatternGenerator2d(),
      pgi_ ()
  {
  }

  Kajita2003::Kajita2003(const Kajita2003&)
    : walk::PatternGenerator2d(),
      pgi_ ()
  {}

  Kajita2003::~Kajita2003()
  {}

  Kajita2003&
  Kajita2003::operator= (const Kajita2003& rhs)
  {
    if (this == &rhs)
      return *this;
    return *this;
  }
  void
  Kajita2003::computeTrajectories()
  {
    //FIXME
  }
} // end of namespace jrlWalkgenBridge.

