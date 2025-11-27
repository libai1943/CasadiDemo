#pragma once
#include <ros/ros.h>

namespace common {
namespace util {

inline double GetCurrentTimestamp() {
  return ros::Time::now().toSec();
}

}
}
