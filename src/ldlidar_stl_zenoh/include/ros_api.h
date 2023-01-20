#ifndef __ROS_API_H__
#define __ROS_API_H__

#include <sensor_msgs/LaserScan.h>
#include <string>

struct LaserScanSetting
{
  std::string frame_id;
  bool laser_scan_dir;
  bool enable_angle_crop_func;
  double angle_crop_min;
  double angle_crop_max;
};

#endif //__ROS_API_H__
