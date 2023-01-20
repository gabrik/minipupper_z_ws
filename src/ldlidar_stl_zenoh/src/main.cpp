/**
 * @file main.cpp
 * @author LDRobot (marketing1@ldrobot.com)
 * @brief  main process App
 *         This code is only applicable to LDROBOT LiDAR LD06 products
 * sold by Shenzhen LDROBOT Co., LTD
 * @version 0.1
 * @date 2021-10-28
 *
 * @copyright Copyright (c) 2021  SHENZHEN LDROBOT CO., LTD. All rights
 * reserved.
 * Licensed under the MIT License (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License in the file LICENSE
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "ros_api.h"
#include "ldlidar_driver.h"
#include <argparse/argparse.hpp>
#include <cstdio>
#include <chrono>
#include <thread>




extern "C" {
  #include "zenoh-pico.h"
}

void  ToLaserscanMessagePublish(ldlidar::Points2D& src, double lidar_spin_freq,
    LaserScanSetting& setting, z_owned_publisher_t& z_lidar_pub);

uint64_t GetSystemTimeStamp(void);



// add parameter parsing instead of ros parameters to master

int main(int argc, char **argv) {
  ldlidar::LDLidarDriver* ldlidarnode = new ldlidar::LDLidarDriver();
  std::string product_name;
	std::string topic_name;
	std::string port_name;
  std::string frame_id;
  int serial_port_baudrate;
  bool laser_scan_dir;
  bool enable_angle_crop_func;
  double angle_crop_min;
  double angle_crop_max;
  LaserScanSetting setting;
  ldlidar::LDType type_name;

  std::string mode;
  std::string locator;
  z_owned_session_t z_session;
  z_owned_publisher_t z_scan_publisher;


  argparse::ArgumentParser program("zenoh_lidar");


  program.add_argument("--product_name")
    .help("Lidar product name")
    .default_value(std::string("LDLiDAR_LD06"));

  program.add_argument("--topic_name")
    .help("topic where data is being published")
    .default_value(std::string("scan"));

  program.add_argument("--frame_id")
    .help("display the square of a given integer")
    .default_value(std::string("base_laser"));

  program.add_argument("--port_name")
    .help("lidar serial port")
    .default_value(std::string("/dev/ttyUSB0"));

  program.add_argument("--port_baudrate")
    .help("lidar serial baudrate")
    .default_value(int(230400));

  program.add_argument("--no_laser_scan_dir")
    .help("lidar scan direction")
    .default_value(bool(true))
    .implicit_value(false);

  program.add_argument("--enable_angle_crop_func")
    .help("enable or disable angle crop")
    .default_value(bool(false))
    .implicit_value(true);

  program.add_argument("--angle_crop_min")
    .help("angle crop start (radiants)")
    .default_value(double(0.0));

  program.add_argument("--angle_crop_max")
    .help("angle crop end (radiants)")
    .default_value(double(0.0));

  program.add_argument("--mode")
    .help("angle crop end (radiants)")
    .default_value(std::string("client"));

  program.add_argument("--connect")
    .help("angle crop end (radiants)")
    .default_value(std::string("tcp/127.0.0.1:7447"));


  try {
    program.parse_args(argc, argv);
  }
  catch (const std::runtime_error& err) {
    std::cerr << err.what() << std::endl;
    std::cerr << program;
    std::exit(1);
  }

  product_name = program.get<std::string>("--product_name");
  topic_name = program.get<std::string>("--topic_name");
  frame_id = program.get<std::string>("--frame_id");
  port_name = program.get<std::string>("--port_name");
  serial_port_baudrate = program.get<int>("--port_baudrate");
  laser_scan_dir = program.get<bool>("--laser_scan_dir");
  enable_angle_crop_func = program.get<bool>("--enable_angle_crop_func");
  angle_crop_min = program.get<double>("--angle_crop_min");
  angle_crop_max = program.get<double>("--angle_crop_max");
  mode = program.get<std::string>("--mode");
  locator = program.get<std::string>("--topic_nlocatorame");



  printf("LDLiDAR SDK Pack Version is: %s", ldlidarnode->GetLidarSdkVersionNumber().c_str());
  printf("ROS params input:");
  printf("<product_name>: %s", product_name.c_str());
  printf("<topic_name>: %s", topic_name.c_str());
  printf("<frame_id>: %s", setting.frame_id.c_str());
  printf("<port_name>: %s", port_name.c_str());
  printf("<port_baudrate>: %d", serial_port_baudrate);
  printf("<laser_scan_dir>: %s", (setting.laser_scan_dir?"Counterclockwise":"Clockwise"));
  printf("<enable_angle_crop_func>: %s", (setting.enable_angle_crop_func?"true":"false"));
  printf("<angle_crop_min>: %f", setting.angle_crop_min);
  printf("<angle_crop_max>: %f", setting.angle_crop_max);

  if (product_name == "LDLiDAR_LD06") {
    type_name = ldlidar::LDType::LD_06;
  } else if (product_name == "LDLiDAR_LD19") {
    type_name = ldlidar::LDType::LD_19;
  } else {
    printf("Error, input <product_name> is illegal.");
    exit(EXIT_FAILURE);
  }

  ldlidarnode->RegisterGetTimestampFunctional(std::bind(&GetSystemTimeStamp));

  ldlidarnode->EnableFilterAlgorithnmProcess(true);

  if (ldlidarnode->Start(type_name, port_name, serial_port_baudrate, ldlidar::COMM_SERIAL_MODE)) {
    printf("ldlidar node start is success");
  } else {
    printf("ldlidar node start is fail");
    exit(EXIT_FAILURE);
  }

  if (ldlidarnode->WaitLidarCommConnect(3000)) {
    printf("ldlidar communication is normal.");
  } else {
    printf("ldlidar communication is abnormal.");
    exit(EXIT_FAILURE);
  }

    z_owned_config_t z_config = z_config_default();

    // Default config for the time being
    zp_config_insert(z_config_loan(&z_config), Z_CONFIG_MODE_KEY, z_string_make(mode.c_str()));
    zp_config_insert(z_config_loan(&z_config), Z_CONFIG_PEER_KEY, z_string_make(locator.c_str()));


    z_session = z_open(z_config_move(&z_config));
    if (!z_session_check(&z_session)) {
      printf("Unable to open session!\n");
        exit(EXIT_FAILURE);
    }

    // Start read and lease tasks for zenoh-pico
    if (zp_start_read_task(z_session_loan(&z_session), NULL) < 0 || zp_start_lease_task(z_session_loan(&z_session), NULL) < 0) {
        printf("Unable to start read and lease tasks");
        exit(EXIT_FAILURE);
    }

    z_scan_publisher = z_declare_publisher(z_session_loan(&z_session), z_keyexpr(topic_name.c_str()), NULL);
    if (!z_publisher_check(&z_scan_publisher)) {
            printf("Unable to declare publisher for: %s!\n", topic_name.c_str());
            exit(EXIT_FAILURE);
    }


  ldlidar::Points2D laser_scan_points;
  double lidar_scan_freq;
  printf("Publish topic message:ldlidar scan data .");

  while (true) {

    switch (ldlidarnode->GetLaserScanData(laser_scan_points, 1500)){
      case ldlidar::LidarStatus::NORMAL:
        ldlidarnode->GetLidarScanFreq(lidar_scan_freq);
        ToLaserscanMessagePublish(laser_scan_points, lidar_scan_freq, setting, z_scan_publisher );
        break;
      case ldlidar::LidarStatus::DATA_TIME_OUT:
        printf("get ldlidar data is time out, please check your lidar device.");
        break;
      case ldlidar::LidarStatus::DATA_WAIT:
        break;
      default:
        break;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100)); //10 Hz
  }

  return 0;
}

void  ToLaserscanMessagePublish(ldlidar::Points2D& src, double lidar_spin_freq,
    LaserScanSetting& setting, z_owned_publisher_t& z_lidar_pub) {
  float angle_min, angle_max, range_min, range_max, angle_increment;
  float scan_time;
  ros::Time start_scan_time;
  static ros::Time end_scan_time;
  static bool first_scan = true;

  start_scan_time = ros::Time::now();
  scan_time = (start_scan_time - end_scan_time).toSec();

  if (first_scan) {
    first_scan = false;
    end_scan_time = start_scan_time;
    return;
  }

  // Adjust the parameters according to the demand
  angle_min = 0;
  angle_max = (2 * M_PI);
  range_min = 0.02;
  range_max = 12;
  int beam_size = static_cast<int>(src.size());
  angle_increment = (angle_max - angle_min) / (float)(beam_size -1);

  // Calculate the number of scanning points
  if (lidar_spin_freq > 0) {
    sensor_msgs::LaserScan output;
    output.header.stamp = start_scan_time;
    output.header.frame_id = setting.frame_id;
    output.angle_min = angle_min;
    output.angle_max = angle_max;
    output.range_min = range_min;
    output.range_max = range_max;
    output.angle_increment = angle_increment;
    if (beam_size <= 1) {
      output.time_increment = 0;
    } else {
      output.time_increment = scan_time / (float)(beam_size - 1);
    }
    output.scan_time = scan_time;
    // First fill all the data with Nan
    output.ranges.assign(beam_size, std::numeric_limits<float>::quiet_NaN());
    output.intensities.assign(beam_size, std::numeric_limits<float>::quiet_NaN());

    for (auto point : src) {
      float range = point.distance / 1000.f;  // distance unit transform to meters
      float intensity = point.intensity;      // laser receive intensity
      float dir_angle = point.angle;

      if ((point.distance == 0) && (point.intensity == 0)) { // filter is handled to  0, Nan will be assigned variable.
        range = std::numeric_limits<float>::quiet_NaN();
        intensity = std::numeric_limits<float>::quiet_NaN();
      }

      if (setting.enable_angle_crop_func) { // Angle crop setting, Mask data within the set angle range
        if ((dir_angle >= setting.angle_crop_min) && (dir_angle <= setting.angle_crop_max)) {
          range = std::numeric_limits<float>::quiet_NaN();
          intensity = std::numeric_limits<float>::quiet_NaN();
        }
      }

      float angle = ANGLE_TO_RADIAN(dir_angle); // Lidar angle unit form degree transform to radian
      int index = static_cast<int>(ceil((angle - angle_min) / angle_increment));
      if (index < beam_size) {
        if (index < 0) {
          printf("[ldrobot] error index: %d, beam_size: %d, angle: %f, angle_min: %f, angle_increment: %f",
              index, beam_size, angle, angle_min, angle_increment);
        }

        if (setting.laser_scan_dir) {
          int index_anticlockwise = beam_size - index - 1;
          // If the current content is Nan, it is assigned directly
          if (std::isnan(output.ranges[index_anticlockwise])) {
            output.ranges[index_anticlockwise] = range;
          } else { // Otherwise, only when the distance is less than the current
                    //   value, it can be re assigned
            if (range < output.ranges[index_anticlockwise]) {
                output.ranges[index_anticlockwise] = range;
            }
          }
          output.intensities[index_anticlockwise] = intensity;
        } else {
          // If the current content is Nan, it is assigned directly
          if (std::isnan(output.ranges[index])) {
            output.ranges[index] = range;
          } else { // Otherwise, only when the distance is less than the current
                  //   value, it can be re assigned
            if (range < output.ranges[index]) {
              output.ranges[index] = range;
            }
          }
          output.intensities[index] = intensity;
        }
      }
    }

    // Here we serialize and publish over zenoh
    uint32_t ser_size = ros::serialization::serializationLength(output);
    uint8_t *buffer = (uint8_t*) std::calloc((size_t)ser_size, sizeof(uint8_t));
    ros::serialization::OStream ostream(buffer, ser_size);
    ros::serialization::serialize(ostream, output);

     z_publisher_put_options_t options = z_publisher_put_options_default();
    options.encoding = z_encoding(Z_ENCODING_PREFIX_APP_OCTET_STREAM, NULL);

    z_publisher_put(z_publisher_loan(&z_lidar_pub), (const uint8_t*) buffer, (size_t) ser_size, &options);

    // lidarpub.publish(output);
    end_scan_time = start_scan_time;


  }
}

uint64_t GetSystemTimeStamp(void) {
  std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> tp =
    std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now());
  auto tmp = std::chrono::duration_cast<std::chrono::nanoseconds>(tp.time_since_epoch());
  return ((uint64_t)tmp.count());
}

/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/
