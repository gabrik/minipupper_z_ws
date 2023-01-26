#include "ros_api.h"

#include <zenoh.h>

void z_scan_cb(const z_sample_t *sample, void *ctx) {
    sensor_msgs::LaserScan* msg = (sensor_msgs::LaserScan*) ctx;

    uint8_t *buffer = (uint8_t*) std::calloc(sample->payload.len, sizeof(uint8_t));

    std::memcpy(buffer, sample->payload.start, sample->payload.len);

    ros::serialization::IStream de_stream(buffer, (uint32_t) sample->payload.len);
    ros::serialization::deserialize(de_stream, *msg);

}


void  ToLaserscanMessagePublish(ros::Publisher& lidarpub, sensor_msgs::LaserScan& output) {
    lidarpub.publish(output);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "ldldiar_publisher");
  ros::NodeHandle nh;  // create a ROS Node
  ros::NodeHandle nh_private("~");
  std::string product_name;
	std::string topic_name;
	std::string port_name;
  int serial_port_baudrate;
  sensor_msgs::LaserScan output;

  std::string mode;
  std::string locator;
  z_owned_session_t z_session;
  z_owned_subscriber_t z_scan_subscriber;


  nh_private.param<std::string>("topic_name", topic_name, "scan");
  nh_private.param<std::string>("mode", mode, "client");
  nh_private.param<std::string>("locator", locator, "");

  z_owned_config_t z_config = z_config_default();

  // Default config for the time being
  zc_config_insert_json(z_loan(z_config), Z_CONFIG_MODE_KEY, mode.c_str());
  if (!locator.empty()) {
        zc_config_insert_json(z_loan(z_config), Z_CONFIG_CONNECT_KEY, locator.c_str());
  }


  z_session = z_open(z_move(z_config));
  if (!z_check(z_session)) {
    ROS_ERROR("Unable to open session!\n");
      exit(EXIT_FAILURE);
  }

  // Start read and lease tasks for zenoh-pico
  // if (zp_start_read_task(z_session_loan(&z_session), NULL) < 0 || zp_start_lease_task(z_session_loan(&z_session), NULL) < 0) {
  //     ROS_ERROR("Unable to start read and lease tasks");
  //     exit(EXIT_FAILURE);
  // }


  // cmd_vel subscriber

  z_owned_closure_sample_t callback =  z_closure(z_scan_cb, NULL, (void*) &output );
  z_scan_subscriber =
      z_declare_subscriber(z_loan(z_session), z_keyexpr(topic_name.c_str()), z_move(callback), NULL);
  if (!z_check(z_scan_subscriber)) {
      ROS_ERROR("Unable to declare subscriber on %s.\n",topic_name.c_str());
      exit(-1);
  }
  ///



  ros::Publisher lidar_pub = nh.advertise<sensor_msgs::LaserScan>(topic_name, 10);  // create a ROS topic

  ros::Rate r(10); //10hz

  ROS_WARN("Publish topic message:ldlidar on %s.", topic_name.c_str());

  while (ros::ok()) {

    ToLaserscanMessagePublish(lidar_pub, output);

    r.sleep();
  }

  return 0;
}


