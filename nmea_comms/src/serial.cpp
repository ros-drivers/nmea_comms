#include "ros/ros.h"
#include "nmea_msgs/Sentence.h"

#include <poll.h>
#include <sstream>


void tx_callback(const nmea_msgs::Sentence& sentence)
{
  
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "nmea_serial_node");
  ros::NodeHandle n("~");

  ros::Publisher pub = n.advertise<nmea_msgs::Sentence>("rx", 5);
  ros::Subscriber sub = n.subscribe("tx", 5, tx_callback);
  
  std::string port;
  int32_t baud;
  n.param<std::string>("port", port, "/dev/ttyUSB0");
  n.param("baud", baud, 115200);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    nmea_msgs::Sentence msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.sentence = ss.str();
    ROS_INFO("%s", msg.sentence.c_str());
    pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}
