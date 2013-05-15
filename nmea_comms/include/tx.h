
//#include "ros/ros.h"
#include "nmea_msgs/Sentence.h"

//namespace ros {
//  class NodeHandle;
//};
//namespace nmea_msgs {
//  class SentenceConstPtr;
//}

/*void tx_start(ros::NodeHandle& n, int fd);
void tx_stop();*/

void tx_msg_callback(const nmea_msgs::SentenceConstPtr sentence_msg_ptr, int fd);
