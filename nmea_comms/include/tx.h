
#include "ros/ros.h"

namespace nmea_msgs {
ROS_DECLARE_MESSAGE(Sentence);
}

void tx_msg_callback(const nmea_msgs::SentenceConstPtr sentence_msg_ptr, int fd);
