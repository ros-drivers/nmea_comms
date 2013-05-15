
#include "rx.h"
#include "checksum.h"

#include <stdio.h>
#include <poll.h>
#include <boost/algorithm/string.hpp>

#include "ros/ros.h"
#include "nmea_msgs/Sentence.h"

#include "checksum.h"


void _msg_callback(const nmea_msgs::SentenceConstPtr sentence_msg_ptr, int fd)
{
  static int consecutive_errors = 0;
  std::stringstream sentence_body_ss;
  sentence_body_ss << sentence_msg_ptr->talker << sentence_msg_ptr->sentence << "," <<
      boost::join(sentence_msg_ptr->fields, ",");
  std::string sentence_body = sentence_body_ss.str();

  char checksum[2];
  compute_checksum(sentence_body.c_str(), checksum); 

  char buffer[256];
  int buffer_length = snprintf(buffer, 256, "$%s*%s\r\n", sentence_body.c_str(), checksum);

  // No guarantee that write() will write everything, so we use poll() to block
  // on the availability of the fd for writing until the whole message has been
  // written out.
  char* buffer_write = buffer;
  struct pollfd pollfds[] = { { fd, POLLOUT, 0 } };
  while(ros::ok()) {
    int retval = poll(pollfds, 1, 1000);

    if (pollfds[0].revents & (POLLHUP | POLLERR)) {
      ROS_FATAL("Killing node due to device hangup.");
      ros::shutdown();
    }

    retval = write(fd, buffer_write, buffer_length - (buffer_write - buffer));
    if (retval > 0) {
      buffer_write += retval;
    } else {
      ROS_WARN("Device write error; abandoning message (%s%s).", 
               sentence_msg_ptr->talker.c_str(), sentence_msg_ptr->sentence.c_str());
      if (++consecutive_errors >= 10) {
        ROS_FATAL("Killing node due to %d consecutive write errors.", consecutive_errors);
        ros::shutdown();
      }
      break;
    }
    if (buffer_write - buffer >= buffer_length) {
      consecutive_errors = 0;
      break;
    }
  }
}


static ros::Subscriber subscriber;

void tx_start(ros::NodeHandle& n, int fd)
{
  subscriber = n.subscribe<nmea_msgs::Sentence>("tx", 5, boost::bind(_msg_callback, _1, fd) ); 
}

void tx_stop()
{
  subscriber.shutdown();
}
