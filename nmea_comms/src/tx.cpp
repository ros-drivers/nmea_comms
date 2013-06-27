
#include "rx.h"
#include "checksum.h"

#include <stdio.h>
#include <poll.h>
#include <boost/algorithm/string.hpp>

#include "ros/ros.h"
#include "nmea_msgs/Sentence.h"

#include "checksum.h"


void tx_msg_callback(const nmea_msgs::SentenceConstPtr sentence_msg_ptr, int fd)
{
  static int consecutive_errors = 0;

  char buffer[256];
  int buffer_length = snprintf(buffer, 256, "%s\r\n", sentence_msg_ptr->sentence.c_str());

  // No guarantee that write() will write everything, so we use poll() to block
  // on the availability of the fd for writing until the whole message has been
  // written out.
  const char* buffer_write = buffer;
  struct pollfd pollfds[] = { { fd, POLLOUT, 0 } };
  while(ros::ok()) {
    int retval = poll(pollfds, 1, 1000);

    if (pollfds[0].revents & POLLHUP) {
      ROS_FATAL("Killing node due to device hangup.");
      ros::shutdown();
    }

    if (pollfds[0].revents & POLLERR) {
      ROS_FATAL("Killing node due to device error.");
      ros::shutdown();
    }

    retval = write(fd, buffer_write, buffer_length - (buffer_write - buffer));
    if (retval > 0) {
      buffer_write += retval;
    } else {
      ROS_WARN("Device write error; abandoning message (%s).", 
               sentence_msg_ptr->sentence.c_str());
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
