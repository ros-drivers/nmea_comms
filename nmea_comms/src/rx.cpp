
#include "rx.h"
#include "tx.h"
#include "checksum.h"

#include <poll.h>
#include <sstream>
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>

#include "ros/ros.h"
#include "nmea_msgs/Sentence.h"


static void _handle_sentence(ros::Publisher& publisher, ros::Time& stamp, char* sentence)
{
  char* sentence_body = strtok(sentence, "*");
  char* sentence_checksum = strtok(NULL, "*");
  if (sentence_checksum == NULL) {
    ROS_DEBUG("No checksum marker (*), discarding sentence.");
    return;   
  }
  if (strlen(sentence_checksum) != 2) {
    ROS_DEBUG("Checksum wrong length, discarding sentence.");
    return;
  }

  char computed_checksum[2];
  compute_checksum(sentence, computed_checksum);
  if (memcmp(computed_checksum, sentence_checksum, 2) != 0) {
    ROS_DEBUG("Bad checksum, discarding sentence.");
    return;
  }

  nmea_msgs::Sentence sentence_msg;
  boost::split(sentence_msg.fields, sentence_body, boost::is_any_of(","));

  sentence_msg.talker = sentence_msg.fields[0].substr(0, 2);
  sentence_msg.sentence = sentence_msg.fields[0].substr(2);
  sentence_msg.fields.erase(sentence_msg.fields.begin());

  sentence_msg.header.stamp = stamp;
  publisher.publish(sentence_msg);
}


static int threads_active = 1;
 
static void _thread_func(ros::NodeHandle& n, int fd)
{
  ros::Publisher pub = n.advertise<nmea_msgs::Sentence>("rx", 5);
  ros::Subscriber sub = n.subscribe<nmea_msgs::Sentence>("tx", 5, boost::bind(tx_msg_callback, _1, fd) ); 

  struct pollfd pollfds[] = { { fd, POLLIN, 0 } };
  char buffer[2048];
  char* buffer_write = buffer;
  char* buffer_end = &buffer[sizeof(buffer)];

  while(threads_active) {
    int retval = poll(pollfds, 1, 500);

    if (retval == 0) {
      // No event, just 1 sec timeout.
      continue;
    } else if (retval < 0) {
      ROS_FATAL("Error polling device. Terminating node.");
      ros::shutdown();
    } else if (pollfds[0].revents & (POLLHUP | POLLERR | POLLNVAL)) {
      ROS_INFO("Device error/hangup.");
      ROS_INFO("Shutting down publisher and subscriber.");
      pub.shutdown();
      sub.shutdown();
      ROS_INFO("Closing file descriptor.");
      close(fd);
      ROS_INFO("Exiting handler thread.");
      return;
    }

    // Read in contents of buffer and null-terminate it.
    ros::Time now = ros::Time::now();
    errno = 0;
    retval = read(fd, buffer_write, buffer_end - buffer_write - 1);
    if (retval > 0) {
      buffer_write += retval;
    } else {
      ROS_ERROR("Error reading from device. retval=%d, errno=%d, revents=%d", retval, errno, pollfds[0].revents);
      ROS_INFO("Shutting down publisher and subscriber.");
      pub.shutdown();
      sub.shutdown();
      ROS_INFO("Closing file descriptor.");
      close(fd);
      ROS_INFO("Exiting handler thread.");
      return;
    }
    ROS_DEBUG_STREAM("Buffer size after reading from fd: " << buffer_write - buffer);
    *buffer_write = '\0';

    char* buffer_read = buffer;
    while(1) {
      char* sentence = strchr(buffer_read, '$');
      if (sentence == NULL) break;
      char* sentence_end = strchr(sentence, '\r');
      if (sentence_end == NULL) break;
      *sentence_end = '\0';
      _handle_sentence(pub, now, sentence + 1);
      buffer_read = sentence_end + 1; 
    }

    int remainder = buffer_write - buffer_read;
    ROS_DEBUG_STREAM("Remainder in buffer is: " << remainder);
    memcpy(buffer, buffer_read, remainder);
    buffer_write = buffer + remainder;
  }
  close(fd);
}


static std::list<boost::thread*> rx_threads;

int rx_prune_threads()
{
  std::list<boost::thread*>::iterator thread_iter = rx_threads.begin();
  while (thread_iter != rx_threads.end()) {
    if ((**thread_iter).timed_join(boost::posix_time::milliseconds(10))) {
        delete *thread_iter;
        thread_iter = rx_threads.erase(thread_iter);
    } else {
        ++thread_iter;
    }
  } 
  return rx_threads.size();
}

void rx_stop_all()
{
  threads_active = 0;
  int thread_close_i = 0;
  std::list<boost::thread*>::iterator thread_iter = rx_threads.begin();
  while (thread_iter != rx_threads.end()) {
    if ((**thread_iter).timed_join(boost::posix_time::milliseconds(600))) {
      // Thread joined cleanly.
      thread_close_i++;
    } else {    
      ROS_WARN("Thread required interrupt() to exit.");
      (**thread_iter).interrupt();
    }
    delete *thread_iter;
    thread_iter = rx_threads.erase(thread_iter);
  } 
  ROS_INFO_STREAM("Closed " << thread_close_i << " thread(s) cleanly.");
}

void rx_thread_start(ros::NodeHandle& n, int fd)
{
  rx_threads.push_back(new boost::thread(_thread_func, boost::ref(n), fd));
}

