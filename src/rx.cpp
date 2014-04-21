
#include "nmea_comms/rx.h"
#include "nmea_comms/tx.h"
#include "nmea_comms/checksum.h"

#include <poll.h>
#include <sstream>
#include <sys/ioctl.h>
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>

#include "ros/ros.h"
#include "nmea_msgs/Sentence.h"

#define RX_INITIAL_LENGTH 40
#define RX_SUCCESSIVE_LENGTH 8

static void _handle_sentence(ros::Publisher& publisher, ros::Time& stamp, const char* sentence, const char* frame_id)
{
  ROS_DEBUG("Sentence RX: %s", sentence);
 
  static nmea_msgs::Sentence sentence_msg;
  sentence_msg.sentence = sentence;
  sentence_msg.header.stamp = stamp;
  sentence_msg.header.frame_id = frame_id;
  publisher.publish(sentence_msg);
}


static int threads_active = 1;
 
static void _thread_func(ros::NodeHandle& n, int fd, std::string frame_id, uint32_t byte_time_ns=0)
{
  ROS_DEBUG("New connection handler thread beginning.");

  ros::Publisher pub = n.advertise<nmea_msgs::Sentence>("nmea_sentence", 5);
  ros::Subscriber sub = n.subscribe<nmea_msgs::Sentence>("nmea_sentence_out", 5, 
                            boost::bind(tx_msg_callback, _1, fd) ); 

  struct pollfd pollfds[] = { { fd, POLLIN, 0 } };
  char buffer[2048];
  char* buffer_write = buffer;
  char* buffer_end = &buffer[sizeof(buffer)];

  while(threads_active) {
    errno = 0;
    int retval = poll(pollfds, 1, 500);
    ROS_DEBUG("Poll retval=%d, errno=%d, revents=%d", retval, errno, pollfds[0].revents);

    if (retval == 0) {
      // No event, just 1 sec timeout.
      continue;
    } else if (retval < 0) {
      ROS_FATAL("Error polling device. Terminating node.");
      ros::shutdown();
    } else if (pollfds[0].revents & (POLLHUP | POLLERR | POLLNVAL)) {
      ROS_INFO("Device error/hangup.");
      ROS_DEBUG("Shutting down publisher and subscriber.");
      pub.shutdown();
      sub.shutdown();
      ROS_DEBUG("Closing file descriptor.");
      close(fd);
      ROS_DEBUG("Exiting handler thread.");
      return;
    }
    
    // We can save some CPU by sleeping if the number waiting bytes is really small
    if (byte_time_ns > 0) {
      int waiting_bytes;
      errno = ioctl(fd, FIONREAD, &waiting_bytes);
      if (errno == 0) {
        int wait_for = 0;
        int buffer_plus_waiting = (buffer_write - buffer) + waiting_bytes;
        if (buffer_plus_waiting < RX_INITIAL_LENGTH) {
          wait_for = RX_INITIAL_LENGTH - buffer_plus_waiting;
        } else if (waiting_bytes < RX_SUCCESSIVE_LENGTH) {
          wait_for = RX_SUCCESSIVE_LENGTH - waiting_bytes;
        }
        if (wait_for > 0) {
          struct timespec req = { 0, wait_for * byte_time_ns }, rem;
          ROS_DEBUG_STREAM("Sleeping for " << wait_for << " bytes (" << byte_time_ns << " ns)");
          nanosleep(&req, &rem);
        }
      }
    }

    // Read in contents of buffer and null-terminate it.
    ros::Time now = ros::Time::now();
    errno = 0;
    retval = read(fd, buffer_write, buffer_end - buffer_write - 1);
    ROS_DEBUG("Read retval=%d, errno=%d", retval, errno);
    ROS_DEBUG_COND(retval < 0, "Read error: %s", strerror(errno));
    if (retval > 0) {
      if (strnlen(buffer_write, retval) != retval) {
        ROS_WARN("Null byte received from serial port, flushing buffer.");
        buffer_write = buffer;
        continue;
      }
      buffer_write += retval;
    } else if (retval == 0) {
      ROS_INFO("Device stream ended.");
      ROS_DEBUG("Shutting down publisher and subscriber.");
      pub.shutdown();
      sub.shutdown();
      ROS_DEBUG("Closing file descriptor.");
      close(fd);
      ROS_DEBUG("Exiting handler thread.");
      return; 
    } else {
      // retval < 0, indicating an error of some kind.
      if (errno == EAGAIN) {
        // Can't read from the device, try again.
        continue;
      } else {
        ROS_FATAL("Error reading from device. retval=%d, errno=%d, revents=%d", retval, errno, pollfds[0].revents);
        ros::shutdown();
      }
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
      _handle_sentence(pub, now, sentence, frame_id.c_str());
      buffer_read = sentence_end + 1; 
    }

    int remainder = buffer_write - buffer_read;
    if (remainder > 2000) {
      ROS_WARN("Buffer size >2000 bytes, resetting buffer.");
      remainder = 0;
    }
    ROS_DEBUG_STREAM("Remainder in buffer is: " << remainder);
    memmove(buffer, buffer_read, remainder);
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

void rx_thread_start(ros::NodeHandle& n, int fd, std::string frame_id, uint32_t byte_time_ns)
{
  rx_threads.push_back(new boost::thread(_thread_func, boost::ref(n), fd, frame_id, byte_time_ns));
}

