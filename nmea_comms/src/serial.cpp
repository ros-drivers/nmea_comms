#include "ros/ros.h"
#include "nmea_msgs/Sentence.h"

#include <poll.h>
#include <sstream>
#include <boost/thread.hpp>

#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <termios.h> /* POSIX terminal control definitions */


void compute_checksum(char* sentence_body, char checksum_out[2])
{
  char checksum = 0;
  while(*sentence_body) {
    checksum ^= *sentence_body;
    sentence_body++;
  }
  sprintf(checksum_out, "%2X", checksum);
}


void tx_callback(const nmea_msgs::Sentence& sentence)
{

}


void handle_sentence(ros::Publisher& publisher, ros::Time& stamp, char* sentence)
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
  std::string first_token = strtok(sentence_body, ",");
  sentence_msg.talker = first_token.substr(0, 2);
  sentence_msg.sentence = first_token.substr(2);

  char* token;
  while((token = strtok(NULL, ","))) {
    sentence_msg.fields.push_back(token);
  }

  sentence_msg.header.stamp = stamp;
  publisher.publish(sentence_msg);
}


void rx_thread_func(ros::NodeHandle& n, int fd)
{
  ros::Publisher pub = n.advertise<nmea_msgs::Sentence>("rx", 5);
  struct pollfd pollfds[] = { { fd, POLLIN, 0 } };
  char buffer[2048];
  char* buffer_write = buffer;
  char* buffer_end = &buffer[sizeof(buffer)];

  while(ros::ok()) {
    poll(pollfds, 1, 1000);

    // Read in contents of buffer and null-terminate it.
    ros::Time now = ros::Time::now();
    int buffer_in_count = read(fd, buffer_write, buffer_end - buffer_write - 1);
    if (buffer_in_count > 0) {
      buffer_write += buffer_in_count;
    } else {
      continue;
    }
    ROS_DEBUG("Buffer size after reading from fd: %d ", buffer_write - buffer);
    *buffer_write = '\0';

    char* buffer_read = buffer;
    while(1) {
      char* sentence = strchr(buffer_read, '$');
      if (sentence == NULL) break;
      char* sentence_end = strchr(sentence, '\r');
      if (sentence_end == NULL) break;
      *sentence_end = '\0';
      handle_sentence(pub, now, sentence + 1);
      buffer_read = sentence_end + 1; 
    }

    int remainder = buffer_write - buffer_read;
    ROS_DEBUG("Remainder in buffer is %d", remainder);
    memcpy(buffer, buffer_read, remainder);
    buffer_write = buffer + remainder;
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "nmea_serial_node");
  ros::NodeHandle n_local("~");
  ros::NodeHandle n;

  std::string port;
  int32_t baud;
  n_local.param<std::string>("port", port, "/dev/ttyUSB0");
  n_local.param("baud", baud, 115200);
 
  int serial_fd;
  while(ros::ok()) {
    static int first_connection = 1;
    serial_fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);
    
    if (serial_fd == -1) {
      ROS_WARN_STREAM("Could not open " << port << "."); 
      goto retry_connection; 
    } 

    if (!isatty(serial_fd)) {
      ROS_WARN_STREAM("File " << port << " is not a tty."); 
      goto close_serial; 
    }

    struct termios options;
    tcgetattr(serial_fd, &options);
    options.c_cflag = 0;
    options.c_cflag |= CS8; 

    options.c_cflag |= (CLOCAL | CREAD);
    cfsetispeed(&options, B19200);
    cfsetospeed(&options, B19200);
    options.c_iflag = 0;
    options.c_oflag = 0;
    options.c_lflag = 0;
    options.c_cc[VMIN]  = 0;
    options.c_cc[VTIME] = 1;
    tcsetattr(serial_fd, TCSAFLUSH, &options);

    // successful connection setup.
    break;

    close_serial:
      close(serial_fd); 
    retry_connection:
      if (first_connection) {
        ROS_INFO("Retrying every 1 second."); 
        first_connection = 0;
      }
      ros::Duration(1.0).sleep(); 
  }

  if (ros::ok()) {  
    boost::thread rx_thread(rx_thread_func, boost::ref(n), serial_fd);
    ros::Subscriber sub = n.subscribe("tx", 5, tx_callback);
    ros::spin();
 
    rx_thread.interrupt();
  }
  
  close(serial_fd); 
  return 0;
}
