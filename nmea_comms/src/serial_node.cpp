#include "ros/ros.h"

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#include "rx.h"


void manage_connection(const ros::TimerEvent& event, ros::NodeHandle& n, std::string port, int32_t baud)
{
  if (rx_prune_threads() > 0) {
    // Serial thread already active. Nothing to do here.
    return;
  }

  // Only output error messages when things were previously peachy.
  static int previous_success = 1;

  while(ros::ok()) {
    int serial_fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);
    
    if (serial_fd == -1) {
      if (previous_success) { 
        ROS_ERROR_STREAM("Could not open " << port << "."); 
      }
      goto retry_connection; 
    } 

    if (!isatty(serial_fd)) {
      if (previous_success) { 
        ROS_ERROR_STREAM("File " << port << " is not a tty."); 
      }
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

    // Successful connection setup; kick off servicing thread.
    ROS_INFO_STREAM("Successfully connected on " << port << "."); 
    previous_success = 1;
    rx_thread_start(n, serial_fd); 
    break;

  close_serial:
    close(serial_fd); 

  retry_connection:
    if (previous_success) { 
      ROS_ERROR("Retrying connection every 1 second."); 
      previous_success = 0;
    }

    ros::Duration(1.0).sleep(); 
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

  ros::Timer timer = n.createTimer(ros::Duration(1.0),
      boost::bind(manage_connection, _1, n, port, baud)); 
  ros::spin();

  rx_stop_all();

  return 0;
}
