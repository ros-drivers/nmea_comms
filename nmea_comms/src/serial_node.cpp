#include "ros/ros.h"

#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <termios.h> /* POSIX terminal control definitions */

#include "rx.h"
#include "tx.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "nmea_serial_node");
  ros::NodeHandle n_local("~");
  ros::NodeHandle n;

  std::string port;
  int32_t baud;
  n_local.param<std::string>("port", port, "/dev/ttyUSB0");
  n_local.param("baud", baud, 115200);
 
  int serial_fd = -1;
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
    rx_start(n, serial_fd);
    tx_start(n, serial_fd);

    ros::spin();

    rx_stop(); 
    tx_stop(); 
  }
  
  close(serial_fd); 
  return 0;
}
