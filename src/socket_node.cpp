#include "ros/ros.h"
#include "nmea_msgs/Sentence.h"

#include "nmea_comms/rx.h"
#include "nmea_comms/tx.h"

#include <poll.h>
#include <sys/socket.h>
#include <netinet/in.h>


void msg_callback(const nmea_msgs::SentenceConstPtr sentence_msg_ptr) {
  // noop
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "nmea_socket_node");
  ros::NodeHandle n_local("~");
  ros::NodeHandle n;

  int listener_fd = socket(AF_INET, SOCK_STREAM, 0);
  if (listener_fd < 0) 
  {
    ROS_FATAL("ERROR opening socket");
    ros::shutdown();
  }

  int port;
  n_local.param("port", port, 29500);

  std::string frame_id;
  n_local.param<std::string>("frame_id", frame_id, "navsat");
 
  /* Initialize socket structure */
  struct sockaddr_in serv_addr, cli_addr;
  bzero((char *) &serv_addr, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_addr.s_addr = INADDR_ANY;
  serv_addr.sin_port = htons(port);
 
  /* Now bind the host address using bind() call.*/
  int previous_success = 1; 
  while(1) {
    if (bind(listener_fd, (struct sockaddr *) &serv_addr,
                        sizeof(serv_addr)) < 0) {
      if (previous_success) {
        ROS_ERROR("Unable to bind socket. Is port %d in use? Retrying every 1s.", port);
        previous_success = 0;
      }
      ros::Duration(1.0).sleep();
    } else {
      break;
    }
  }

  // Start ROS spinning in the background.
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Dummy publisher and subscriber.
  // These are here because rospy has some bugs related to setting up
  // and tearing down the same connection repeatly. So having these topics
  // persistently available avoids issues with external Python-based nodes.
  // See: https://github.com/ros/ros_comm/issues/129
  ros::Publisher dummy_pub = n.advertise<nmea_msgs::Sentence>("rx", 5);
  ros::Subscriber dummy_sub = n.subscribe<nmea_msgs::Sentence>("tx", 5, msg_callback ); 

  // Now start listening for the clients, here 
  // process will go in sleep mode and will wait 
  // for the incoming connection
  listen(listener_fd, 5);

  unsigned int clilen = sizeof(cli_addr);

  ROS_INFO("Now listening for connections on port %d.", port);
  struct pollfd pollfds[] = { { listener_fd, POLLIN, 0 } };
  while (ros::ok()) 
  {
    int retval = poll(pollfds, 1, 500);
    if (retval > 0) {
      int new_client_fd = accept(listener_fd, (struct sockaddr *) &cli_addr, &clilen);
      if (new_client_fd < 0) {
        // Error of some kind? 
      } else {
        rx_thread_start(n, new_client_fd, frame_id); 
      }
    } else {
      // just in case
      ros::Duration(0.2).sleep();
    }
  }

  close(listener_fd); 
  return 0;
}
