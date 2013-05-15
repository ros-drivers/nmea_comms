#include "ros/ros.h"

#include "rx.h"
#include "tx.h"

#include <sys/socket.h>
#include <netinet/in.h>



int main(int argc, char **argv)
{
  ros::init(argc, argv, "nmea_socket_node");
  ros::NodeHandle n_local("~");
  ros::NodeHandle n;

  int socket_fd = socket(AF_INET, SOCK_STREAM, 0);
  if (socket_fd < 0) 
  {
    ROS_FATAL("ERROR opening socket");
    ros::shutdown();
  }

  int port;
  n_local.param("port", port, 29500);
 
  /* Initialize socket structure */
  struct sockaddr_in serv_addr, cli_addr;
  bzero((char *) &serv_addr, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_addr.s_addr = INADDR_ANY;
  serv_addr.sin_port = htons(port);
 
  /* Now bind the host address using bind() call.*/
  if (bind(socket_fd, (struct sockaddr *) &serv_addr,
                      sizeof(serv_addr)) < 0) {
    ROS_FATAL("ERROR binding socket. Is port %d in use?", port);
    ros::shutdown();
  }

  /* Now start listening for the clients, here 
   * process will go in sleep mode and will wait 
   * for the incoming connection
   */
  listen(socket_fd,5);
  int new_client_fd = -1;
  unsigned int clilen = sizeof(cli_addr);
  while (ros::ok()) 
  {
    new_client_fd = accept(socket_fd, (struct sockaddr *) &cli_addr, &clilen);
    if (new_client_fd < 0) {
       
    }

    //rx_start(n, new_client_fd);
    tx_start(n, new_client_fd);
    ros::spin();
  }
  //rx_stop(); 
  tx_stop(); 
  
  close(socket_fd); 
  return 0;
}
