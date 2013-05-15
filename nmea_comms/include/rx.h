
namespace ros {
  class NodeHandle;
};

void rx_start(ros::NodeHandle& n, int fd);
void rx_stop();
