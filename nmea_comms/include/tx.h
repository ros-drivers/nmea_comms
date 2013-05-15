
namespace ros {
  class NodeHandle;
};

void tx_start(ros::NodeHandle& n, int fd);
void tx_stop();
