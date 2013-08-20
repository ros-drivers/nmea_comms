
namespace ros {
  class NodeHandle;
};

void rx_thread_start(ros::NodeHandle& n, int fd);
void rx_stop_all();
int rx_prune_threads();
