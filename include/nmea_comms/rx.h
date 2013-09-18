
#include <string>

namespace ros {
  class NodeHandle;
};

void rx_thread_start(ros::NodeHandle& n, int fd, std::string frame_id);
void rx_stop_all();
int rx_prune_threads();
