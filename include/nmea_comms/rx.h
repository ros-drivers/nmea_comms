
#include <string>
#include <stdint.h>

namespace ros {
  class NodeHandle;
};

void rx_thread_start(ros::NodeHandle& n, int fd, std::string frame_id, uint32_t byte_time_ns=0);
void rx_stop_all();
int rx_prune_threads();
