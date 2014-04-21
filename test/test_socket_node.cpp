
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <gtest/gtest.h>

#include <ros/ros.h>
#include "nmea_msgs/Sentence.h"


int g_argc;
char** g_argv;

class NMEASocketTest : public testing::Test
{
  public:
    ros::NodeHandle* n;
    ros::Publisher tx;
    ros::Subscriber rx;
    const char *mock_sentence;
    boost::shared_ptr<nmea_msgs::Sentence const> last_received;
    int total_received;

    NMEASocketTest() : mock_sentence("$JUST,a,test*00"), total_received(0)
    {
      ros::init(g_argc, g_argv, "test_socket_node");
      n = new ros::NodeHandle();
      tx = n->advertise<nmea_msgs::Sentence>("tx", 5);
      rx = n->subscribe("rx", 1, &NMEASocketTest::_rx_callback, this);
    }

    void connect(int* sockfd)
    {
      *sockfd = socket(AF_INET, SOCK_STREAM, 0);

      struct sockaddr_in serv_addr; 
      memset(&serv_addr, '0', sizeof(serv_addr)); 
      serv_addr.sin_addr.s_addr = 0x0100007F;  // 127.0.0.1
      serv_addr.sin_family = AF_INET;
      serv_addr.sin_port = htons(29500);

      ASSERT_EQ(0, ::connect(*sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)));
    }

    void disconnect(int* sockfd)
    {
      close(*sockfd); 
    }

    ~NMEASocketTest()
    {
      delete n;
    }

    void _rx_callback(const nmea_msgs::SentenceConstPtr& msg) {
      ROS_INFO_STREAM("Received: [" << msg->sentence << "]");
      total_received++;
      last_received = msg;
    }
};


TEST_F(NMEASocketTest, basic_rx_test)
{
  int dev;
  connect(&dev);

  // Time for publisher and subscriber to link up.
  ros::Duration(1.0).sleep();

  char buffer[40];
  int len = sprintf(buffer, "%s%s", mock_sentence, "\r\n");
  EXPECT_EQ(len, write(dev, buffer, len));

  // Time for message to arrive.
  ros::Duration(0.1).sleep();
  ros::spinOnce();

  ASSERT_TRUE(NULL != last_received);
  ASSERT_EQ(1, total_received);
  ASSERT_STREQ(mock_sentence, last_received->sentence.c_str());
  disconnect(&dev);
}


TEST_F(NMEASocketTest, multi_rx_test)
{
  const int num_devs = 5;
  const int msgs_per_dev = 10;
  int devs[num_devs], i;

  for (i = 0; i < num_devs; i++) {
    connect(&devs[i]);
  }

  // Time for publishers and subscribers to link up.
  ros::Duration(2.0).sleep();

  for (int m = 0; m < msgs_per_dev; m++) {
    for (i = 0; i < num_devs; i++) {
      char buffer[40];
      int len = sprintf(buffer, "$MSG,blah,%d,%d*00%s", i, m, "\r\n");
      ROS_INFO("Sending msg #%d on dev #%d: [%s]", m, i, buffer);
      EXPECT_EQ(len, write(devs[i], buffer, len));

      // TODO: Understand better why this sleep and spin are necessary here; seems like they shouldn't be.
      ros::Duration(0.01).sleep();
      ros::spinOnce();
    }
  }

  // Time for final messages to arrive.
  ros::Duration(0.1).sleep();
  ros::spinOnce();

  EXPECT_EQ(num_devs * msgs_per_dev, total_received);

  for (i = 0; i < num_devs; i++) {
    disconnect(&(devs[i]));
  }
}


TEST_F(NMEASocketTest, basic_tx_test)
{
  int dev;
  connect(&dev);

  // Time for publisher and subscriber to link up.
  ros::Duration(1.0).sleep();

  nmea_msgs::Sentence msg;
  msg.sentence = mock_sentence;
  tx.publish(msg);

  // If this fails, GoogleTest will kill it after 60s.
  char buffer[40];
  ASSERT_EQ(strlen(mock_sentence), read(dev, buffer, strlen(mock_sentence)));

  ASSERT_STREQ(mock_sentence, buffer); 

  disconnect(&dev);
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  g_argc = argc;
  g_argv = argv;
  return RUN_ALL_TESTS();
}
