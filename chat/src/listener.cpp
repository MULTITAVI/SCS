#include "ros/ros.h"
#include "std_msgs/String.h"


template<typename type>
class Publisher {
public:
  Publisher(ros::NodeHandle node, const std::string& topic_name, uint32_t size) {
    pub_ = node.advertise<type>(topic_name, size);
  }
  void publish(const std_msgs::String& msg) {
    pub_.publish(msg);
  }
private:
  ros::Publisher pub_;
};

class Subscriber {
public:
  Subscriber(ros::NodeHandle node, const std::string& topic_name, uint32_t size, void (*func)(const std_msgs::String::ConstPtr&)) {
    sub_ = node.subscribe(topic_name, size, func);
  }
private:
  ros::Subscriber sub_;
};

void chatterCallback(const std_msgs::String::ConstPtr& msg) {
  ROS_ERROR("I heard: [%s]", msg->data.c_str());
}

// /**
//  * This tutorial demonstrates simple receipt of messages over the ROS system.
//  */


int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "listener");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle node;

  ros::Rate loop_rate(1);

  Subscriber sub = Subscriber(node, "chatter", 1000, chatterCallback);
  Publisher<std_msgs::String> pub = Publisher<std_msgs::String>(node, "reply", 1000);

  while (ros::ok()) {
    std_msgs::String msg;
    
    msg.data = "Sending reply";

    pub.publish(msg);

    ROS_INFO("Sending reply");

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}


// #include "ros/ros.h"
// #include "std_msgs/String.h"

// /**
//  * This tutorial demonstrates simple receipt of messages over the ROS system.
//  */
// void chatterCallback(const std_msgs::String::ConstPtr& msg)
// {
//   ros::NodeHandle node = ros::NodeHandle();

//   ROS_ERROR("I heard: [%s]", msg->data.c_str());

//   ros::Publisher reply_pub = node.advertise<std_msgs::String>("reply", 1000);
//   std_msgs::String new_msg;

//   new_msg.data = "Sending reply";
//   reply_pub.publish(new_msg);
// }

// int main(int argc, char **argv)
// {
//   /**
//    * The ros::init() function needs to see argc and argv so that it can perform
//    * any ROS arguments and name remapping that were provided at the command line.
//    * For programmatic remappings you can use a different version of init() which takes
//    * remappings directly, but for most command-line programs, passing argc and argv is
//    * the easiest way to do it.  The third argument to init() is the name of the node.
//    *
//    * You must call one of the versions of ros::init() before using any other
//    * part of the ROS system.
//    */
//   ros::init(argc, argv, "listener");

//     /**
//    * NodeHandle is the main access point to communications with the ROS system.
//    * The first NodeHandle constructed will fully initialize this node, and the last
//    * NodeHandle destructed will close down the node.
//    */
//   ros::NodeHandle n;

//   /**
//    * The subscribe() call is how you tell ROS that you want to receive messages
//    * on a given topic.  This invokes a call to the ROS
//    * master node, which keeps a registry of who is publishing and who
//    * is subscribing.  Messages are passed to a callback function, here
//    * called chatterCallback.  subscribe() returns a Subscriber object that you
//    * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
//    * object go out of scope, this callback will automatically be unsubscribed from
//    * this topic.
//    *
//    * The second parameter to the subscribe() function is the size of the message
//    * queue.  If messages are arriving faster than they are being processed, this
//    * is the number of messages that will be buffered up before beginning to throw
//    * away the oldest ones.
//    */
//   ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

//   /**
//    * ros::spin() will enter a loop, pumping callbacks.  With this version, all
//    * callbacks will be called from within this thread (the main one).  ros::spin()
//    * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
//    */
//   ros::spin();

//   return 0;
// }