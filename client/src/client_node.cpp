#include "ros/ros.h"
#include "std_msgs/UInt16.h"
#include <iostream>

int main(int argc, char **argv) {
    ros::init(argc, argv, "talker");

    ros::NodeHandle n;

    ros::Publisher pub = n.advertise<std_msgs::UInt16>("/selector", 1000);

    while (ros::ok()) {
            std::cout << "Выберите режим работы:\n0. DUMMY\n1. VOYAGER\n2. WALLFOLLOWER\n";

            std_msgs::UInt16 msg;

            std::cin >> msg.data;

            while(std::cin.fail()) {
                std::cin.clear();
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
                std::cout << "Bad entry.  Enter a NUMBER: ";
                std::cin >> msg.data;
            }

            pub.publish(msg);
        }

    return 0;
}
