#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <vector>

class MPCControl
{
private:
    ros::NodeHandle _nh;
    ros::NodeHandle _pnh;

public:
    MPCControl();
};

MPCControl::MPCControl() :
_nh(),
_pnh("~")
{}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mpc_control");

    MPCControl mpc;

    ros::Rate rate(50.0);
    while (ros::ok())
    {
        ros::spinOnce();



        rate.sleep();
    }

    return 0;
}