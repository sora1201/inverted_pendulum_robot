#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <sensor_msgs/Imu.h>
#include <jsk_rviz_plugins/OverlayText.h>
#include <jsk_rviz_plugins/Pictogram.h>
#include <std_msgs/Float32.h>
#include <string>

class PIDControl
{
private:
    ros::NodeHandle _nh;
    ros::NodeHandle _pnh;

    ros::Publisher _cmd_vel_pub;
    ros::Subscriber _imu_data_sub;
    ros::Subscriber _goal_sub;

    ros::Publisher _plotter2D_pitch_pub;
    ros::Publisher _plotter2D_init_vel_pub;
    ros::Publisher _piechart_vel_x_pub;
    ros::Publisher _text_pub;
    ros::Publisher _piechart_distance_pub;
    ros::Publisher _plotter2D_target_pitch_pub;

    sensor_msgs::Imu _imu_data;
    geometry_msgs::PoseStamped _goal;

    tf2_ros::Buffer _tfBuffer;
    tf2_ros::TransformListener _listener;

    double _Kp;
    double _Ki;
    double _Kd;
    double _init_vel;

    void getEuler(geometry_msgs::Quaternion quat, double yaw, double pitch, double roll);
    void getEuler(tf2::Quaternion quat, double yaw, double pitch, double roll);

    void infoJSKfloatData(ros::Publisher pub, double val);
    void infoJSKtextData(std::string str);
    void infoJSKtextData(std::string str, int32_t width, int32_t height, int32_t left, int32_t top);

    bool listenTF(geometry_msgs::TransformStamped *transformStamped);

    geometry_msgs::Pose getPose();

    double outputPID(double target, double now);

    double getTargetPitch(double velocity);
    
    geometry_msgs::Twist setTwist(geometry_msgs::Pose pose);

public:
    PIDControl();

    void drive();

    void IMUDataCallback(const sensor_msgs::Imu& msg);
    void GoalCallback(const geometry_msgs::PoseStamped& msg);
};

PIDControl::PIDControl() :
_nh(),
_pnh("~"),
_Kp(0),
_Ki(0),
_Kd(0),
_init_vel(0),
_listener(_tfBuffer)
{
    _pnh.getParam("Kp", _Kp);
    _pnh.getParam("Ki", _Ki);
    _pnh.getParam("Kd", _Kd);
    _pnh.getParam("init_vel", _init_vel);

    _cmd_vel_pub = _nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    _imu_data_sub = _nh.subscribe("imu/data", 10, &PIDControl::IMUDataCallback, this);
    _goal_sub = _nh.subscribe("move_base_simple/goal", 10, &PIDControl::GoalCallback, this);
    _plotter2D_pitch_pub = _nh.advertise<std_msgs::Float32>("pitch", 10);
    _plotter2D_init_vel_pub = _nh.advertise<std_msgs::Float32>("init_vel", 10);
    _piechart_vel_x_pub = _nh.advertise<std_msgs::Float32>("vel_x", 10);
    _text_pub = _nh.advertise<jsk_rviz_plugins::OverlayText>("overlay_text", 10);
    _piechart_distance_pub = _nh.advertise<std_msgs::Float32>("distance", 10);
    _plotter2D_target_pitch_pub = _nh.advertise<std_msgs::Float32>("target_pitch", 10);

    _imu_data.orientation.x = 0;
    _imu_data.orientation.y = 0;
    _imu_data.orientation.z = 0;
    _imu_data.orientation.w = 1.0;

    _goal.pose.position.x = 0;
    _goal.pose.position.y = 0;
    _goal.pose.position.z = 0;
}

void PIDControl::infoJSKfloatData(ros::Publisher pub, double val)
{
    std_msgs::Float32 jsk;
    jsk.data = val;
    pub.publish(jsk);
}

void PIDControl::infoJSKtextData(std::string str)
{
    jsk_rviz_plugins::OverlayText text;
    text.action = jsk_rviz_plugins::OverlayText::ADD;
    text.text = str;
    _text_pub.publish(text);
}

void PIDControl::infoJSKtextData(std::string str, int32_t width, int32_t height, int32_t left, int32_t top)
{
    jsk_rviz_plugins::OverlayText text;
    text.action = jsk_rviz_plugins::OverlayText::ADD;
    text.width = width;
    text.height = height;
    text.left = left;
    text.top = top;
    text.text = str;
    _text_pub.publish(text);
}

bool PIDControl::listenTF(geometry_msgs::TransformStamped *transformStamped)
{
    try {
        
        *transformStamped = _tfBuffer.lookupTransform("odom", "base_link", ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();

        return false;
    }

    return true;
}

geometry_msgs::Pose PIDControl::getPose()
{
    geometry_msgs::Pose pose;
    geometry_msgs::TransformStamped transformStamped;
    if (listenTF(&transformStamped) == true)
    {
        pose.position.x = transformStamped.transform.translation.x;
        pose.position.y = transformStamped.transform.translation.y;
        pose.position.z = transformStamped.transform.translation.z;
    }
    else
    {
        pose.position.x = 0;
        pose.position.y = 0;
        pose.position.z = 0;
    }

    pose.orientation = _imu_data.orientation;

    return pose;
}

double PIDControl::outputPID(double target, double now)
{
    static double e[2] = {0, 0};
    static ros::Time last_time = ros::Time::now();
    static double ei = 0;

    ros::Time current_time = ros::Time::now();

    e[1] = target - now;
    double p = _Kp * e[1];
    
    double dt = (current_time-last_time).toSec();
    double d = _Kd*(e[1]-e[0])/dt;

    ei += (e[1]+e[0])*dt/2; //台形積分
    double i = _Ki * ei;

    e[0] = e[1];
    last_time = current_time;

    return p+i+d;
}

double PIDControl::getTargetPitch(double velocity)
{
    double target_pitch = velocity;

    return target_pitch;
}

geometry_msgs::Twist PIDControl::setTwist(geometry_msgs::Pose pose)
{
    double x_diff = _goal.pose.position.x-pose.position.x;
    double y_diff = _goal.pose.position.y-pose.position.y;

    double roll, pitch, yaw;
    tf2::getEulerYPR(pose.orientation, yaw, pitch, roll);

    double a = atan2(y_diff, x_diff) - yaw;
    double L = sqrt(x_diff*x_diff+y_diff*y_diff);

    geometry_msgs::Twist cmd_vel;
    double velocity;
    if (L < 1.0)
    {
        velocity = 0.5*L/1.0;
    }
    else
    {
        velocity = 0.5;
    }

    cmd_vel.linear.x = -velocity - outputPID(0, getTargetPitch(velocity));
    cmd_vel.angular.z = -2*cmd_vel.linear.x*sin(a)/L;

    infoJSKfloatData(_plotter2D_pitch_pub, pitch);
    infoJSKfloatData(_plotter2D_target_pitch_pub, getTargetPitch(velocity));

    return cmd_vel;
}

void PIDControl::drive()
{
    geometry_msgs::Pose pose;
    pose = getPose();

    geometry_msgs::Twist cmd_vel;
    cmd_vel = setTwist(pose);

    // if (fabs(pitch) >= M_PI/360)
    // {
    //     _init_vel = 0;
    // }

    // if (fabs(pitch) >= M_PI/3) //制御不能(仮)
    // {
    //     cmd_vel.linear.x = 0;
    // }

    infoJSKfloatData(_plotter2D_init_vel_pub, _init_vel);
    infoJSKfloatData(_piechart_vel_x_pub, cmd_vel.linear.x);

    double distance = sqrt(pose.position.x*pose.position.x + pose.position.y*pose.position.y + pose.position.z*pose.position.z);
    infoJSKfloatData(_piechart_distance_pub, distance);

    std::string str = std::string("Kp:") + std::to_string(_Kp)
                    + std::string("\nKi:") + std::to_string(_Ki)
                    + std::string("\nKd:") + std::to_string(_Kd);
    infoJSKtextData(str);

    _cmd_vel_pub.publish(cmd_vel);
}

void PIDControl::IMUDataCallback(const sensor_msgs::Imu& msg)
{
    _imu_data = msg;
}

void PIDControl::GoalCallback(const geometry_msgs::PoseStamped& msg)
{
    _goal = msg;   
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pid_control");

    PIDControl pid;

    ros::Rate rate(50.0);
    while (ros::ok())
    {
        ros::spinOnce();

        pid.drive();

        rate.sleep();
    }

    return 0;
}