#include "ros/ros.h"
#include "std_msgs/Float64.h"

class SinusoidalLidarMotionGazebo {
private:
    // Declare and initialize ros node handler
    ros::NodeHandle nh_;
    
    // Declare publishers
    ros::Publisher mount_position_pub_;
    
    double angle_max_;
    double angle_min_;
    double angle_current_;
    double angle_range_;

    int steps_;

    /*
        Rotation direction
        anticlockwise dir = 1
        clockwise dir = 0
    */
    bool dir_;
    
public:
    SinusoidalLidarMotionGazebo();
    void changePosition(void);
};
SinusoidalLidarMotionGazebo::SinusoidalLidarMotionGazebo(){

    // Initialize rate for a constant publishing frequency
    
    
    // get initial private member values from the parameter server
    nh_.getParam("/sinusoidal_lidar_motion_gazebo/angle_max", angle_max_);
    nh_.getParam("/sinusoidal_lidar_motion_gazebo/angle_min", angle_min_);
    nh_.getParam("/sinusoidal_lidar_motion_gazebo/steps", steps_);

    // Set Initial motor direction
    dir_ = 0;

    // Set initial angle direction
    // initial angle is set to the min angle
    angle_current_ = angle_min_;

    angle_range_ = angle_max_ - angle_min_;

    // Initialize publishers
    mount_position_pub_ = nh_.advertise<std_msgs::Float64>("/husky_joint_position_controller/command",100);
}
void SinusoidalLidarMotionGazebo::changePosition(void){
    ros::Rate r_(steps_);
    std_msgs::Float64 float_msgs;
    float_msgs.data = angle_current_;
    mount_position_pub_.publish(float_msgs);
    if ((angle_current_ <= angle_min_) || (angle_current_ >= angle_max_)) {
        dir_ = (dir_)?(0):(1);
    }
    
    double angle_change = angle_range_ / steps_;
    
    if (dir_) {
       angle_current_ += angle_change;
    } else {
        angle_current_ -= angle_change;
    }

    r_.sleep();
    
    
}

int main(int argc, char **argv) {

    // Initialize the ros node
    ros::init(argc, argv, "sinusoidal_lidar_motion_gazebo");
    
    SinusoidalLidarMotionGazebo slmg;

    while(ros::ok()) {
        slmg.changePosition();
        ros::spinOnce();
    }

    return 0;
}
