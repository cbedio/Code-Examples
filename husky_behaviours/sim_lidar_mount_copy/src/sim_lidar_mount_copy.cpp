#ifndef SIM_LIDAR_MOUNT_COPY_H
#define SIM_LIDAR_MOUNT_COPY_H

#include <string>
#include<stdio.h>
#include <ros/ros.h>

#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>

template<typename T>
T getParam(ros::NodeHandle& n, const std::string& name, const T& defaultValue)
{
    T v;
    if (n.getParam(name, v))
    {
        ROS_INFO_STREAM("Found parameter: " << name << ", value: " << v);
        return v;
    }
    else
    {
        ROS_WARN_STREAM("Cannot find value for parameter: " << name << ", assigning default: " << defaultValue);
    }
    return defaultValue;
}
class SimLidarMountCopy
{
private:
    ros::NodeHandle n_;
protected:
    ros::NodeHandle node;

    std::string command_topic_name;
    std::string stepper_joint_topic_name;
public:

    ros::Publisher command_pub;
    ros::Subscriber stepper_joint_sub;

    SimLidarMountCopy();
    ~SimLidarMountCopy();

    void stepper_joint_callback(const sensor_msgs::JointStateConstPtr& joint_state_msg);


};

SimLidarMountCopy::SimLidarMountCopy(): n_("~"){
    stepper_joint_topic_name = getParam<std::string>(n_,"stepper_joint_topic_name", "/joint_states");
    stepper_joint_sub = node.subscribe(stepper_joint_topic_name, 1, &SimLidarMountCopy::stepper_joint_callback, this);

    command_topic_name = getParam<std::string>(n_,"command_topic_name", "/husky_joint_position_controller/command");
    command_pub = node.advertise<std_msgs::Float64>(command_topic_name,1);
}
SimLidarMountCopy::~SimLidarMountCopy(){}

void SimLidarMountCopy::stepper_joint_callback(const sensor_msgs::JointStateConstPtr& joint_state_msg){
    std_msgs::Float64 command_msg;
    command_msg.data = joint_state_msg->position[0];
    command_pub.publish(command_msg);
}


#endif /* SIM_LIDAR_MOUNT_COPY_H */

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "sim_lidar_mount_copy");
    SimLidarMountCopy sim_lidar_mount_copy;
    ros::spin();
}