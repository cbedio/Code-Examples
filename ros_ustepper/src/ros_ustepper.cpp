#ifndef ROS_USTEPPER_H
#define ROS_USTEPPER_H

#include <string>
#include<stdio.h>
#include <ros/ros.h>
#include <angles/angles.h>

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
class RosUStepper
{
private:
    ros::NodeHandle n_;
protected:
    ros::NodeHandle node;

    std::string stepper_encoder_topic_name;
    std::string stepper_joint_topic_name;
	//std::string joint_states_topic_name;

	//sensor_msgs::JointState current_joint_msg;
public:

    ros::Publisher stepper_joint_pub;
    ros::Subscriber stepper_encoder_sub;
	//ros::Subscriber joint_states_sub;

    RosUStepper();
    ~RosUStepper();

    void stepper_encoder_callback(const std_msgs::Float64ConstPtr& encoder_msg);
	//void joint_states_callback(const sensor_msgs::JointStateConstPtr& joint_msg);



};

RosUStepper::RosUStepper(): n_("~"){
	
	/*joint_states_topic_name = getParam<std::string>(n_, "joint_states_topic_name", "/joint_states");
	joint_states_sub = node.subscribe(joint_states_topic_name, 1, &RosUStepper::joint_states_callback, this);*/

    stepper_encoder_topic_name = getParam<std::string>(n_, "stepper_encoder_topic_name", "/stepper_encoder");
    stepper_encoder_sub = node.subscribe(stepper_encoder_topic_name, 1, &RosUStepper::stepper_encoder_callback, this);

    stepper_joint_topic_name = getParam<std::string>(n_,"stepper_joint_topic_name", "/joint_states");
    stepper_joint_pub = node.advertise<sensor_msgs::JointState>(stepper_joint_topic_name,1);
}
RosUStepper::~RosUStepper(){}

/*void RosUStepper::joint_states_callback(const sensor_msgs::JointStateConstPtr& joint_msg){
	current_joint_msg.position = joint_msg->position;
	current_joint_msg.velocity = joint_msg->velocity;
	current_joint_msg.effort = joint_msg->effort;
	current_joint_msg.name = joint_msg->name;
}*/

void RosUStepper::stepper_encoder_callback(const std_msgs::Float64ConstPtr& encoder_msg){
	
	double angle_radians = -1 * angles::from_degrees(encoder_msg->data);    
	
	sensor_msgs::JointState joint_msg;
	/*joint_msg.position = current_joint_msg.position;
	joint_msg.velocity = current_joint_msg.velocity;
	joint_msg.effort = current_joint_msg.effort;
	joint_msg.name = current_joint_msg.name;

	std::vector<std::string>::iterator iterator = std::find(joint_msg.name.begin(), joint_msg.name.end(), "top_lidar_mount_link_joint");

	if (iterator != joint_msg.name.end()){
		int index = std::distance(joint_msg.name.begin(),iterator);
		joint_msg.position[index] = angle_radians;
	} else {*/
		joint_msg.position.push_back(angle_radians);
		joint_msg.name.push_back("top_lidar_mount_link_joint");
		joint_msg.header.stamp = ros::Time::now();
	//}
	

    

    stepper_joint_pub.publish(joint_msg);
}


#endif /* ROS_USTEPPER_H */

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "ros_ustepper");
    RosUStepper ros_ustepper;
    ros::spin();
}
