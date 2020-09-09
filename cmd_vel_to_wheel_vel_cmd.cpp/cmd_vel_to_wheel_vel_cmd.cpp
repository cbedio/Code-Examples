#ifndef CMD_VEL_TO_WHEEL_VEL_CMD_H
#define CMD_VEL_TO_WHEEL_VEL_CMD_H

#include <string>
#include <stdio.h>
#include <ros/ros.h>

#include <nifti_robot_driver_msgs/TracksStamped.h>
#include <geometry_msgs/Twist.h>

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
class CmdVelToWheelVelCmd
{
private:
	ros::NodeHandle n_;
protected:
	ros::NodeHandle node;

	std::string cmd_vel_topic_name_;
	std::string wheels_vel_cmd_topic_name_;

	ros::Publisher wheels_vel_cmd_pub_;
	
	ros::Subscriber cmd_vel_sub_;
	
	double max_wheel_vel;
	double robot_width;
public:
	CmdVelToWheelVelCmd();
	
	void cmd_vel_callback(const geometry_msgs::TwistConstPtr& msg);
	void getWheelsCmdVel(double linear_vel, double angular_vel, double robot_width, nifti_robot_driver_msgs::TracksStamped& tracks_cmd);
};

CmdVelToWheelVelCmd::CmdVelToWheelVelCmd(): n_("~"){
	cmd_vel_topic_name_ = getParam<std::string>(n_,"cmd_vel_topic_name", "/cmd_vel");
	cmd_vel_sub_ = node.subscribe(cmd_vel_topic_name_, 1, &CmdVelToWheelVelCmd::cmd_vel_callback, this);

	wheels_vel_cmd_topic_name_ = getParam<std::string>(n_,"wheels_vel_cmd_topic_name", "/wheels_vel_cmd");
	wheels_vel_cmd_pub_ = node.advertise<nifti_robot_driver_msgs::TracksStamped>(wheels_vel_cmd_topic_name_, 1);

	max_wheel_vel = getParam<double>(n_, "max_wheel_vel_value", 2.0f);
	robot_width = getParam<double>(n_, "robot_width_value", 0.354f);
}

void CmdVelToWheelVelCmd::cmd_vel_callback(const geometry_msgs::TwistConstPtr& msg)
{
	nifti_robot_driver_msgs::TracksStamped tracks_cmd;
  geometry_msgs::Twist callback_msg;

  double v = msg->linear.x;
  double w = msg->angular.z;

	getWheelsCmdVel(v, w, robot_width, tracks_cmd);
	wheels_vel_cmd_pub_.publish(tracks_cmd);
}

void CmdVelToWheelVelCmd::getWheelsCmdVel(double linear_vel, double angular_vel, double robot_width, nifti_robot_driver_msgs::TracksStamped& tracks_cmd)
{
	double d = robot_width/2;

	tracks_cmd.left = linear_vel - (d * angular_vel);
	tracks_cmd.right = linear_vel + (d * angular_vel);

	if(tracks_cmd.left < -max_wheel_vel)
	{
		tracks_cmd.left = -max_wheel_vel;
	}
	if(tracks_cmd.right > max_wheel_vel)
	{
		tracks_cmd.left = max_wheel_vel;
	}
	if(tracks_cmd.right < -max_wheel_vel)
	{
		tracks_cmd.right = -max_wheel_vel;
	}
	if(tracks_cmd.right > max_wheel_vel)
	{
		tracks_cmd.right = max_wheel_vel;
	}
}

#endif /* TRAJECTORY_CONTROLLER _H */

int main(int argc, char* argv[]){
	ros::init(argc, argv, "cmd_vel_to_wheel_vel_cmd");
	CmdVelToWheelVelCmd cvtwvc;
	ros::spin();
}
