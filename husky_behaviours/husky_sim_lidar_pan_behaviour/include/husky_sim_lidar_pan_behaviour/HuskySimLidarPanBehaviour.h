#ifndef HUSKYSIMLIDARPANBEHAVIOUR_H
#define HUSKYSIMLIDARPANBEHAVIOUR_H

#include "string"
#include "ros/ros.h"
#include "stdio.h"
#include "math.h"

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float64.h>

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
		ROS_WARN_STREAM("Cannot find value for parameter: " << name << ", assigning default: " << defaultValue);
	return defaultValue;
}

class HuskySimLidarPanBehaviour
{
private:
	
	ros::NodeHandle n_;

protected:

	ros::NodeHandle node;

	std::string lidar_cmd_topic_name;
	ros::Publisher lidar_cmd_pub;

	std::string parent_link_name;
	std::string child_link_name;

	double pan_speed;

	double max_angle;
	double min_angle;

	double frequency;

	double period;

	ros::Timer timer;

	tf::TransformListener listener;
	
public:

	HuskySimLidarPanBehaviour();
	~HuskySimLidarPanBehaviour();
	
	void timerCallback(const ros::TimerEvent& event);
};

#endif
