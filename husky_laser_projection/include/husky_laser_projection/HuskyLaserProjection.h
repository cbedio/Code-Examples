#ifndef HUSKYLASERPROJECTION_H
#define HUSKYLASERPROJECTION_H

#include "string"
#include "ros/ros.h"
#include "stdio.h"

#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

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

class HuskyLaserProjection 
{
private:
	
	ros::NodeHandle n_;

protected:
	
	ros::NodeHandle node;

	std::string laser_scan_topic_name;
	ros::Subscriber laser_scan_sub;

	std::string cloud_topic_name;
	ros::Publisher cloud_pub;
	
	std::string target_frame_name;

	double tf_extrapolation_limit;

	laser_geometry::LaserProjection projector;
	tf::TransformListener tfListener;

public:
	HuskyLaserProjection();
	~HuskyLaserProjection();

	void laser_scan_callback(const sensor_msgs::LaserScanConstPtr& scan_msg);
};

#endif
