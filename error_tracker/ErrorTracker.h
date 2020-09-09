#ifndef ERRORTRACKER_H
#define ERRORTRACKER_H

#include <string>
#include <stdio.h>
#include <ros/ros.h>

#include <math.h>

#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float64.h>

//#include <move_base_msgs/MoveBaseAction.h>
//#include <actionlib/client/simple_action_client_client.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Path, nav_msgs::Path> MyPolicy;

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

class ErrorTracker{

private:

	ros::NodeHandle n_;

protected:

	//actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>* ac_;

	ros::NodeHandle node;

	std::string filename;

	//std::string client_name;

	std::string global_path_topic_name;
	std::string local_path_topic_name;

public:

	MyPolicy* policy;
	message_filters::Synchronizer<MyPolicy>* sync_;

	message_filters::Subscriber<nav_msgs::Path> global_path_sub;
	message_filters::Subscriber<nav_msgs::Path> local_path_sub;
	
	ErrorTracker();
	~ErrorTracker();

	void path_error_callback(const nav_msgs::PathConstPtr& global_path, const nav_msgs::PathConstPtr& local_path);

};

#endif /* ERRORTRACKER_H */
