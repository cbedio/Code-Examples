#ifndef HUSKY_SCANS_ASSEMBLE_CLIENT_H
#define HUSKY_SCANS_ASSEMBLE_CLIENT_H

#include <string>
#include <stdio.h>
#include <ros/ros.h>
#include <angles/angles.h>

#include <laser_assembler/AssembleScans2.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
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
    {
        ROS_WARN_STREAM("Cannot find value for parameter: " << name << ", assigning default: " << defaultValue);
    }
    return defaultValue;
}
class HuskyScansAssembleClient
{
private:
    ros::NodeHandle n_;

	double angle_max_;
	double angle_min_;

	ros::Time time_start_;
	bool cloud_created_flag_;
	
protected:
    ros::NodeHandle node;

	std::string assemble_service_name;

	std::string sim_lidar_position_topic_name;
	std::string joint_states_topic_name;

	std::string sim_cloud_topic_name;
	std::string non_sim_cloud_topic_name;


public:
	ros::ServiceClient client_;

	ros::Publisher sim_point_cloud_pub;
	ros::Publisher non_sim_point_cloud_pub;

    ros::Subscriber sim_lidar_position_sub;
	ros::Subscriber joint_states_sub;

    HuskyScansAssembleClient();
    ~HuskyScansAssembleClient();

    void sim_lidar_position_callback(const std_msgs::Float64 &msg);
	void joint_states_callback(const sensor_msgs::JointStateConstPtr& joint_msg_const);



};

HuskyScansAssembleClient::HuskyScansAssembleClient(): n_("~"){
	
	angle_max_ = getParam<double>(n_, "angle_max_param_value", 3.14);
	angle_min_ = getParam<double>(n_, "angle_min_param_value", 0);

	assemble_service_name = getParam<std::string>(n_, "assemble_service_name", "/assemble_scans2");
	ros::service::waitForService(assemble_service_name);
	client_ = node.serviceClient<laser_assembler::AssembleScans2>(assemble_service_name);

	sim_lidar_position_topic_name = getParam<std::string>(n_, "sim_lidar_position_topic_name", "/husky_joint_position_controller/command");
    sim_lidar_position_sub = node.subscribe(sim_lidar_position_topic_name, 1, &HuskyScansAssembleClient::sim_lidar_position_callback, this);
	
	joint_states_topic_name = getParam<std::string>(n_, "joint_states_topic_name", "/joint_states");
	// joint_states_sub = node.subscribe(joint_states_topic_name, 1, &HuskyScansAssembleClient::joint_states_callback, this);

   	sim_cloud_topic_name = getParam<std::string>(n_,"sim_cloud_topic_name", "/sim_cloud");
    sim_point_cloud_pub = node.advertise<sensor_msgs::PointCloud2>(sim_cloud_topic_name,1);

	non_sim_cloud_topic_name = getParam<std::string>(n_,"non_sim_cloud_topic_name", "/assembled_cloud");
	non_sim_point_cloud_pub = node.advertise<sensor_msgs::PointCloud2>(non_sim_cloud_topic_name,1);

	time_start_ = ros::Time::now();
}
HuskyScansAssembleClient::~HuskyScansAssembleClient(){}

void HuskyScansAssembleClient::sim_lidar_position_callback(const std_msgs::Float64 &msg){
    if ((msg.data <= angle_min_) || (msg.data >= angle_max_)) {
        laser_assembler::AssembleScans2 srv;
        ros::Time time_end = ros::Time::now();
        srv.request.begin = time_start_;
        srv.request.end = time_end;
        time_start_ = time_end;
        if (client_.call(srv)) {
            //AssembleScans
            //printf("Got cloud with %u points\n", (uint32_t)(srv.response.cloud.points.size()));

            //AssembleScans2
            printf("Got cloud with %u points\n", srv.response.cloud.data.size());

            if (srv.response.cloud.data.size() > 0) {
              sim_point_cloud_pub.publish(srv.response.cloud);
            }
        } else {
            printf("Service call failed");
        }
    }
}

void HuskyScansAssembleClient::joint_states_callback(const sensor_msgs::JointStateConstPtr& joint_msg_const){
	sensor_msgs::JointState joint_msg;
	joint_msg.name = joint_msg_const->name;
	joint_msg.position = joint_msg_const->position;
    
	std::vector<std::string>::iterator iterator = std::find(joint_msg.name.begin(), joint_msg.name.end(), "top_lidar_mount_link_joint");

	if (iterator != joint_msg.name.end()){	
		int index = std::distance(joint_msg.name.begin(), iterator);
		double position = joint_msg.position[index];
		if (((position <= angle_min_) || (position >= angle_max_))&&(!cloud_created_flag_)) {
			cloud_created_flag_ = 1;
		    laser_assembler::AssembleScans2 srv;
		    ros::Time time_end = ros::Time::now();
		    srv.request.begin = time_start_;
		    srv.request.end = time_end;
		    time_start_ = time_end;
			ROS_INFO("Calling Service");
			ROS_INFO("%f",position);
		    if (client_.call(srv)) {
		        //AssembleScans
		        //printf("Got cloud with %u points\n", (uint32_t)(srv.response.cloud.points.size()));

		        //AssembleScans2
		        printf("Got cloud with %u points\n", srv.response.cloud.data.size());

		        if (srv.response.cloud.data.size() > 10000) {
		          non_sim_point_cloud_pub.publish(srv.response.cloud);
		        }
		    } else {
		        printf("Service call failed");
		    }
		} else {
			cloud_created_flag_ = 0;
		}
	}
}


#endif /* HUSKY_SCANS_ASSEMBLE_CLIENT_H */

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "husky_scan_assemble_client");
    HuskyScansAssembleClient husky_scans_assemble_client;
    ros::spin();
}
