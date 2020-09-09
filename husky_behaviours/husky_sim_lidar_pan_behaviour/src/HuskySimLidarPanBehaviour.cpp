#include <husky_sim_lidar_pan_behaviour/HuskySimLidarPanBehaviour.h>

HuskySimLidarPanBehaviour::HuskySimLidarPanBehaviour(): n_("~"){
	
	parent_link_name = getParam<std::string>(n_, "parent_link_name", "/bottom_lidar_mount_link");
	child_link_name = getParam<std::string>(n_, "child_link_name", "/top_lidar_mount_link");

	pan_speed = fabs(getParam<double>(n_, "pan_speed", 1));

	max_angle = getParam<double>(n_, "max_angle", 1.569);
	min_angle = getParam<double>(n_, "min_angle", -1.569);

	frequency = getParam<double>(n_, "frequency", 100);
	period = 1/frequency;
	
	lidar_cmd_topic_name = getParam<std::string>(n_, "lidar_cmd_topic_name", "/husky_lidar_velocity_controller/command");
	lidar_cmd_pub = node.advertise<std_msgs::Float64>(lidar_cmd_topic_name,1);

	timer = node.createTimer(ros::Duration(period), &HuskySimLidarPanBehaviour::timerCallback, this);

	std_msgs::Float64 initial_lidar_cmd;
	initial_lidar_cmd.data = pan_speed;

}

HuskySimLidarPanBehaviour::~HuskySimLidarPanBehaviour(){}

void HuskySimLidarPanBehaviour::timerCallback(const ros::TimerEvent& event){
	
	tf::StampedTransform transform;	

	std_msgs::Float64 lidar_cmd;
	
	try {
		listener.lookupTransform(parent_link_name, child_link_name, ros::Time(0), transform);
	}
	catch (tf::TransformException &ex) {
		ROS_ERROR("%s", ex.what());
		ros::Duration(1.0).sleep();
		return;
	}

	tf::Quaternion q = transform.getRotation();

	tf::Matrix3x3 m(q);

	double roll, pitch, yaw;

	m.getRPY(roll,pitch, yaw);

	ROS_INFO("%f",yaw);

	if (yaw >= max_angle) {

		pan_speed = fabs(pan_speed);

		pan_speed = -pan_speed;

		lidar_cmd.data = pan_speed;

		lidar_cmd_pub.publish(lidar_cmd);

	} else if (yaw <= min_angle) {

		pan_speed = fabs(pan_speed);

		lidar_cmd.data = pan_speed;

		lidar_cmd_pub.publish(lidar_cmd);
	}
}


int main(int argc, char** argv){
	ros::init(argc, argv, "husky_sim_lidar_pan_behaviour");

	HuskySimLidarPanBehaviour hslpb;

	ros::spin();
	
	return 0;
}

