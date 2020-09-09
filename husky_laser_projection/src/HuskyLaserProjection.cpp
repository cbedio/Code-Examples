#include <husky_laser_projection/HuskyLaserProjection.h>

HuskyLaserProjection::HuskyLaserProjection(): n_("~"){
	laser_scan_topic_name = getParam<std::string>(n_,"laser_scan_topic_name", "/scan");
	laser_scan_sub = laser_scan_sub = node.subscribe(laser_scan_topic_name, 1, &HuskyLaserProjection::laser_scan_callback, this);

	cloud_topic_name = getParam<std::string>(n_,"cloud_topic_name", "/cloud_out");
	cloud_pub = node.advertise<sensor_msgs::PointCloud2>(cloud_topic_name,1);

	tf_extrapolation_limit = getParam<double>(n_,"tf_extrapolation_limit", 0.1);
	tfListener.setExtrapolationLimit(ros::Duration(tf_extrapolation_limit));

	target_frame_name = getParam<std::string>(n_,"target_frame_name", "laser_link");
}

HuskyLaserProjection::~HuskyLaserProjection() {}

void HuskyLaserProjection::laser_scan_callback(const sensor_msgs::LaserScanConstPtr& callback_msg){
	sensor_msgs::PointCloud2 cloud_msg;

	projector.transformLaserScanToPointCloud(target_frame_name, *callback_msg, cloud_msg, tfListener);
	cloud_pub.publish(cloud_msg);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "husky_laser_projection");

	HuskyLaserProjection hlp;

	ros::spin();

	return 0;
}
