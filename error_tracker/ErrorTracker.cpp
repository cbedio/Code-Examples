#include <error_tracker/ErrorTracker.h>

ErrorTracker::ErrorTracker():
n_("~"),
policy(new MyPolicy(10)),
sync_(new message_filters::Synchronizer<MyPolicy>(*policy))
{
	//client_name = getParam<std::string>(n_,"client_name", "move_base");
	//ac_ = new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>(client_name, true);
	
	filename = getParam<std::string>(n_, "filename", "filename.csv");

	global_path_topic_name = getParam<std::string>(n_, "global_path_topic", "/move_base/TebLocalPlannerROS/global_plan");
	global_path_sub.subscribe(node,global_path_topic_name,1);
	
	local_path_topic_name = getParam<std::string>(n_, "local_path_topic", "/move_base/TebLocalPlannerROS/local_plan");
	local_path_sub.subscribe(node,local_path_topic_name,1);

	sync_->connectInput(global_path_sub,local_path_sub);
	sync_->registerCallback(&ErrorTracker::path_error_callback,this);
	

	//ac_->waitForServer();
}

ErrorTracker::~ErrorTracker(){}

void ErrorTracker::path_error_callback(const nav_msgs::PathConstPtr& global_path, const nav_msgs::PathConstPtr& local_path)
{
	std_msgs::Float64 error;	

	geometry_msgs::Pose current_pose = local_path->poses[0].pose;
	geometry_msgs::Pose expected_pose = global_path->poses[0].pose;

	error.data = sqrt(pow(current_pose.position.x - expected_pose.position.x,2) + pow(current_pose.position.y - expected_pose.position.y,2));

	FILE *file=fopen(filename.c_str(),"a");
	fprintf(file,"%lf,"
			"%lf\n",
			error.data);
	
	fclose(file);
	
}

int main(int argc, char* argv[])
{
	ros::init (argc, argv, "error_tracker");
	ErrorTracker error_tracker;
	ros::spin();
	return 0;
}

