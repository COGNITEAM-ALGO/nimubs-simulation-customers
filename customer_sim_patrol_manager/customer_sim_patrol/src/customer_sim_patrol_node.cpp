

#include <customer_sim_patrol/Patrol.h>
#include <signal.h>
#include <ros/ros.h>

using namespace std;


bool readWaypoints(vector<WayPoint>& waypoints) {
	
	ros::NodeHandle nodePrivate("~");

	vector<string> waypointsList;


	if (nodePrivate.getParam("waypoints", waypointsList)) {

		geometry_msgs::PoseStamped pose;

		int line = 1;

		for(auto waypointString : waypointsList) {
			double heading = 0;

			auto parsedValues = sscanf(waypointString.c_str(), "%lf,%lf,%lf",
					&pose.pose.position.x,
					&pose.pose.position.y,
					&heading);

			pose.header.frame_id = "map";
			pose.header.stamp = ros::Time(0);

			pose.pose.orientation = tf::createQuaternionMsgFromYaw(heading);

			WayPoint waypoint;
			waypoint.w_pose_ = pose;
			waypoints.push_back(waypoint);

			if (parsedValues < 3) {
				ROS_ERROR("Failed to parse a waypoint (line %i)", line);
				return false;
			}

			line++;
		}

	} else {


		ROS_ERROR("Error: waypoints parameter does not exists or empty");
		
		return false;
	}

	return true;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "customer_sim_patrol_node");

	vector<WayPoint> waypoints;

	if (readWaypoints(waypoints) == false) {
		
		ROS_WARN("Waypoints list is empty, exiting");
		cerr<<" Waypoints list is empty, exiting "<<endl;
		
		return -1;

	} else {

		signal(SIGINT, (void (*)(int))Patrol::mySigintHandler); 
	
		Patrol patrol(waypoints);

		patrol.run();

		ros::spin();
	}




	return 0;
}

