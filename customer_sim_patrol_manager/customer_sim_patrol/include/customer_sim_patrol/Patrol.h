/*
 * Patrol.h
 *
 *  Created on: Nov 9, 2021
 *      Author: yakirhuri
 */



#include <vector>
#include<string.h>
#include <tf/tf.h>
#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <nav_msgs/Path.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <string>
#include <nav_msgs/Path.h>
#include <angles/angles.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>


#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

#include "MoveBaseController.h"
#include "logManger.h"

#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;


struct WayPoint {

	geometry_msgs::PoseStamped w_pose_;

	bool status_ = false;
};


enum RobotState
{
  IDLE = 0,
  PATROL = 2,
  ERROR = 4
};


class Patrol {



public:

	Patrol(vector<WayPoint> waypoints){

		waypoints_ = waypoints;


		ros::NodeHandle node_p("~");


		node_p.param("/robot_state", robotState_, string("IDLE"));
		robotState_ = "IDLE";

		backDockWaypoint_.header.frame_id = globalFrame_;
        backDockWaypoint_.header.stamp = ros::Time::now();
		backDockWaypoint_.pose.orientation =  tf::createQuaternionMsgFromYaw(initial_pose_a_);
		backDockWaypoint_.pose.position.x = 	initial_pose_x_ + (-1.0 * cos(initial_pose_a_));
		backDockWaypoint_.pose.position.y = 	initial_pose_y_ + (-1.0 * sin(initial_pose_a_));


		//pubs

		goals_marker_array_publisher_ =
        	node.advertise<visualization_msgs::MarkerArray>("/waypoints_markers", 10);	
		
		path_pub_ = node.advertise<nav_msgs::Path>("/patrol_path", 1, false);

		
	
	}

	virtual ~Patrol(){

		logManager_.closeFile();
	}

	static void mySigintHandler(int sig, void *ptr)
    {

        cerr << " user pressed CTRL+C " << endl;
    }

	void setState(RobotState state)
	{	
		switch (state)
		{
			case IDLE: {
				node.setParam("/robot_state", "IDLE");
				robotState_ = "IDLE";
				return;
			}			
			case PATROL: {
				node.setParam("/robot_state", "PATROL");
				robotState_ = "PATROL";
				return;
			}			
			case ERROR: {
				node.setParam("/robot_state", "ERROR");
				robotState_ = "ERROR";
				return;
			}				
		}

		
	}

	bool checkLocalizationOk() {

        tf::StampedTransform transform;

        try
        {
            //get current robot pose
            tfListener_.lookupTransform(globalFrame_, odomFrame_,
                                       ros::Time(0), transform);

            return true;
        }

        catch (...)
        {
            //cerr << " error between map to odom"<<endl;
            return false;
        }
    }

	double distanceCalculate(cv::Point2d p1, cv::Point2d p2)
    {
        double x = p1.x - p2.x; //calculating number to square in next step
        double y = p1.y - p2.y;
        double dist;

        dist = pow(x, 2) + pow(y, 2); //calculating Euclidean distance
        dist = sqrt(dist);

        return dist;
    }

	void run(){

		if ( checkIfRobtotIsCharged()){
		}


		//verify amcl works and the robot have location
		bool recv_map_odom = false;
		cerr<<" waiting for robot's location ... "<<endl;

		while( ros::ok() && !recv_map_odom) {	
			
			// cerr<<" waiting for robot's location ... "<<endl;
			ros::spinOnce();

			recv_map_odom =  checkLocalizationOk();
			
		}


		ROS_INFO("Waiting for /move_base action server...");
		moveBaseController_.waitForServer();
		
		ros::Duration(1).sleep();
		
		ROS_INFO("Connected to /move_base!\n");
		
		logManager_.writeToLog("mission started");


		publishWaypointsWithStatus();

		publishPath();

		
		setState(PATROL);

		for (int i = 0; i < waypoints_.size(); i++){
			
			publishPath();

			bool reachedGoal = false;

			if ( !abort_){

				cerr<<" navigate to "<<waypoints_[i].w_pose_.pose.position.x<<" , "<<waypoints_[i].w_pose_.pose.position.y<<endl;
				logManager_.writeToLog("sending goal # "+to_string(i));
			}
			

			moveBaseController_.navigate(waypoints_[i].w_pose_);

			while(ros::ok()) {

				ros::spinOnce();

				updateRobotLocation();				

				float distDromRobot =
					distanceCalculate(cv::Point2d(waypoints_[i].w_pose_.pose.position.x, waypoints_[i].w_pose_.pose.position.y),
									cv::Point2d(robotPose_.pose.position.x, robotPose_.pose.position.y));

				cerr<<" distDromRobot "<<distDromRobot<<endl;
				

				moveBaseController_.moveBaseClient_.waitForResult(ros::Duration(0.1));
				
				if( waypoints_[i].status_ ==  true ||
					moveBaseController_.moveBaseClient_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED 
					||  moveBaseController_.moveBaseClient_.getState() == actionlib::SimpleClientGoalState::ABORTED
					||  moveBaseController_.moveBaseClient_.getState() == actionlib::SimpleClientGoalState::REJECTED) {
					
					if( moveBaseController_.moveBaseClient_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED ||
						waypoints_[i].status_ ==  true){
						waypoints_[i].status_ = true;
						cerr<<" goal reached !! "<<endl;
						
					} else {

						if (distDromRobot < 0.15)
						{	
							cerr<<" dist is good "<<endl;
							waypoints_[i].status_ = true;
							moveBaseController_.moveBaseClient_.cancelGoal();

						} else {

							waypoints_[i].status_ = false;
							cerr<<" goal failed even with dist !! "<<endl;
						}						

					}

					publishWaypointsWithStatus();					

					break;
				} 
			}

		}

		cerr<<"FINSIHED PATROL!!!!! "<<endl;


	}

	
	

private:

	
	
	void publishPath() {

		nav_msgs::Path pathMsg;
		pathMsg.header.frame_id = globalFrame_;

		for (int i = 0; i < waypoints_.size(); i++){
			pathMsg.poses.push_back(waypoints_[i].w_pose_);
		}

		path_pub_.publish(pathMsg);

	}
	bool checkIfRobtotIsCharged() {

		return true;
	}

	void publishInitialPose() {

		{
			visualization_msgs::Marker marker;
			marker.lifetime = ros::Duration(0.1);
			marker.action = visualization_msgs::Marker::ADD;
			marker.type = visualization_msgs::Marker::ARROW;
			marker.header.frame_id = "map";
			marker.header.stamp  = ros::Time::now(); 
			marker.id = rand();
			marker.pose.position.x =  initial_pose_x_;
			marker.pose.position.y = initial_pose_y_;
			marker.pose.position.z = 0;

			marker.pose.orientation = tf::createQuaternionMsgFromYaw(initial_pose_a_);
			marker.scale.x = 0.2;
			marker.scale.y = 0.2;                
			marker.scale.z = 0.2;

			marker.color.r = 0.0f;
			marker.color.g = 0.0f;
			marker.color.b = 0.0f;
			marker.color.a = 1.0;

			marker.lifetime = ros::Duration(500);


			text_duration_publisher_.publish(marker);
		}
		

		/// BACK

		{
			visualization_msgs::Marker marker;
			marker.lifetime = ros::Duration(0.1);
			marker.action = visualization_msgs::Marker::ADD;
			marker.type = visualization_msgs::Marker::ARROW;
			marker.header.frame_id = backDockWaypoint_.header.frame_id;
			marker.header.stamp  = ros::Time::now(); 
			marker.id = rand();
			marker.pose.position    =  backDockWaypoint_.pose.position;			
			marker.pose.orientation = backDockWaypoint_.pose.orientation;
			marker.scale.x = 0.2;
			marker.scale.y = 0.2;                
			marker.scale.z = 0.2;

			marker.color.r = 210.0 / 255.0;
			marker.color.g = 105.0 / 255.0;
			marker.color.b = 30.0 / 255.0;
			marker.color.a = 1.0;

			

			marker.lifetime = ros::Duration(500);


			text_duration_publisher_.publish(marker);
		}

		

	}

	void setYawTolerance(float yawTolerance = 0.0)
	{
		dynamic_reconfigure::ReconfigureRequest srv_req;
		dynamic_reconfigure::ReconfigureResponse srv_resp;
		dynamic_reconfigure::DoubleParameter float_param;
		dynamic_reconfigure::Config conf;

		float_param.name = "yaw_goal_tolerance";
		float_param.value = yawTolerance;
		conf.doubles.push_back(float_param);

		srv_req.config = conf;
		if (ros::service::call("/move_base/DWAPlannerROS/set_parameters", srv_req, srv_resp))
		{
		cerr << " after set reverse " << endl;
		}
		else
		{
		cerr << "errrrrrrrrrrrrrrrrrrrrr " << endl;
		}

		ros::Duration(0.5).sleep();
		
	}


	

	void publishWaypointsWithStatus() {

		visualization_msgs::MarkerArray markers;


		for (int i = 0; i < waypoints_.size(); i++) {

			visualization_msgs::Marker marker;
			marker.lifetime = ros::Duration(100.0);
			marker.action = visualization_msgs::Marker::ADD;
			marker.type = visualization_msgs::Marker::ARROW;
			marker.header.frame_id = "map";
			marker.header.stamp  = ros::Time::now(); 
			marker.id = i;
			marker.pose.position = waypoints_[i].w_pose_.pose.position;
			marker.pose.orientation = waypoints_[i].w_pose_.pose.orientation;
			marker.scale.x = 0.2;
			marker.scale.y = 0.2;                
			marker.scale.z = 0.2;

			if( waypoints_[i].status_){
				marker.color.r = 0.0f;
				marker.color.g = 1.0f;
				marker.color.b = 0.0f;
				marker.color.a = 1.0;
		
			} else {
				marker.color.r = 1.0f;
				marker.color.g = 0.0f;
				marker.color.b = 0.0f;
				marker.color.a = 1.0;
			}
			
			
		
			markers.markers.push_back(marker);              
		}

		goals_marker_array_publisher_.publish(markers);

	}

	bool updateRobotLocation()
    {

        tf::StampedTransform transform;

        try
        {
            // get current robot pose
            tfListener_.lookupTransform(globalFrame_, baseFrame_,
                                        ros::Time(0), transform);

            robotPose_.header.frame_id = globalFrame_;
            robotPose_.header.stamp = ros::Time::now();
            robotPose_.pose.position.x = transform.getOrigin().x();
            robotPose_.pose.position.y = transform.getOrigin().y();
            robotPose_.pose.position.z = 0;
            robotPose_.pose.orientation.x = transform.getRotation().x();
            robotPose_.pose.orientation.y = transform.getRotation().y();
            robotPose_.pose.orientation.z = transform.getRotation().z();
            robotPose_.pose.orientation.w = transform.getRotation().w();       

            return true;
        }

        catch (...)
        {
            cerr << " error between " << globalFrame_ << " to " << baseFrame_ << endl;
            return false;
        }
    }

	

private:

	ros::NodeHandle node;

	geometry_msgs::PoseStamped robotPose_; // on map frame

	geometry_msgs::PoseStamped backDockWaypoint_;

	string robotState_ = "IDLE";
	
	//move-base
	MoveBaseController moveBaseController_;

	//log-manager
	LogManager logManager_;

	tf::TransformListener tfListener_;
	string globalFrame_ = "map";
	string baseFrame_ = "base_link";
	string odomFrame_ = "odom";

	//ros params	
	double numOfRounds_ = 0;
	double rotationSpeed_ = 0.3;
	double wait_duration_seconds_ = 0.0;
	string log_path_ = "";

	double initial_pose_x_ = 0.0;
	double initial_pose_y_ = 0.0;
	double initial_pose_a_ = 0.0;


	bool abort_ = false;


	ros::Publisher	twistPub_ ;
	ros::Publisher goals_marker_array_publisher_ ;
	ros::Publisher text_duration_publisher_ ;
	ros::Publisher path_pub_;
	

	ros::Subscriber abort_sub_;


	// system params	
	vector<WayPoint> waypoints_;

};

