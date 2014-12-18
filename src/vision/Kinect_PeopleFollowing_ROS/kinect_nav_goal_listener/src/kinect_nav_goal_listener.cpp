// kinect_nav_goal_listener.cpp

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include <tf/tf.h>
#include "tf/transform_listener.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>


#define PRINT_OUTPUT 1
#define RUN_WITH_HARDWARE_ATTACHED 0  // if actuators and sensors are attached via USB set to "1" 


class KinectNavGoalListener
{
  public: 
    KinectNavGoalListener();
    void detectionCallBack(const geometry_msgs::Twist::ConstPtr& msg);
    void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void poseStampedCallBack(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void runActionClient(void);
    void runConcurrencyTest(void);  // test

  private:
    ros::NodeHandle nh;
    ros::Subscriber sub_target, sub_initialpose, sub_amcl_pose, sub_pose_stamped;
    ros::Publisher  pub_move_base_simple;
    geometry_msgs::Twist saved_msg;
    volatile double saved_lin_x, saved_lin_y, saved_ang_z; 
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> *ac;  
    move_base_msgs::MoveBaseGoal goal;
    geometry_msgs::Quaternion q_msg_in, q_msg_out;  // temp vars
    tf::Quaternion current_pose_q, delta_z_q;
    geometry_msgs::PoseWithCovarianceStamped robot_pose_estimate;
    geometry_msgs::PoseStamped msg_to_publish;
    bool initialPosePublished;
		geometry_msgs::PointStamped laser_point, map_point;
    tf::TransformListener tfListener;
    tf::StampedTransform tfLazerToMap;
    double lazerPositionX, lazerPositionY;
    tf::Stamped<tf::Pose> person_global;      // transformed point into “global”, or “map”
};



// This function is called when the initialpose topic is published 
// (e.g. by RVIZ user input):
void KinectNavGoalListener::initialPoseCallback(
  const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
  // TODO: Only checks if "initialpose" is published. We might 
  //       need to check if a given confidence value is reached.
  initialPosePublished = true;
}



// This function is called when a pose estimation is published by AMCL node:
void KinectNavGoalListener::amclPoseCallback( 
  const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
  // this works when viewed by 'rostopic echo /amcl_pose':
  robot_pose_estimate = geometry_msgs::PoseWithCovarianceStamped(*msg);
}



KinectNavGoalListener::KinectNavGoalListener()
{
    initialPosePublished = false;
    sub_target = nh.subscribe<geometry_msgs::Twist>
	      ("/detected_person", 1, &KinectNavGoalListener::detectionCallBack, this);
    sub_initialpose = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>
        ("initialpose", 1, &KinectNavGoalListener::initialPoseCallback, this);
    sub_amcl_pose = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>
        ("amcl_pose", 1, &KinectNavGoalListener::amclPoseCallback, this);
    pub_move_base_simple = nh.advertise<geometry_msgs::PoseStamped>
        ("move_base_simple/goal", 1);
    sub_pose_stamped = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>
        ("pose_stamped", 1, &KinectNavGoalListener::poseStampedCallBack, this);


    // MoveBase SimpleActionClient: "true/false: spin a thread by default
    // "move_base" is not a topic here, but it is a namespace used by Server
    // Server "move_base" uses "/move_base_simple/goal" to send messages to base
    ac = new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
        ("move_base", true);
    
    runActionClient();   //  first, run without Kinect, manually set target
    // runConcurrencyTest();    


}


void KinectNavGoalListener::runConcurrencyTest() {
  while (ros::ok()) {
    ros::spinOnce();
    ROS_INFO("saved_msg.linear.x: %f, saved_msg.linear.y: %f, saved_msg.angular.z: %f",
              saved_msg.linear.x, saved_msg.linear.y, saved_msg.angular.z );
    ROS_INFO("runConcurrencyTest(): sleeping for  10 seconds ...");
    ros::Duration(2).sleep();
  }
}


void KinectNavGoalListener::runActionClient(void){
  while( ros::ok()) { 
		ros::spinOnce();
		if (RUN_WITH_HARDWARE_ATTACHED ) {
		  ROS_INFO("runActionClient dummy function: checking for ActionServer.");
		  // ActionServer will not come up if hardware is not detected (not attached):
		  while(!ac->waitForServer(ros::Duration(3.0) ) && nh.ok() ){
		    ROS_INFO("Waiting for the move_base action server to come up ...");
		  }
		  ROS_INFO("\"move_base\" action server is running.");
		}

    /*
    // ============== Debugging: ================================================
    ROS_INFO("Checking spinning, detections:          x: %f, y: %f, z: %f",
             saved_msg.linear.x, saved_msg.linear.y, saved_msg.angular.z);
    // ROS_INFO("Checking spinning, detections volatile: x: %f, y: %f, z: %f",
    //          saved_lin_x, saved_lin_y, saved_ang_z);
    // saved_msg.linear.x = 1.0;
    // saved_msg.linear.y = 0.0;
    // saved_msg.angular.z = 0.0;
    // ==========================================================================
		ros::Duration(2).sleep();
    */

    
		try { 
			tfListener.lookupTransform("map", "lazer", ros::Time(0), tfLazerToMap);
		  current_pose_q = tfLazerToMap.getRotation();
		  lazerPositionX = tfLazerToMap.getOrigin().x();  // current robot (lazer) position
		  lazerPositionY = tfLazerToMap.getOrigin().y();
		} 
		catch (tf::TransformException &ex) {
			ROS_ERROR (	"%s", ex.what() );
		}

		delta_z_q = tf::createQuaternionFromYaw(saved_msg.angular.z);  // angle of person in current frame ("laser" or "kinect")
		current_pose_q *= delta_z_q;  // rotate quarternion by delta
		tf::Stamped<tf::Pose> person_local( tf::Pose(current_pose_q, 
																				tf::Vector3(saved_msg.linear.x, saved_msg.linear.y, 0.0)), 
																				ros::Time(0), "laser" );
		tfListener.transformPose("map", person_local, person_global);

		// convert "tf::Stamped<tf::Pose> person_local" to  "move_base_msgs::MoveBaseGoal goal"
		tf::Quaternion person_global_q = person_global.getRotation();
		tf::quaternionTFToMsg(person_global_q, q_msg_out);
		goal.target_pose.pose.orientation = q_msg_out;
		goal.target_pose.pose.position.x = person_global.getOrigin().x();
		goal.target_pose.pose.position.y = person_global.getOrigin().y();
		goal.target_pose.pose.position.z = 0.0;


		// ====== display coords of attempted move: ============================  
		ROS_INFO("Current lazer position: x: %f, y: %f", lazerPositionX, lazerPositionY);
		ROS_INFO("Moving to               x: %f, y: %f", 
              goal.target_pose.pose.position.x, goal.target_pose.pose.position.y  );
		// ros::Duration(60).sleep();


    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    ac->sendGoal(goal);      
    bool finished_before_timeout = ac->waitForResult(ros::Duration(30.0));
    actionlib::SimpleClientGoalState state = ac->getState();
    if (finished_before_timeout) { 
      ROS_INFO("\t\tFINISHED: %s\n\n", state.toString().c_str());        
    }
    else 
      ROS_INFO("\t\tFAILED: %s\n\n", state.toString().c_str());
  }
}


void KinectNavGoalListener::poseStampedCallBack(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  // robot_pose_estimate = geometry_msgs::PoseWithCovarianceStamped(*msg);
}


void KinectNavGoalListener::detectionCallBack(const geometry_msgs::Twist::ConstPtr& msg)
{  
	saved_ang_z = saved_msg.angular.z = msg->angular.z;
  saved_lin_x = saved_msg.linear.x = msg->linear.x;
  saved_lin_y = saved_msg.linear.y = msg->linear.y;
  ROS_INFO("\n!!!!!! DETECTIONS CALLBACK: x: %f, y: %f, z: %f\n", saved_lin_x, saved_lin_y, saved_ang_z);    
}



int main(int argc, char** argv)
{  
    ros::init(argc, argv, "kinect_nav_goal_listener");
    KinectNavGoalListener kngl; 
    
    /*
    if (RUN_WITH_HARDWARE_ATTACHED ) {
    	// ActionServer will not come up if hardware is not detected (not attached):
    	while(!kngl.ac->waitForServer(ros::Duration(3.0) ) && kngl.nh.ok() ){
            ROS_INFO("Waiting for the move_base action server to come up ...");
        }
    }
    */

    if (PRINT_OUTPUT)
        ROS_INFO("spinning node \"kinect_nav_goal_listener\"");
    // ros::spin();
    // ros::rate r(1)
    return 0;
}




