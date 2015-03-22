// kinect_nav_goal_listener.cpp
// 2015.01.18, 2015.03.16
// Tested and working Person Following.


#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include <tf/tf.h>
#include "tf/transform_listener.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <iostream>


#define PRINT_OUTPUT 1
#define RUN_WITH_HARDWARE_ATTACHED 1  // if actuators and sensors are attached via USB set to "1" 


class KinectNavGoalListener
{
  public: 
    KinectNavGoalListener();
    void detectionCallBack(const geometry_msgs::Twist::ConstPtr& msg);
    void runActionClient(void);
    
    // these are not needed, were used for testing, debugging:
    void runConcurrencyTest(void);  // test, deprecated
    void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void poseStampedCallBack(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);


  private:
    ros::NodeHandle nh;
    ros::Subscriber sub_target, sub_initialpose, sub_amcl_pose, sub_pose_stamped;
    ros::Publisher  pub_move_base_simple;
    geometry_msgs::Twist saved_msg;
    volatile double saved_lin_x, saved_lin_y, saved_ang_z; 
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> *ac;  
    move_base_msgs::MoveBaseGoal goal;
    geometry_msgs::Quaternion q_msg_in, q_msg_out;  // temp vars
    tf::Quaternion cur_laser_pose_q, delta_z_q, target_pose_q;
    geometry_msgs::PoseWithCovarianceStamped robot_pose_estimate;
    geometry_msgs::PoseStamped msg_to_publish;
    bool initialPosePublished;
		geometry_msgs::PointStamped laser_point, map_point;
    tf::TransformListener tfListener, tfListenerOdom;
    tf::StampedTransform tfLaserToMap, tfBaseFootprintToMap, tfOdomToMap;
    double lazerPositionX, lazerPositionY;
    tf::Stamped<tf::Pose> person_global_pose_from_laser; // in “global”, “map"
    double roll1, pitch1, yaw1, roll2, pitch2, yaw2;
    tf::Stamped<tf::Quaternion> base_target_transformed_q;
    int debugCounter, goalCounter;
};



// Not needed, deprecated, was used for testing, debugging:
// This function is called when the initialpose topic is published 
// (e.g. by RVIZ user input):
void KinectNavGoalListener::initialPoseCallback(
  const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
  initialPosePublished = true;
}


// Not needed, deprecated, was used for testing, debugging:
// This function is called when a pose estimation is published by AMCL node:
void KinectNavGoalListener::amclPoseCallback( 
  const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
  // this works when viewed by 'rostopic echo /amcl_pose':
  robot_pose_estimate = geometry_msgs::PoseWithCovarianceStamped(*msg);
}



KinectNavGoalListener::KinectNavGoalListener()
{
    debugCounter = 1;
    goalCounter = 1;
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
    runActionClient(); 
}


void KinectNavGoalListener::runActionClient(void){
  while( ros::ok()) { 
		ros::spinOnce();
		if (RUN_WITH_HARDWARE_ATTACHED ) {
		  while(!ac->waitForServer(ros::Duration(3.0) ) && nh.ok() ){
		    ROS_INFO("Waiting for the move_base action server to come up ...");
		  }
		}
    
    ROS_INFO("Compiled: %s,  %s", __DATE__,  __TIME__);
    // ============ DEBUGGING 1, angular only: ============
    // saved_msg.linear.x = 0.5;
    // saved_msg.linear.y = 0.1;
    // saved_msg.angular.z = 0.5;
    // Result: works. invoked many times, still works.
    // =========== end DEBUGGING 1. ====================


    // ============ DEBUGGING 2 : ==================
    // saved_msg.linear.x = 1.0;
    // saved_msg.linear.y = 0.0;
    // saved_msg.angular.z = 0.0;
    // Result: works, moves base forward, every time.
    // =========== end DEBUGGING 2.==================

    // ========== DEBUGGING 3: ========================
    /*    
    saved_msg.linear.x = 0.0;
    saved_msg.linear.y = 0.0;
    if (debugCounter % 2 == 0 )
      saved_msg.angular.z = 0.7;   // test positive and negative
    else 
      saved_msg.angular.z = - 0.7;
    debugCounter++;
    */

    //  "fixed frame" in jeeves' RVIZ:
    // base_footprint, laser, map, odom
    

    try { 
        tfListener.waitForTransform("map", "laser", ros::Time(0), ros::Duration(4.0));
        tfListener.lookupTransform("map", "laser", ros::Time(0), tfLaserToMap);
        cur_laser_pose_q = tfLaserToMap.getRotation();   // tf::Quaternion type
        lazerPositionX = tfLaserToMap.getOrigin().x();  // robot (lazer) position
        lazerPositionY = tfLaserToMap.getOrigin().y();
        double laser_roll, laser_pitch, laser_yaw;
        tf::Matrix3x3 m_laser_pose(cur_laser_pose_q);
        m_laser_pose.getRPY(laser_roll, laser_pitch, laser_yaw);
     
      double result_yaw = laser_yaw + saved_msg.angular.z;
      tf::Quaternion target_q = tf::createQuaternionFromYaw(result_yaw);
      goal.target_pose.pose.orientation.x = target_q.x();
      goal.target_pose.pose.orientation.y = target_q.y();
      goal.target_pose.pose.orientation.z = target_q.z();
      goal.target_pose.pose.orientation.w = target_q.w();

           
      tf::Quaternion delta_q = tf::createQuaternionFromYaw(saved_msg.angular.z);
		  tf::Stamped<tf::Pose> person_local_pose( tf::Pose(delta_q, 
			  tf::Vector3(saved_msg.linear.x, saved_msg.linear.y, 0.0)), 
          ros::Time(0), "laser" );
		  tfListener.transformPose("map", person_local_pose, person_global_pose_from_laser);
      goal.target_pose.pose.position.x = person_global_pose_from_laser.getOrigin().x();
		  goal.target_pose.pose.position.y = person_global_pose_from_laser.getOrigin().y();

      // for curiosity, how dummy_q(0, 0, 0, 1) was transformed from "laser" to "map":
      tf::Quaternion delta_transformed_q = person_global_pose_from_laser.getRotation();
      double delta_trans_q_x = delta_transformed_q.x();
      double delta_trans_q_y = delta_transformed_q.y();
      double delta_trans_q_z = delta_transformed_q.z();
      double delta_trans_q_w = delta_transformed_q.w();
      double delta_trans_roll, delta_trans_pitch, delta_trans_yaw;
      tf::Matrix3x3 delta_m(delta_transformed_q);
      delta_m.getRPY(delta_trans_roll, delta_trans_pitch, delta_trans_yaw);
      ROS_INFO("=============================================================");
      ROS_INFO("delta_q, from saved_msg.angular.z, from laser to map: ");
      ROS_INFO("delta_q laser to map: x: %f, y: %f, z: %f, w: %f",
          delta_trans_q_x, delta_trans_q_y, delta_trans_q_z, delta_trans_q_w);
      ROS_INFO("delta_q laser to map: r: %f, p: %f, y: %f",
          delta_trans_roll, delta_trans_pitch, delta_trans_yaw);
      ROS_INFO("=============================================================");
		} 
		catch (tf::TransformException &ex) {
			ROS_ERROR (	"%s", ex.what() );
		}


    // Position(27.180, 13.550, 0.000), Orientation(0.000, 0.000, 0.882, 0.472) = Angle: 2.159
    // test: manually hard-code orientation (0.000, 0.000, 0.802, 0.597),
    // for situation when lab-door is to the right. Taken (copied) from RVIZ:
    /*
    goal.target_pose.pose.orientation.x = 0.0;
    goal.target_pose.pose.orientation.y = 0.0;
    goal.target_pose.pose.orientation.z = 0.802;
    goal.target_pose.pose.orientation.w = 0.597;
    works but robot will spin forever after reaching the goal.
    Robot will fail if (x,y) are zeros, just rotation is requested.
    */

 
		// ====== display coords of attempted move: ============================ 
		ROS_INFO("Cur laser point:      x: %f, y: %f", lazerPositionX, lazerPositionY);
    double cur_q_x = cur_laser_pose_q.x();
    double cur_q_y = cur_laser_pose_q.y();
    double cur_q_z = cur_laser_pose_q.z();
    double cur_q_w = cur_laser_pose_q.w();
    double cur_roll, cur_pitch, cur_yaw, target_roll, target_pitch, target_yaw;
    tf::Matrix3x3 m_cur (cur_laser_pose_q);
    m_cur.getRPY(cur_roll, cur_pitch, cur_yaw);

    ROS_INFO("Cur laser orient:     x: %f, y: %f, z: %f, w: %f", 
      cur_q_x, cur_q_y, cur_q_z, cur_q_w); 
    ROS_INFO("Cur lazer RPY:        r: %f, p: %f, y: %f", cur_roll, cur_pitch, cur_yaw);
    ROS_INFO("============================================");

    ROS_INFO("                      GOAL No. %d ", goalCounter);
    goalCounter++;
    ROS_INFO("Target (local frame): x: %f, y: %f,  angle: %f", 
        saved_msg.linear.x, saved_msg.linear.y, saved_msg.angular.z);
    ROS_INFO("Target Point Global:  x: %f, y: %f",
       goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
    ROS_INFO("Target Orient Global: x: %f, y: %f, z: %f, w: %f", 
        goal.target_pose.pose.orientation.x, goal.target_pose.pose.orientation.y,
        goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w);

    // ROS_INFO("Issued ros::shutdown(), node stopped.");
    // ros::shutdown();  // exit node




    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    ac->sendGoal(goal);      
    float waitForResultTime = 30.0; 
    ROS_INFO("Waiting for Result for %f", waitForResultTime);
    bool finished_before_timeout = ac->waitForResult(ros::Duration(waitForResultTime));
    actionlib::SimpleClientGoalState state = ac->getState();
    if (finished_before_timeout) { 
      ROS_INFO("\t\tFINISHED before timeout of %f: %s\n\n", waitForResultTime, state.toString().c_str());        
    }
    else 
      ROS_INFO("\t\tFAILED: %s\n\n", state.toString().c_str());
 
    // ============== DEBUGGING: ================= 
    // ROS_INFO("Sleeping for 10 seconds ...");
		// ros::Duration(10).sleep();
    // ============= end DEBUGGING. =============


  } // while (ros::ok());
}

// Not needed, deprecated, was used for testing, debugging:
void KinectNavGoalListener::poseStampedCallBack(  // not needed
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  // robot_pose_estimate = geometry_msgs::PoseWithCovarianceStamped(*msg);
}


void KinectNavGoalListener::detectionCallBack(const geometry_msgs::Twist::ConstPtr& msg)
{  
  saved_ang_z = saved_msg.angular.z = msg->angular.z;
  saved_lin_x = saved_msg.linear.x = msg->linear.x;
  saved_lin_y = saved_msg.linear.y = msg->linear.y;
  ROS_INFO("DETECTIONS CALLBACK: x: %f, y: %f, z: %f\n", 
            saved_lin_x, saved_lin_y, saved_ang_z);    
}



int main(int argc, char** argv)
{  
    ros::init(argc, argv, "kinect_nav_goal_listener");
    KinectNavGoalListener kngl; 
    ROS_INFO("spinning node \"kinect_nav_goal_listener\"");
    return 0;
}






