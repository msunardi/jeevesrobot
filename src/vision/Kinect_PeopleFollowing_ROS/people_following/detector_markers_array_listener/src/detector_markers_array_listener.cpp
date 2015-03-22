// detector_markers_array_listener.cpp
// subsribes to /detector/markers_array topic
// prints person's position coorditates x, y, z
// 2014.12.22, 2015.03.16


#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <math.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "visualization_msgs/MarkerArray.h" 

#define PRINT_OUTPUT 0


class DetectorMarkersArrayListener
{
    public: 
        DetectorMarkersArrayListener();
        void callBack(const visualization_msgs::MarkerArray::ConstPtr& msg);

    private:
        ros::NodeHandle nh;
        ros::Subscriber sub;
        ros::Publisher pub;
};


DetectorMarkersArrayListener::DetectorMarkersArrayListener()
{
    pub = nh.advertise<geometry_msgs::Twist>("/detected_person", 1);
    // pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    sub = nh.subscribe<visualization_msgs::MarkerArray>("/tracker/markers_array", 1,
            &DetectorMarkersArrayListener::callBack, this);
}


void DetectorMarkersArrayListener::callBack(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
    int detections_count; 
    int index;
    double x, y, z;
    int minArrayIndex = 0;
    double minGlobalVal = 999.999;   // arbitrary large distance
    double minLocalVal =  999.999;
    double minXglobalVal, minYglobalVal;     // coordinates of person closest to the robot
    geometry_msgs::Twist msg_out;
    geometry_msgs::PoseStamped pose_stamped_msg;

    // the following variables are for second part, goal setting and person following:
    double x_out, y_out;            // goal coordinates
    const double dist_min_const = 1.5;    // in meters, distance between robot and a person
    const double delta_const = 1.0;       // in meters, range for which robot will not start approaching again. 
    double dist_min_coeff = 0.0;          // intermediate coefficient
    double distance;                      // current actual distance between person and robot


    detections_count = msg->markers.size();
    if (detections_count > 0) {
        for( index = 0; index < detections_count; index++)
        {
            // do not dereference position coords inside of ROS_INFO macro,
            // else it will seg fault, do NOT do:
            // ROS_INFO("x: %lf", msg->markers[0].pose.position.x);
            x = msg->markers[index].pose.position.x;
            y = msg->markers[index].pose.position.y;
            z = msg->markers[index].pose.position.z;
            
            // There can be several people in the Field Of View.
            // We could pick closest to the robot.
            // wiki.ros.org/geometry/CoordinateFrameConventions:
            // Distances are in meters, angles are in radians: X: forward, Y: left, Z: up
            // Pick a closest person: min(x,y), for each element of array:      
            if (x < y) minLocalVal = x;
            else minLocalVal = y;
            if ( index == 0) {
                minGlobalVal = minLocalVal;
            }
            else {
                if (minLocalVal < minGlobalVal)  {
                    minGlobalVal = minLocalVal;
                    minArrayIndex = index;
                }
            }
            if (PRINT_OUTPUT)
                ROS_INFO("seq %i:  x: % .12f   y: % .12f  z: % .12f", index, x, y, z);
        }
        minXglobalVal = msg->markers[minArrayIndex].pose.position.x;
        minYglobalVal = msg->markers[minArrayIndex].pose.position.y;
        if(PRINT_OUTPUT)
            ROS_INFO("closest seq: %i  x: % .12f  y: % .12f\n", 
                        minArrayIndex, minXglobalVal, minYglobalVal);
    }
    
    if (minXglobalVal == 0.0 || minYglobalVal == 0.0)
        msg_out.angular.z = 0.0;
    else
        msg_out.angular.z = atan(minYglobalVal/minXglobalVal);   // "yaw"
    if (PRINT_OUTPUT)
        ROS_INFO("angular.z: %f", msg_out.angular.z);

    distance = sqrt( powl(minXglobalVal, 2) + powl(minYglobalVal, 2) );
    // Check if robot is already within the dist_min:
    if ((minXglobalVal == 0.0 && minYglobalVal == 0.0) || ( distance <= dist_min_const )) 
    {
        msg_out.linear.x = 0.0;
        msg_out.linear.y = 0.0;
    }
    else {
        dist_min_coeff = 1 - dist_min_const/distance;
        msg_out.linear.x = minXglobalVal * dist_min_coeff;
        msg_out.linear.y = minYglobalVal * dist_min_coeff;
    }
    ROS_INFO("local x: %f, local y: %f, real dist: %f", minXglobalVal, minYglobalVal, distance);
    ROS_INFO("published: x: %f, y: %f, z: %f; approach dist: %f \n\n",
            msg_out.linear.x, msg_out.linear.y, msg_out.angular.z, dist_min_const);
    pub.publish(msg_out);  
}
        

int main(int argc, char** argv)
{
    ros::init(argc, argv, "DetectorMarkersArrayListener");
    DetectorMarkersArrayListener dmal;
    ROS_INFO("Spinning node detector_markers_array_listener");
    ros::spin();
    return 0;
}



