// detector_markers_array_listener.cpp
// subsribes to /detector/markers_array topic
// prints person's position coorditates x, y, z


#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <math.h>
// #include "tf/transform_listener.h"
// #include <sstream>
#include "geometry_msgs/Pose.h"
// #include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h" 

#define PRINT_OUTPUT 1


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
    pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    sub = nh.subscribe<visualization_msgs::MarkerArray>("/tracker/markers_array", 1000,
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
    geometry_msgs::Twist cmd_vel_msg;

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
            
            // As code shows, there can be several people in the Field Of View.
            // Which one to target? We could pick closest to the robot.
            // wiki.ros.org/geometry/CoordinateFrameConventions:
            // Distances are in meters, angles are in radians: X: forward, Y: left, Z: up
            // Thus, we pick a closest person: min(x,y), for each element of array
            
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
        cmd_vel_msg.angular.z = 0.0;
    else
        cmd_vel_msg.angular.z = atan(minYglobalVal/minXglobalVal);   // yaw
    if (PRINT_OUTPUT)
        ROS_INFO("angular.z: %f", cmd_vel_msg.angular.z);

    // cmd_vel_msg.angular.z = tf::getYaw(msg->markers[minArrayIndex].pose.orientation);
    pub.publish(cmd_vel_msg);
}
        

int main(int argc, char** argv)
{
    ros::init(argc, argv, "PeopleTracker");
    DetectorMarkersArrayListener dmal;
    if (PRINT_OUTPUT)
        ROS_INFO("Spinning node");
    ros::spin();
    return 0;
}



