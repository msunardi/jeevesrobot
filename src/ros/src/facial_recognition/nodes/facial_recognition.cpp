#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <math.h>

using namespace std;
using namespace cv;

#define MAX_FRAMES 15 // Total number of recent frames to consider when publishing a location
#define XSIZE 320.0     // HALF Width of the x-axis in pixels
#define YSIZE 240.0     // HALF Height of the y-axis in pixels

/** Function Headers */
void ImageReceivedCallback( const sensor_msgs::ImageConstPtr& msg );
void ImageReceivedCallback2( Mat frame );

/** Global variables */
// Executable located in roboticsclub-mcecsbot/src/ros/devel/lib/facial_recognition so 
// path to cascade file must take this into account.
String cascade_filename = "src/facial_recognition/cascades/haarcascade_frontalface_alt.xml";
CascadeClassifier my_cascade;
ros::Publisher pub_yaw;
ros::Publisher pub_pitch;

int main(int argc, char **argv)
{
  // Initialize node
  ros::init(argc, argv, "facial_recognition");
  ros::NodeHandle n;

  // CV things
  CvCapture* capture;
  Mat frame;

  // Current working directory, for debugging purposes
  char cwd[100];
  getcwd(cwd, sizeof(cwd));
  cout<< "Present working directory is: " << cwd << endl;

  // Load cascade
  if( !my_cascade.load( cascade_filename ) ){ printf("--(!)Error loading cascade\n"); return -1; };

  // Publisher
  //ros::Publisher pub = n.advertise("/head/cmd_pose", 100);
  pub_yaw   = n.advertise<std_msgs::Float32>("/head/cmd_pose_yaw", 100);
  pub_pitch = n.advertise<std_msgs::Float32>("/head/cmd_pose_pitch", 100);


  // Continually read images from webcam
  // THIS LOOP TAKES THE PLACE OF spin() AND WAITNIG FOR A MESSAGE
  
  // **********************************************************************
  capture = cvCaptureFromCAM( -1 ); //cvCaptureFromCAM same as VideoCapture
    if( capture )
    {
      while( true )
      {
        frame = cvQueryFrame( capture ); // Same as VideoCapture.read()

        // Apply the classifier to the frame
        if( !frame.empty() )
        { ImageReceivedCallback2( frame ); }
        else
        { printf(" --(!) No captured frame -- Break!"); break; }

        int c = waitKey(10);
        if( (char)c == 'c' ) { break; }
      }
    }
  // *********************************************************************
  

  // Create window for displaying images
  //cv::namedWindow("view");
  //cv::startWindowThread();

  // Subscriber
  image_transport::ImageTransport it(n);
  image_transport::Subscriber sub = it.subscribe("/camera/rgb/image_color", 10, ImageReceivedCallback);

  ros::spin();
  return 0;
}




void ImageReceivedCallback(const sensor_msgs::ImageConstPtr& msg)
{
  // Create an OpenCV image type
  cv_bridge::CvImagePtr frame_ptr;

  // Convert the message to type cv::Mat
  try
  {
    frame_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  //frame = frame_ptr->image;

  // Send the converted cv::MAT image to the the main callback
  ImageReceivedCallback2(frame_ptr->image);
}


void ImageReceivedCallback2(Mat frame)
{
  static int count = 0;
  static Point recent_locations[MAX_FRAMES];
  std::vector<Rect> faces;  // Dynamic array of rectangles
  Point average_location( 0, 0);

  int biggest = 0;
  int valid_count = 0;

  Mat grayframe;

  // Convert frame to grayscale
  cvtColor( frame, grayframe, CV_BGR2GRAY );
  // Normalize brightness, increase contrast
  equalizeHist( grayframe, grayframe );

  // Check the frame for faces
  my_cascade.detectMultiScale( grayframe, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30) );
  cout<<"I SEE "<<faces.size()<<" FACES AT: ";
  recent_locations[count].x = 0;
  recent_locations[count].y = 0;

  // Go through the list of faces in this image and store the best one in recent_locations[i]
  for( size_t i = 0; i < faces.size(); i++ )
  {
    // Find the point at the center of the Rect and the face size
    Point center( faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5 );
    int size = (faces[i].width + faces[i].height)/2;

    // Replace the entry in recent_locations if current size above minimum and is bigger
    if(size > 50 && size > biggest)
    {
      biggest = size;
      recent_locations[count] = center;
    }
    cout << "(" << center.x << ", " << center.y << ") with size " << size;
    // Draw ellipse on frame
    // void ellipse(Mat& img, Point center, Size axes, double angle, double startAngle, double endAngle, 
    //              const Scalar& color, int thickness=1, int lineType=8, int shift=0)
    ellipse( frame, center, Size( faces[i].width*0.5, faces[i].height*0.5), 0, 0, 360, Scalar( 0, 255, 0 ), 2, 8, 0 );
  }
  
  cout << endl;

  // Count the number of valid frames among recent_locations
  for(int j = 0; j < MAX_FRAMES; j++)
  {
    // Check each point for a valid coordinate. Add up all the x's and y's.
    if(recent_locations[j].x || recent_locations[j].y)
    {
      valid_count++;
      average_location.x += recent_locations[j].x;
      average_location.y += recent_locations[j].y;
    }
  }

  // Calculate center of x and y
  if(valid_count)
  {
    average_location.x = (average_location.x)/valid_count;
    average_location.y = (average_location.y)/valid_count;
  }

  // Decide whether there are enough valid frames to go ahead with publishing a head location
  if(valid_count > (MAX_FRAMES/2))
  {
    // Convert to radians and publish
    cout<< "Qualified location at (" <<average_location.x<<", "<<average_location.y<<")";

    std_msgs::Float32 yaw;
    std_msgs::Float32 pitch;

    yaw.data = (float)((XSIZE - average_location.x) * 3.14/(XSIZE * 3)); // Limits yaw to +/-pi/3
    pitch.data = (float)((YSIZE - average_location.y) * 3.14/(YSIZE * 12)); // Limits yaw to +/-pi/12

    // Round off to prevent micro-movements
    yaw.data = roundf(yaw.data * 100) / 100;
    pitch.data = roundf(pitch.data * 100) / 100;

    cout<< " ---> (yaw, pitch) = "<< yaw.data << " " << pitch.data << endl;

    pub_yaw.publish(yaw);
    pub_pitch.publish(pitch);
  }
  else cout<<"recent_locations doesn't qualify to send a head message"<<endl;

  cout << endl;
  // Show this frame in the window
  imshow( "Capture - Face Detection", frame );

  count++;
  if(count >= MAX_FRAMES) count = 0; // Cycle through array of recent frames
}
