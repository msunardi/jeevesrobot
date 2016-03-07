#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "opencv2/core/core.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <unistd.h>
#include <math.h>

using namespace std;
using namespace cv;

#define MAX_FRAMES 15 // Total number of recent frames to consider when publishing a location
#define XSIZE 320.0     // HALF Width of the x-axis in pixels
#define YSIZE 240.0     // HALF Height of the y-axis in pixels
#define FACE_SIZE 100   // Size of training faces

/** Function Headers */
void read_csv(const string& filename, vector<Mat>& images, vector<int>& labels, char separator );
void ImageReceivedCallback( const sensor_msgs::ImageConstPtr& msg );
void ImageReceivedCallback2( Mat frame );

/** Global variables */
// Executable located in roboticsclub-mcecsbot/src/ros/devel/lib/facial_recognition so 
// path to cascade file must take this into account.
String cascade_filename = "src/facial_recognition/cascades/haarcascade_frontalface_alt.xml";
CascadeClassifier my_cascade;
//Ptr<FaceRecognizer> model createLBPHFaceRecognizer(1,8,8,8, 123.0); (int radius, neighbors, grid_x, grid_y, double threshold)
Ptr<FaceRecognizer> model = createLBPHFaceRecognizer();
ros::Publisher pub_yaw;
ros::Publisher pub_pitch;

int main(int argc, char **argv)
{
  // Initialize node
  ros::init(argc, argv, "facial_recognition");
  ros::NodeHandle n;

  CvCapture* capture;   // Structure for capturing video frames
  Mat frame;            // Single frame from video feed


  /****************************************************************************************
  //Code for reading training images and creating a new model. To be moved to a separate application.

  vector<Mat> images;   // Where to store training images
  vector<int> labels;   // Labels for training images

  // Get training images and train FaceRecognizer
  read_csv("src/facial_recognition/people/people.csv", images, labels, ';');

  //if (images[0].data == NULL) cout << "FAILURE TO READ FIRST IMAGE"<<endl;

  model->train(images, labels);

  // Save and/or update the model
  //model->update(newimages, newlabels);
  model->save("src/facial_recognition/people/model.yaml");

  *****************************************************************************************/

  model->load("src/facial_recognition/people/model.yaml");
  model->set("threshold", 125.0);

  // Current working directory, for debugging
  //char cwd[100];
  //getcwd(cwd, sizeof(cwd));
  //cout<< "Present working directory is: " << cwd << endl;

  // Load cascade
  if( !my_cascade.load( cascade_filename ) ){ printf("--(!)Error loading cascade\n"); return -1; };

  // Publisher
  //ros::Publisher pub = n.advertise("/head/cmd_pose", 100);
  pub_yaw   = n.advertise<std_msgs::Float32>("/head/cmd_pose_yaw", 100);
  pub_pitch = n.advertise<std_msgs::Float32>("/head/cmd_pose_pitch", 100);


  /*// Subscribe to third party usb_cam_node
  image_transport::ImageTransport it(n);
  image_transport::Subscriber subscribe = it.subscribe("/usb_cam/image_raw", 10, ImageReceivedCallback);
  ros::spin();
*/
  // Continually read images from webcam
  // THIS LOOP TAKES THE PLACE OF spin() AND WAITING FOR A MESSAGE FROM FREENECT
  // Use one or the other
  /*
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
  
*/
  // Create window for displaying images
  //cv::namedWindow("view");
  //cv::startWindowThread();

  // Subscriber
  image_transport::ImageTransport itrans(n);
  image_transport::Subscriber sub = itrans.subscribe("/camera/rgb/image_color", 10, ImageReceivedCallback);

  ros::spin();
  return 0;
}



// This function borrowed from docs.opencv.org tutorials by Philipp Wagner (aka Bytefish)
void read_csv(const string& filename, vector<Mat>& images, vector<int>& labels, char separator = ';') {
    std::ifstream file(filename.c_str(), ifstream::in);
    if (!file) {
        cout<< "No input file was given!!"<<endl;
        string error_message = "No valid input file was given, please check the given filename.";
        CV_Error(CV_StsBadArg, error_message);
    }
    string line, path, classlabel;
    while (getline(file, line)) {
        stringstream liness(line);
        getline(liness, path, separator);
        getline(liness, classlabel);
        if(!path.empty() && !classlabel.empty()) {
            images.push_back(imread(path, 0));
            labels.push_back(atoi(classlabel.c_str()));
        }
    }
}



// Image received from freenect topic gets converted to a Mat
// and then passed to the main callback function
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
  // Send the converted cv::MAT image to the the main callback
  ImageReceivedCallback2(frame_ptr->image);
  imshow( "Capture - Face Detection", frame_ptr->image );
}



// Primary callback function when an image is published
void ImageReceivedCallback2(Mat frame)
{
  static int face_counter = 0;                    // 
  static Point recent_locations[MAX_FRAMES];  // Array of points corresponding to found faces
  static int count = 0;                       // Current index of recent_locations
  std::vector<Rect> faces;                    // Dynamic array of rectangles
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
  cout<<"I SEE "<<faces.size()<<" FACES ";
  recent_locations[count].x = 0;
  recent_locations[count].y = 0;

  // Go through the list of faces in this image and store the best one in recent_locations[i]
  for( size_t i = 0; i < faces.size(); i++ )
  {
    // Extract Rect for facial recognition
    Rect faces_i = faces[i];
    Mat person = grayframe(faces_i);
    Mat person_resized;
    resize(person, person_resized, Size(100, 100), 1.0, 1.0, INTER_CUBIC);

    // Code for saving training images
    // Use this to save faces detected from the cascade classifier, and then update LBPH model
    /*
    face_counter++;
    std::string file_name;
    file_name = "src/facial_recognition/people/unallocated/";
    ostringstream convert;
    convert << face_counter;
    file_name = file_name + convert.str();
    file_name = file_name + ".jpg";
    imwrite(file_name, person_resized);
    */

    // Predict!
    int prediction = model->predict(person_resized);
    cout<< "PREDICTION: " << prediction << endl; 


    // Find the point at the center of the Rect and the face size
    Point center( faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5 );
    int size = (faces[i].width + faces[i].height)/2;

    // Replace the entry in recent_locations if current size above minimum and is bigger
    if(size > 50 && size > biggest)
    {
      biggest = size;
      recent_locations[count] = center;
    }
    cout << "(" << center.x << ", " << center.y << ") with size " << size << endl;
    // Draw ellipse on frame
    // void ellipse(Mat& img, Point center, Size axes, double angle, double startAngle, double endAngle, const Scalar& color, int thickness=1, int lineType=8, int shift=0)
    ellipse( frame, center, Size( faces[i].width*0.5, faces[i].height*0.5), 0, 0, 360, Scalar( 0, 255, 0 ), 2, 8, 0 );
    
    // Draw text with guess
    string person_name;
    switch(prediction){
      case -1: person_name = "Stranger";
              break;
      case  0: person_name = "Chris";
              cout << "HELLO CHRIS" << endl;
              break;
      case  1: person_name = "Perkowski";
              cout << "HELLO PERKOWSKI" << endl;
              break;
    }
    putText(frame, person_name, Point(faces[i].x, faces[i].y), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 2.0);
  }

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

  // Show the edited frame in the window
  imshow( "Capture - Face Detection", frame );

  // Cycle through circular array of most recent points
  count++;
  if(count >= MAX_FRAMES) count = 0; 
}
