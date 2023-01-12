#include <ros/ros.h>
#include <aruco_detect/aruco_info.h>
#include <aruco_detect/aruco_list.h>
#include <opencv2/imgproc.hpp>


#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <cmath>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <iostream>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>


using namespace std;
using namespace cv;
using namespace Eigen;

void sort(vector<int> & ids, vector<Vec3d> & rvecs, vector<Vec3d> & tvecs);

double distance(vector<Vec3d> & tvec, int i);

template <class T>
void swap_info(T & a, T & b);

int main(int argc, char **argv)
{

  ros::init(argc, argv, "aruco");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<aruco_detect::aruco_list>("aruco_list", 100);
  ros::Rate loop_rate(3);

  VideoCapture video(0);
  video.set(CAP_PROP_FRAME_WIDTH, 1280);
  video.set(CAP_PROP_FRAME_HEIGHT, 720);
  video.set(CAP_PROP_BUFFERSIZE, 1);

  Mat cameraMatrix = (Mat_<double>(3, 3) << 768.96, 0, 640, 0, 771.38, 360, 0, 0, 1.0); 
  Mat distCoeffs = (Mat_<double>(1, 5) << 0.0, 0.0, 0.0, 0.0, 0.0);
  Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_100);

 
  while (ros::ok())
  {
    static Mat image;
    // video >> image;
    video.read(image);
    video.read(image);
    video.read(image);
    video.read(image);
    cv::flip(image, image, 0);
    cv::flip(image, image, 1);

    vector<int> ids; 
    vector<vector<Point2f> > corners; 

    aruco::detectMarkers(image, dictionary, corners, ids); 

    if (ids.size() > 0) // if at least one marker detected
    { 
      ROS_INFO("Arucode Detected!");
      aruco::drawDetectedMarkers(image, corners, ids);
      vector<Vec3d> rvecs, tvecs; 
      aruco::estimatePoseSingleMarkers(corners, 50, cameraMatrix, distCoeffs, rvecs, tvecs);

      sort(ids, rvecs, tvecs);

      aruco_detect::aruco_list msg_list;

      
      for (int i = 0; i < ids.size(); i++)
        {
          if (ids[i] == 47 || ids[i] == 13 || ids[i] == 36 || ids[i] == 17)

          {
            aruco_detect::aruco_info msg;
            msg.id = ids[i];

            tf::tfMessage tf_msg;

            geometry_msgs::TransformStamped geo_msg;
          
            geo_msg.transform.translation.x = tvecs[i][0]/1000;
            geo_msg.transform.translation.y = tvecs[i][1]/1000;
            geo_msg.transform.translation.z = tvecs[i][2]/1000;

            Mat R;
            Rodrigues(rvecs[i], R);
            Matrix<double,3,3> M;
            cv::cv2eigen(R, M);
            Quaterniond q = Quaterniond(M);

            geo_msg.transform.rotation.x = q.coeffs()[0];
            geo_msg.transform.rotation.y = q.coeffs()[1];
            geo_msg.transform.rotation.z = q.coeffs()[2];
            geo_msg.transform.rotation.w = q.coeffs()[3];
            
            msg.tf_msg.transforms.push_back(geo_msg);

            msg_list.aruco_list.push_back(msg);

          }
        }

        chatter_pub.publish(msg_list);

      

      // draw axis for each marker 
      for(int i=0; i<ids.size(); i++) 
        aruco::drawAxis(image, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 30);  
    }
    else
    {
      aruco_detect::aruco_list msg_list;
      chatter_pub.publish(msg_list);
      ROS_INFO("No Arucode");
    }

    waitKey(1000 / video.get(CAP_PROP_FPS));
//  imshow("aruco", image);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

void sort(vector<int> & ids, vector<Vec3d> & rvecs, vector<Vec3d> & tvecs)
{
  for(int i= 0; i < ids.size(); i++)
  {
    for(int j = 0; j < ids.size() - i - 1; j++)
    {
      if(distance(tvecs, j) > distance(tvecs, j + 1))
      {
        swap_info(ids[j], ids[j + 1]);
        swap_info(rvecs[j], rvecs[j + 1]);
        swap_info(tvecs[j], tvecs[j + 1]);
      }
    }
  }
}

double distance(vector<Vec3d> & tvecs, int i)
{
  return sqrt(pow(tvecs[i][0], 2) + pow(tvecs[i][1], 2) + pow(tvecs[i][2], 2));
}

template <class T>
void swap_info(T & a, T & b)
{
   T temp;
   temp = a;
   a = b;
   b = temp;
 }

