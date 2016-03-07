#include <ros/ros.h>

#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>

#include <iostream>
#include <stdio.h>
#include <time.h>
#include <vector>
#include <cmath>
#include "kalman.h"

#include <Eigen/Dense>

#include "calculation.h"




using namespace Eigen;
using namespace std;
using namespace cv;


#define PI 3.14159265
// #define DEBUG

double Angle = 61; // 55
double Angle2 = 354; // 354
double Angle3 = -2;

struct MyPoint {
   double x;
   double y;
};

struct My3DPoint {
   double x;
   double y;
   double z;
};

struct My3DVelocity {
   double x;
   double y;
   double z;
};

vector <MyPoint> MyFirst, MySecond;
#ifdef DEBUG
static const std::string OPENCV_WINDOW = "Image window";
static const std::string OPENCV_WINDOW2 = "Image window2";
#endif
int Ccount1 = -2;
int Ccount = -2;
clock_t Start_time, End_time;
clock_t Frame_S_time, Frame_E_time;

double Current_roll;
double Current_yaw;
double Current_pitch;

int Dron_size = 0;
My3DPoint Real_Dron_Loc[4];
My3DVelocity Velocity[8] = {{0, 0, 0},};

My3DPoint Current_Loc[8];
My3DPoint Track_Loc[8];

double Time;
double Acummulate_Time[8] = {0.0,};

My3DPoint Direction_Vector[4];

double origin_x, origin_y, origin_z;
double origin_x2, origin_y2, origin_z2;

typedef struct lpf_t {
   float input;
   double cur_time;

   float last_input;
   double last_time;
   double cycle_time;
   float lpf_filter;
   float lpf_hz;
} lpf_t;

static float get_lpf(lpf_t *lpf, float lpf_hz = 15.0f) {
   if (!lpf->last_time || !lpf_hz) {
      lpf->last_time = lpf->cur_time;
      return 0;
   }
   if (!lpf->lpf_filter || lpf_hz != lpf->lpf_hz) {
      lpf->lpf_filter = (1.0f / (2.0f * M_PI * lpf_hz));
      lpf->lpf_hz = lpf_hz;
   }

   lpf->cycle_time = lpf->cur_time - lpf->last_time;
   lpf->last_time = lpf->cur_time;

   lpf->input = lpf->last_input + (lpf->cycle_time / (lpf->lpf_filter + lpf->cycle_time)) * (lpf->input - lpf->last_input);
   lpf->last_input = lpf->input;
   return lpf->input;
}




int MyCompare(MyPoint a, MyPoint b) {
   return (a.x < b.x || (a.x == b.x && a.y < b.y));
}


class RECONSTRUCTION
{
   ros::NodeHandle nh_;
   image_transport::ImageTransport it_;
   image_transport::Subscriber image_sub_;
   image_transport::Subscriber image_sub_2;
   image_geometry::PinholeCameraModel cam_model_left, cam_model_right;
   image_transport::CameraSubscriber left_subscribe;
   image_transport::CameraSubscriber right_subscribe;



   ros::Publisher pub_drone[4];
   ros::Publisher Nasang[4];

public:
   PV_kalman kalman_x;
   PV_kalman kalman_y;
   lpf_c lpf_x;
   PV3_kalman kalman_xyz;

   RECONSTRUCTION()
      : it_(nh_)
   {
      left_subscribe = it_.subscribeCamera("/stereo/left/image_raw", 2, &RECONSTRUCTION::imageCb, this);
      right_subscribe = it_.subscribeCamera("/stereo/right/image_raw", 2, &RECONSTRUCTION::imageCb2, this);

      pub_drone[0] = nh_.advertise<geometry_msgs::Point>("/KALMAN", 1);
      pub_drone[1] = nh_.advertise<geometry_msgs::Point>("/SECOND/CURRENT_POS", 1);
      pub_drone[2] = nh_.advertise<geometry_msgs::Point>("/THIRD/CURRENT_POS", 1);
      pub_drone[3] = nh_.advertise<geometry_msgs::Point>("/FOURTH/CURRENT_POS", 1);

      Nasang[0] = nh_.advertise<geometry_msgs::Point>("/FIRST/NASANG", 1);
      Nasang[1] = nh_.advertise<geometry_msgs::Point>("/SECOND/NASANG", 1);
      Nasang[2] = nh_.advertise<geometry_msgs::Point>("/THIRD/NASANG", 1);
      Nasang[3] = nh_.advertise<geometry_msgs::Point>("/FOURTH/NASANG", 1);

#ifdef DEBUG
      cv::namedWindow(OPENCV_WINDOW, 0);
      cv::namedWindow(OPENCV_WINDOW2, 0);
#endif
   }

   ~RECONSTRUCTION()
   {
#ifdef DEBUG
      cv::destroyWindow(OPENCV_WINDOW);
      cv::destroyWindow(OPENCV_WINDOW2);
#endif
   }

   void rectify_image(Mat* src, Mat* dest) {


   }

   void imageCb(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& info_msg) {
      MyFirst.clear();
      Ccount1++;
      cv_bridge::CvImageConstPtr cv_ptr;
      double cur = ros::Time::now().toSec();
      try
      {
         cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
      }
      catch (cv_bridge::Exception& e)
      {
         ROS_ERROR("cv_bridge exception: %s", e.what());
         return;
      }

      cam_model_left.fromCameraInfo(info_msg);
      // cout << "dist " << info_msg->D[0] << " " << info_msg->D[1] << " " << info_msg->D[2] << " " <<  endl;

      Mat image, gray, temp, mask;
      image = cv_ptr->image;

      mask = image.clone();

      int niters = 1;
      dilate(mask, temp, Mat(), Point(-1, -1), niters);
      erode(temp, temp, Mat(), Point(-1, -1), niters * 2);
      dilate(temp, temp, Mat(), Point(-1, -1), niters);

      threshold(temp, temp, 200, 255, CV_THRESH_BINARY);

      for (int i = 0; i <= 5; i++) {
         for (int j = 1000; j < temp.cols; j++) {
            temp.at<uchar>(i, j) = 0;
         }
      }

      vector<vector<Point> > contours1, contours2;
      vector<Vec4i> hierarchy;
      findContours(temp, contours2, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);


      int cmin = 0;
      int cmax = 100;


      for (int i = 0; i < contours2.size(); i++) {
         if (contours2[i].size() < cmin || contours2[i].size() > cmax) {
            continue;
         }
         else
            contours1.push_back(contours2[i]);
      }

      image = Scalar(255, 255, 255);

      std::vector<std::vector<cv::Point> > ::iterator itc;
      itc = contours1.begin();

      MyPoint TTEMP;
      while (itc != contours1.end()) {

         cv::Moments mom = cv::moments(cv::Mat(*itc++));
#ifdef DEBUG
         cv::circle(image, cv::Point(mom.m10 / mom.m00, mom.m01 / mom.m00), 2, cv::Scalar(0), 2);
#endif
         TTEMP.x = mom.m10 / mom.m00;
         TTEMP.y = mom.m01 / mom.m00;

         MyFirst.push_back(TTEMP);
      }


#ifdef DEBUG
      cv::imshow(OPENCV_WINDOW, cv_ptr->image);
      //Rect rect(MyFirst[0].x - 50, MyFirst[0].y- 50, 100, 100);
      //Mat subimage = cv_ptr->image(rect);
      //cv::imshow(OPENCV_WINDOW, subimage);
      cv::waitKey(3);
#endif

      vector<My3DPoint> myvec;
      My3DPoint myTempVec;

#ifdef DEBUG
      //cout << Ccount << " " << MyFirst[0].y << " " << MySecond[0].y << endl;
#endif
      // cout << "size : " << MyFirst.size() << "," << MySecond.size() << endl;

      if (MyFirst.size() && MySecond.size()) {
         // cout << MyFirst[0].x << "," << MyFirst[0].y << endl;
         cv::Point2d left_pt(MyFirst[0].x, MyFirst[0].y);
         cv::Point2d right_pt(MySecond[0].x, MySecond[0].y);
         cam_model_left.rectifyPoint(left_pt);

         // cout << cam_model_left.rectifyPoint(left_pt).x << "," << cam_model_left.rectifyPoint(left_pt).y << "rectified" << endl;
         // cout << cam_model_right.rectifyPoint(right_pt).x << "," << cam_model_right.rectifyPoint(right_pt).y << "rectified" << endl;

         // cout << "notcal :: " << MyFirst[0].y - MySecond[0].y << endl;
         // cout << "calibr  :: " << cam_model_left.rectifyPoint(left_pt).y - cam_model_right.rectifyPoint(right_pt).y << endl;

         float leftpt_y = MyFirst[0].y;

         Matrix<float,2,1> X_kalman = kalman_x.getKalman(leftpt_y);
         // cout << "KALMAN" << kalman_x.getKalman_1(leftpt_y) << "," << leftpt_y << endl;;

         geometry_msgs::Point drone1_msg;
         drone1_msg.x = leftpt_y;//get_lpf(&lpf_x,10);
         drone1_msg.y = X_kalman(0,0);//get_lpf(&lpf_z,10);
         drone1_msg.z = X_kalman(1,0);//get_lpf(&lpf_y,5);
         pub_drone[0].publish(drone1_msg);



         cv::Point3d ptr_left = cam_model_left.projectPixelTo3dRay(left_pt);
         cv::Point3d ptr_right = cam_model_right.projectPixelTo3dRay(right_pt);


         // getKalman_1();

         // cout << ptr_left.y - ptr_right.y << endl;
         // cout << "left 3d   :: " <<  ptr_left.x  << "," << ptr_left.y << "," << ptr_left.z << endl;
         // cout << "right 3d  :: " <<  ptr_right.x  << "," << ptr_right.y << "," << ptr_right.z << endl << endl;



         ptr_left = cam_model_left.projectPixelTo3dRay(cam_model_left.rectifyPoint(left_pt));
         ptr_right = cam_model_right.projectPixelTo3dRay(cam_model_right.rectifyPoint(right_pt));

         // cout << ptr_left.y - ptr_right.y << endl;
         // cout << "left 3d   :: " <<  ptr_left.x  << "," << ptr_left.y << "," << ptr_left.z << endl;
         // cout << "right 3d  :: " <<  ptr_right.x  << "," << ptr_right.y << "," << ptr_right.z << endl;


      }
      // cout << MySecond[0].x << "," << MySecond[0].y << endl;
      if (Ccount == 1) {
         sort(MyFirst.begin(), MyFirst.end(), MyCompare);
         sort(MySecond.begin(), MySecond.end(), MyCompare);




         Dron_size = MyFirst.size();

         for (int i = 0; i < Dron_size; i++) {
            double a1, b1, c1;
            // cout << "hello " << MyFirst[i].x - MySecond[i].x << endl;
            c1 =  2.97 * 450 / ((MyFirst[i].x - MySecond[i].x) * 0.00375);
            a1 = (2.97 * 450 / ((MyFirst[i].x - MySecond[i].x) * 0.00375)) * (0.00375 * (MyFirst[i].x + MySecond[i].x - 1280)) / (2 * 2.97);

            b1 =  (-0.00375) * ((MyFirst[i].y + MySecond[i].y) / 2 - 480) * (2.97 * 450 / ((MyFirst[i].x - MySecond[i].x) * 0.00375)) / 2.97;


            Current_Loc[i].x = a1;
            Current_Loc[i].y = b1 * cos(Angle * PI / 180.0) - c1 * sin(Angle * PI / 180.0);
            Current_Loc[i].z = b1 * sin(Angle * PI / 180.0) + c1 * cos(Angle * PI / 180.0);
            a1 = Current_Loc[i].x;
            b1 = Current_Loc[i].y;
            c1 = Current_Loc[i].z;

            Current_Loc[i].x = a1 * cos(Angle2 * PI / 180.0) - b1 * sin(Angle2 * PI / 180.0);
            Current_Loc[i].y = a1 * sin(Angle2 * PI / 180.0) + b1 * cos(Angle2 * PI / 180.0);
            Current_Loc[i].z = c1;

            a1 = Current_Loc[i].x;
            b1 = Current_Loc[i].y;
            c1 = Current_Loc[i].z;

            Current_Loc[i].x = a1 * cos(Angle3 * PI / 180.0) + c1 * sin(Angle3 * PI / 180.0);
            Current_Loc[i].y = b1;
            Current_Loc[i].z = -a1 * sin(Angle3 * PI / 180.0) + c1 * cos(Angle3 * PI / 180.0);
         }

         for (int i = 0; i < Dron_size; i++) {
            Real_Dron_Loc[i].x = (Current_Loc[i].x);
            Real_Dron_Loc[i].y = (Current_Loc[i].y);
            Real_Dron_Loc[i].z = (Current_Loc[i].z);
         }

         Frame_S_time = clock();

         for (int i = 0; i < Dron_size; i++) {
            Velocity[i].x = 0;
            Velocity[i].y = 0;
            Velocity[i].z = 0;
         }
      }
      else {
         if (MyFirst.size() != Dron_size || MySecond.size() != Dron_size)
            return;
         Frame_E_time = clock();

         Time = (double)(Frame_E_time - Frame_S_time) / CLOCKS_PER_SEC;
         Frame_S_time = Frame_E_time;

         for (int i = 0; i < Dron_size; i++) {
            Acummulate_Time[i] += Time;
            Track_Loc[i].x = Current_Loc[i].x;// + Acummulate_Time[i] * Velocity[i].x;
            Track_Loc[i].y = Current_Loc[i].y;// + Acummulate_Time[i] * Velocity[i].y;
            Track_Loc[i].z = Current_Loc[i].z;// + Acummulate_Time[i] * Velocity[i].z;
         }

         for (int k = 0; k < Dron_size; k++) {
            double closest_dis = 987654321;
            My3DPoint closest_3DPoint;

            closest_3DPoint.x = 0;
            closest_3DPoint.y = 0;
            closest_3DPoint.z = 0;

            int flag = 0;
            int AA = 999, BB = 888;

            for (int i = 0; i < MyFirst.size(); i++) {
               for (int j = 0; j < MySecond.size(); j++) {
                  double a1, b1, c1;
                  double aa1, bb1, cc1;

                  cout << MyFirst[i].x - MySecond[j].x << endl;
                  c1 =  2.97 * 450.0 / ((MyFirst[i].x - MySecond[j].x) * 0.00375);
                  a1 = (2.97 * 450.0 / ((MyFirst[i].x - MySecond[j].x) * 0.00375)) * (0.00375 * (MyFirst[i].x + MySecond[j].x - 1280.0)) / (2.0 * 2.97);

                  b1 =  (-0.00375) * ((MyFirst[i].y + MySecond[j].y) / 2 - 480.0) * (2.97 * 450.0 / ((MyFirst[i].x - MySecond[j].x) * 0.00375)) / 2.97;

                  aa1 = a1;
                  bb1 = b1 * cos(Angle * PI / 180.0) - c1 * sin(Angle * PI / 180.0);
                  cc1 = b1 * sin(Angle * PI / 180.0) + c1 * cos(Angle * PI / 180.0);

                  a1 = aa1;
                  b1 = bb1;
                  c1 = cc1;

                  aa1 = a1 * cos(Angle2 * PI / 180.0) - b1 * sin(Angle2 * PI / 180.0);
                  bb1 = a1 * sin(Angle2 * PI / 180.0) + b1 * cos(Angle2 * PI / 180.0);
                  cc1 = c1;


                  a1 = aa1;
                  b1 = bb1;
                  c1 = cc1;

                  aa1 = a1 * cos(Angle3 * PI / 180.0) + c1 * sin(Angle3 * PI / 180.0);
                  bb1 = b1;
                  cc1 = -a1 * sin(Angle3 * PI / 180.0) + c1 * cos(Angle3 * PI / 180.0);

                  if ((sqrt(pow(Track_Loc[k].x - aa1, 2) + pow(Track_Loc[k].y - bb1, 2) + pow(Track_Loc[k].z - cc1, 2)) < closest_dis) && sqrt(pow(Track_Loc[k].x - aa1, 2) + pow(Track_Loc[k].y - bb1, 2) + pow(Track_Loc[k].z - cc1, 2)) < 300 ) {
                     closest_dis = sqrt(pow(Track_Loc[k].x - aa1, 2) + pow(Track_Loc[k].y - bb1, 2) + pow(Track_Loc[k].z - cc1, 2));

                     closest_3DPoint.x = aa1;
                     closest_3DPoint.y = bb1;
                     closest_3DPoint.z = cc1;
                     AA = i;
                     BB = j;
                  }

               }

            }
#ifdef DEBUG
            cout << MyFirst.size() << " " << MySecond.size() << endl;
            cout << AA << " " << BB << endl;
            cout << closest_dis << endl;
            cout << closest_3DPoint.x << " " << closest_3DPoint.y << " " << closest_3DPoint.z << endl;
#endif

            if (closest_dis == 987654321) {
               continue;
            }

            else {
               // pass
               Acummulate_Time[k] = 0;

               Current_Loc[k].x = closest_3DPoint.x;
               Current_Loc[k].y = closest_3DPoint.y;
               Current_Loc[k].z = closest_3DPoint.z;
            }

            cout << "Curren " << Current_Loc[k].x << " " << Current_Loc[k].y << " " << Current_Loc[k].z << endl;


            cout << "-------------------------------------------------------------------------------" << endl << endl;

         }

         std_msgs::Float64 drone1_x_msg;
         std_msgs::Float64 drone1_y_msg;
         std_msgs::Float64 drone1_z_msg;

         geometry_msgs::Point drone1_msg;
         geometry_msgs::Point drone2_msg;

         for (int i = 0; i < Dron_size; i++) {
            Real_Dron_Loc[i].x = Current_Loc[i].x;
            Real_Dron_Loc[i].y = Current_Loc[i].y;
            Real_Dron_Loc[i].z = Current_Loc[i].z;
#ifdef DEBUG
            cout << setw(10) << Real_Dron_Loc[i].x << " " << setw(10) << Real_Dron_Loc[i].y << " " << setw(10) << Real_Dron_Loc[i].z  << endl;
#endif

            static lpf_t lpf_x = {0, };
            static lpf_t lpf_y = {0, };
            static lpf_t lpf_z = {0, };
            lpf_x.cur_time = lpf_y.cur_time = lpf_z.cur_time = ros::Time::now().toSec();
            lpf_x.input = Real_Dron_Loc[i].x;
            lpf_y.input = Real_Dron_Loc[i].y;
            lpf_z.input = Real_Dron_Loc[i].z;

            drone1_msg.x = Real_Dron_Loc[i].x;//get_lpf(&lpf_x,10);
            drone1_msg.y = Real_Dron_Loc[i].z;//get_lpf(&lpf_z,10);
            drone1_msg.z = Real_Dron_Loc[i].y;//get_lpf(&lpf_y,5);

            drone2_msg.x = get_lpf(&lpf_x, 10);
            drone2_msg.y = get_lpf(&lpf_z, 10);
            drone2_msg.z = get_lpf(&lpf_y, 5);

            // pub_drone[i].publish(drone1_msg);
            // Nasang[i].publish(drone2_msg);
         }

      }
      cout << ros::Time::now().toSec() - cur << endl;;
   }


   void imageCb2(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& info_msg) {
      static int i = 0;
      if (!i)
         cout << "asdf" << i++ << endl;
      MySecond.clear();
      Ccount++;

      cam_model_right.fromCameraInfo(info_msg);

      cv_bridge::CvImagePtr cv_ptr;
      try
      {
         cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
      }
      catch (cv_bridge::Exception& e)
      {
         ROS_ERROR("cv_bridge exception: %s", e.what());
         return;
      }

      if (Ccount == 0)
         return;

      Mat image, gray, temp, mask;
      image = cv_ptr->image;

      mask = image.clone();

      int niters = 1;
      dilate(mask, temp, Mat(), Point(-1, -1), niters);
      erode(temp, temp, Mat(), Point(-1, -1), niters * 2);
      dilate(temp, temp, Mat(), Point(-1, -1), niters);

      threshold(temp, temp, 200, 255, CV_THRESH_BINARY);

      for (int i = 0; i <= 5; i++) {
         for (int j = 1000; j < temp.cols; j++) {
            temp.at<uchar>(i, j) = 0;
         }
      }

      vector<vector<Point> > contours1, contours2;
      vector<Vec4i> hierarchy;
      findContours(temp, contours2, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

      int cmin = 0;
      int cmax = 100;

      for (int i = 0; i < contours2.size(); i++) {
         if (contours2[i].size() < cmin || contours2[i].size() > cmax) {
            continue;
         }
         else
            contours1.push_back(contours2[i]);
      }


      image = Scalar(255, 255, 255);

      std::vector<std::vector<cv::Point> > ::iterator itc;
      itc = contours1.begin();

      MyPoint TTEMP;
      while (itc != contours1.end()) {

         cv::Moments mom = cv::moments(cv::Mat(*itc++));
#ifdef DEBUG
         cv::circle(image, cv::Point(mom.m10 / mom.m00, mom.m01 / mom.m00), 2, cv::Scalar(0), 2);
#endif
         TTEMP.x = mom.m10 / mom.m00;
         TTEMP.y = mom.m01 / mom.m00;

         if (TTEMP.x == 0 && TTEMP.y == 0)
            return;

         MySecond.push_back(TTEMP);
      }

#ifdef DEBUG
      cv::imshow(OPENCV_WINDOW2, cv_ptr->image);
      cv::waitKey(3);
#endif
   }
};

int main(int argc, char** argv)
{
   Start_time = clock();
   ros::init(argc, argv, "RECONSTRUCTION");
   RECONSTRUCTION ic;
   ros::spin();
   return 0;
}
