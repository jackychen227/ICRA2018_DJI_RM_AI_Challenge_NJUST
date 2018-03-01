//ROS headers
#include<ros/ros.h>
#include<image_transport/image_transport.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/image_encodings.h>

//OpenCV headers
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>

#include<iostream>
#include<stdlib.h>
#include<thread>

#include "settings.hpp"
#include "armor_detector.hpp"
#include "image_prod_cons.hpp"
using namespace std;
using namespace cv;

#define BUFFER_SIZE 1
volatile unsigned int prdIdx =0;
volatile unsigned int csmIdx= 0;
volatile unsigned int tr_begin= 0;
struct ImageData{
    Mat img;
    unsigned int frame;
};
ImageData data[BUFFER_SIZE];

void ImageProdCons::imageCb(const sensor_msgs::ImageConstPtr& msg){
  //cv_bridge::CvImagePtr cv_ptr;
  namespace env = sensor_msgs::image_encodings;

  try{
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch(cv_bridge::Exception& e){
      ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }
  if(!tr_begin) tr_begin = 1;
  ROS_INFO("S1");
  //detectorAndDraw(cv_ptr->image,...);
  if(orginal_image_display == 1){
      imshow("Original image from CV_bridge", cv_ptr->image);
//      cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
  }
//  draw(cv_ptr->image);
  ImageProducer(cv_ptr->image);
  ImageConsumer();
  image_pub.publish(cv_ptr->toImageMsg());
  waitKey(30);
}

void ImageProdCons::ImageProducer(Mat & img){
    ROS_INFO("ImageProducer");
//    while(1){
//        ROS_INFO("ImageProducer");
//        if(tr_begin){
//            while(prdIdx - csmIdx >= BUFFER_SIZE);
            ROS_INFO("prdIdx=%d",prdIdx);
            ROS_INFO("prdIdx-csmIdx=%d",prdIdx-csmIdx);

            int t1 = cv::getTickCount();
            data[prdIdx % BUFFER_SIZE].img = img;
            data[prdIdx % BUFFER_SIZE].frame++;
            int t2 = cv::getTickCount();
            imshow("ImageProducer", data[prdIdx % BUFFER_SIZE].img);
            cout << "Producer-Time: " << (t2 - t1) * 1000.0 / cv::getTickFrequency() << "ms\n";
            ++prdIdx;
//        }
//    }
}

void ImageProdCons::ImageConsumer(){
    ROS_INFO("ImageConsumer");
//    Settings & setting = *settings;
//    static int init_flag = 0;
//    if(!init_flag){
//        Settings & setting = *settings;
//        ArmorDetector armor_detector(setting.armor);
//        Mat template_img = imread("/home/cqw/catkin_ws/src/ICRA2018_DJI_RM_AI_Challenge_NJUST/vision/infantry_vision/config/template.bmp");
//        Mat small_template_img = imread("/home/cqw/catkin_ws/src/ICRA2018_DJI_RM_AI_Challenge_NJUST/vision/infantry_vision/config/small_template.bmp");
//        armor_detector.initTemplate(template_img, small_template_img);
//        armor_detector.setPara(setting.armor);
//    }
//    init_flag = 1;
//    Mat template_img = imread(setting.template_image_file);
//    Mat small_template_img = imread(setting.small_template_image_file);
//    armor_detector.initTemplate(template_img, small_template_img);
//    armor_detector.setPara(setting.armor);

    Mat src_csm;
    int t1 = 0, t2 = 0;
    Mat src;
    int frame_num = 0;


//    while(1){
        // waiting for image data ready
//        while(prdIdx - csmIdx == 0);
    if(prdIdx - csmIdx > 0){
        data[csmIdx % BUFFER_SIZE].img.copyTo(src);
        frame_num = data[csmIdx % BUFFER_SIZE].frame;
//        ++csmIdx;
//        imshow("src", src);
        if(settings->show_image || settings->save_result){
            t1 = cv::getTickCount();
            src.copyTo(src_csm);
//            cout<<"settings->show_image="<<settings->show_image<<endl;
//            cout<<"settings->save_result="<<settings->save_result<<endl;

//            cout<<"settings->enemy_color="<<settings->armor.enemy_color<<endl;

//            ROS_INFO("ImageConsumer");
        }

        RotatedRect rect;
        rect = armor_detector.getTargetAera(src);
        ROS_INFO("rect armor_detectorsetPara_enemy_color=%d", armor_detector._para.enemy_color);
//        if(settings->mode ==ARMOR_MODE){// armor detection mode
////           armor_detector->setPara(settings->armor);
//           rect = armor_detector->getTargetAera(src);
//        }//End of "if(setting.mode ==ARMOR_MODE)"


        t2 = cv::getTickCount();
        cout << "Consumer-Time: " << (t2 - t1) * 1000.0 / cv::getTickFrequency() << "ms   frame No.:" << frame_num << endl;
//    }
        ++csmIdx;
    }
}

void ImageProdCons::draw(Mat& img)
{
    Point pt0, pt1;

    pt0.x =320;
    pt0.y = 0;
    pt1.x = 320;
    pt1.y = 480;

    line(img,  pt0,  pt1, Scalar(0, 0, 255),0.2);

    putText(img, "Left", cvPoint(50,240), FONT_HERSHEY_SIMPLEX, 1, cvScalar(255,0,0), 2, CV_AA);
    imshow( "try to draw on the image", img );
}



int main(int argc, char** argv){
  ros::init(argc, argv,"Raw_image_to_CV_image");
  ImageProdCons image_cons_prod;

  //pthread_t t1;
//  pthread_create(&t1,NULL, &(ImageProdCons::ImageProducer), image_cons_prod);
//      std::thread t1(&ImageProdCons::ImageProducer, image_cons_prod);
//      t1.join();
//  ros::MultiThreadedSpinner s(4);
////  s.spin();
  ros::spin();
  return 0;
}

