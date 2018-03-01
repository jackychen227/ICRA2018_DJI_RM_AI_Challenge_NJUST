#ifndef IMAGE_PROD_CONS_HPP
#define IMAGE_PROD_CONS_HPP

//ROS headers
#include<ros/ros.h>
#include<image_transport/image_transport.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/image_encodings.h>

//OpenCV headers
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>

#include<iostream>

#include "settings.hpp"
using namespace std;
using namespace cv;


//template <typename TYPE, void (TYPE::*ImageProducer)() >
//void* _thread_t(void* param)//线程启动函数，声明为模板函数
//{
//    TYPE* This = (TYPE*)param;
//    This->ImageProducer();
//    return NULL;
//}

class ImageProdCons
{
public:
    ros::NodeHandle _nh;
    image_transport::ImageTransport _it;
    image_transport::Subscriber image_sub;
    image_transport::Publisher image_pub;

    string input_image_topic, output_image_topic;
    int    orginal_image_display, detecting_image_display;

//    char * config_file_name;
    string config_file_name;
//    string strA = (char*)"text";
    cv_bridge::CvImagePtr cv_ptr;
//    Settings * settings;
    Mat template_img;
    void imageCb(const sensor_msgs::ImageConstPtr& msg);
    void draw(Mat& img);
//    pthread_t t1, t2;
    void ImageProducer(Mat & img);
    void ImageConsumer();
public:
//    cv_bridge::CvImagePtr cv_ptr;
    Settings * settings;
    ArmorDetector  armor_detector;
    ImageProdCons()
      :_it(_nh)
    {
        //Loading default parameters for CV_bridge
        input_image_topic = "/infantry_red_1/camera/image_raw";
        //input_image_topic = "camera/image_raw";
        output_image_topic = "camera/image_cv_bridge";
        orginal_image_display = 1;
        detecting_image_display = 1;
        ROS_INFO("Sucessfully loaded default parameters");

        //Acessing parameters from yaml file
        try{
            _nh.getParam("input_image_topic", input_image_topic);
            _nh.getParam("output_image_topic", output_image_topic);
            _nh.getParam("orginal_image_display", orginal_image_display);
            _nh.getParam("detecting_image_display", detecting_image_display);

            ROS_INFO("Sucessfully loaded parameters from yaml file");
        }
        catch(int e){
            ROS_WARN("Parameters of CV_bridge are not properly lodaded from file, loading defaults");
        }

        //subscribe to input video and publish output video
        image_sub = _it.subscribe(input_image_topic, 1,&ImageProdCons::imageCb, this);
        image_pub = _it.advertise(output_image_topic, 1);

        config_file_name = (char*)"/home/cqw/catkin_ws/src/ICRA2018_DJI_RM_AI_Challenge_NJUST/vision/infantry_vision/config/param_config.xml";
        Settings setting(config_file_name);
        ROS_INFO("SHOW_IMAGE=%d", setting.show_image);
        settings = &setting;

        ArmorDetector armor_detectors(settings->armor);
        armor_detector = armor_detectors;
        Mat template_img = imread("/home/cqw/catkin_ws/src/ICRA2018_DJI_RM_AI_Challenge_NJUST/vision/infantry_vision/config/template.bmp");
        Mat small_template_img = imread("/home/cqw/catkin_ws/src/ICRA2018_DJI_RM_AI_Challenge_NJUST/vision/infantry_vision/config/small_template.bmp");
        armor_detector.initTemplate(template_img, small_template_img);
        armor_detector.setPara(settings->armor);
        ROS_INFO("armor_detectorsetPara_enemy_color=%d", armor_detector._para.enemy_color);
//        cout<<"min_light_gray="<<armor_detector->_para.min_light_gray<<endl;
        ROS_INFO("min_light_gray=%d", armor_detector._para.min_light_gray);
//        min_light_gray

//        pthread_create(&t1, NULL, _thread_t<ImageProdCons, &ImageProdCons::ImageProducer>, this);
    }


};

#endif // IMAGE_PROD_CONS_HPP
