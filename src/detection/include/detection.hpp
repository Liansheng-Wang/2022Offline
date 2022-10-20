#pragma once

#include <string>
#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include <Eigen/Geometry>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PointStamped.h>
#include <detection/object_detector.h>

// 模型路径

const std::string model_dir = ros::package::getPath("detection") + "/weights/pico_online"; 
// 相机内参
const float Fx = 320.0;
const float Cx = 320.0;
const float Fy = 320.0;
const float Cy = 240.0;
// 相机分辨率
int frame_cols = 640;
int frame_raws = 480;
// 圆环实际大小 ~米
double circle_scale = 2.0; 

class Detector
{
public:
  std::string rgb_topic;
  std::string depth_topic;
  std::string target_topic;
  std::string result_topic;

public:
  Detector(ros::NodeHandle& nh): nh_(nh)
  {
    nh_.param<std::string>("rgb_topic", rgb_topic, "/airsim_node/drone_1/front_center/Scene");
    nh_.param<std::string>("depth_topic", depth_topic, "/airsim_node/drone_1/front_center/DepthPlanar");
    nh_.param<std::string>("target_topic", target_topic, "/detect/targets");
    nh_.param<std::string>("result_topic", result_topic, "/paddleseg/image_show");

    rgbImg_sub_   = nh_.subscribe<sensor_msgs::Image>(rgb_topic, 1, &Detector::rgbHandle, this, ros::TransportHints().tcpNoDelay());
    depthImg_sub_ = nh_.subscribe<sensor_msgs::Image>(depth_topic, 1, &Detector::depthHandle, this, ros::TransportHints().tcpNoDelay());
    target_pub_   = nh_.advertise<geometry_msgs::PointStamped>(target_topic, 1);
    img_test_pub_ = nh_.advertise<sensor_msgs::Image>(result_topic, 1);
  }

  void rgbHandle(const sensor_msgs::Image::ConstPtr& msg){
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg,"bgr8");
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
    cv_ptr->image.copyTo(rgb_sub_img);
  }

  void depthHandle(const sensor_msgs::Image::ConstPtr& msg){
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Could not convert from '%s' to 'TYPE_32FC1'.", msg->encoding.c_str());
    }
    cv_ptr->image.copyTo(depth_sub_img);
    depth_sub_img.convertTo(depth_sub_img, CV_32F, 1); // 仿真相机比例因子设为 1，实际相机为 0.001
  }

  cv::Mat PredictImage(const cv::Mat input_img,
                     const cv::Mat input_depth_img,
                     const double threshold,
                     PaddleDetection::ObjectDetector* det
                    ){
    
    std::vector<double> det_t = {0, 0, 0};
    std::vector<cv::Mat> batch_imgs;
    batch_imgs.push_back(input_img);

    // Store all detected result
    std::vector<PaddleDetection::ObjectResult> result;
    std::vector<int> bbox_num;
    std::vector<double> det_times;

    det->Predict(batch_imgs, threshold, 0, 1, &result, &bbox_num, &det_times);
    
    int point_count=0;
    float total_value=0.0;
    cv::Mat vis_img;
    if(result.size()!=0)
    {
        // get labels and colormap
      auto labels = det->GetLabelList();
      auto colormap = PaddleDetection::GenerateColorMap(labels.size()); 
      cv::Mat im = batch_imgs[0];
      std::vector<PaddleDetection::ObjectResult> im_result;
      //int detect_num = 0;
      // for (int j = 0; j < bbox_num[0]; j++) {
        
        PaddleDetection::ObjectResult item = result[0];
        // if (item.confidence < threshold || item.class_id == -1) 
        //     continue;    // 过滤低于阈值的 bbox     
        //detect_num += 1;
        im_result.push_back(item);
        // printf("class=%d confidence=%.4f rect=[%d %d %d %d]\n",
        //         item.class_id,
        //         item.confidence,
        //         item.rect[0],
        //         item.rect[1],
        //         item.rect[2],
        //         item.rect[3]);        
      // }
      //std::cout << " The number of detected box: " << detect_num << std::endl;
      // 可视化 result
      vis_img = PaddleDetection::VisualizeResult(im, im_result, labels, colormap, false);

      // 计算框的深度

      for(int i = item.rect[0]/2; i < item.rect[2]/2; i++)
      {
        for(int j = item.rect[1]/2; j < item.rect[3]/2;j++)
        {
            float temp = input_depth_img.at<float>(j,i);
            if(temp > 0 && temp<=5)
            {
              point_count+=1;
              total_value+=temp;         
            }
        }
      }
      // std::cout<<" find depth point number is :"<< point_count <<std::endl;
      // std::cout<<" calculate depth is : "<< total_value/point_count <<std::endl;

      if( point_count <= 10 ){  // 没检测到深度
        estimate_scale = ( circle_scale /(item.rect[2]-item.rect[0]) + circle_scale /(item.rect[3]-item.rect[1])) / 2 ;
        result_x = estimate_scale * Fx ;
        result_y = estimate_scale * ( frame_cols/2 - ( item.rect[0]+item.rect[2] )/2 );
        result_z = estimate_scale * ( frame_raws/2 - ( item.rect[1]+item.rect[3] )/2 );
       }
      else{                         // 使用检测到的深度
        result_x = total_value/point_count;
        result_y = (total_value/point_count)*( frame_cols/2 - ( item.rect[0]+item.rect[2] )/2 ) / Fx ;
        result_z = (total_value/point_count)*( frame_raws/2 - ( item.rect[1]+item.rect[3] )/2 ) / Fy ;
      }

    }else{
      input_img.copyTo(vis_img);
      result_x = 10001;
      result_y = 0;
      result_z = 0;
    }
    

    det_t[0] += det_times[0];
    det_t[1] += det_times[1];
    det_t[2] += det_times[2];
    det_times.clear();

    return vis_img;
  
}

  void runThread()
  { 
    // transform(FLAGS_device.begin(),FLAGS_device.end(),FLAGS_device.begin(),::toupper);
    cv::Mat rgb_image_src; 
    cv::Mat depth_img_src;
    ros::Rate rate_ = ros::Rate(20.0);

    // 加载模型
    PaddleDetection::ObjectDetector det(model_dir,
                                        "CPU",
                                        false,                      
                                        1,                          
                                        "paddle",                   
                                        1,                         
                                        0,                         
                                        1,                          
                                        1280,                       
                                        640,                        
                                        false );                    
    
    while(rgb_sub_img.empty() || depth_sub_img.empty())
    {
      // ROS_INFO("*** no image or depth data ***");
      ros::spinOnce();
      rate_.sleep();
      continue;
    }
    
    while(ros::ok())
    { 
      // 图像处理流程
      rgb_sub_img.copyTo(rgb_image_src);
      depth_sub_img.copyTo(depth_img_src);
      cv::Mat output_img;
      output_img = PredictImage(rgb_image_src,depth_img_src,0.5,&det);

      if(img_test_pub_.getNumSubscribers() != 0){
        sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", output_img).toImageMsg();
        img_test_pub_.publish(img_msg);
      }

      geometry_msgs::PointStamped msg ;
      msg.point.x = result_x ; 
      msg.point.y = result_y ; 
      msg.point.z = result_z ; 
      target_pub_.publish(msg);
      ros::spinOnce();
    }
  }

public:
  // ros
  ros::NodeHandle nh_;
  ros::Subscriber odom_sub_;
  ros::Subscriber rgbImg_sub_;
  ros::Subscriber depthImg_sub_;
  ros::Publisher  target_pub_;
  ros::Publisher  img_test_pub_;
  // paddle
  cv::Mat rgb_sub_img;
  cv::Mat depth_sub_img;
  // 检测结果
  float result_x;
  float result_y;
  float result_z;
  float estimate_scale;

};
