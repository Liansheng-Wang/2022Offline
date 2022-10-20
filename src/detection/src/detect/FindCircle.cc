#include <algorithm>
#include <chrono>
#include <iostream>
#include <fstream>
#include <numeric>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "paddle/include/paddle_inference_api.h"
#include "yaml-cpp/yaml.h"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <ros/package.h>
#include "detection/object_detector.h"

using namespace std;


DEFINE_string(model_dir, "/home/xinke/paddle_ws/src/vision/weights/picodet_320", "Path of inference model");
DEFINE_int32(batch_size, 1, "batch_size");
DEFINE_int32(camera_id, -1, "Device id of camera to predict");
DEFINE_string(device,
              "GPU",
              "Choose the device you want to run, it can be: CPU/GPU/XPU, "
              "default is CPU.");
DEFINE_double(threshold, 0.5, "Threshold of score.");
DEFINE_string(run_mode,
              "paddle",
              "Mode of running(paddle/trt_fp32/trt_fp16/trt_int8)");
DEFINE_int32(gpu_id, 0, "Device id of GPU to execute");
DEFINE_bool(run_benchmark,
            false,
            "Whether to predict a image_file repeatedly for benchmark");
DEFINE_bool(use_mkldnn, false, "Whether use mkldnn with CPU");
DEFINE_int32(cpu_threads, 1, "Num of threads with CPU");
DEFINE_int32(trt_min_shape, 1, "Min shape of TRT DynamicShapeI");
DEFINE_int32(trt_max_shape, 1280, "Max shape of TRT DynamicShapeI");
DEFINE_int32(trt_opt_shape, 640, "Opt shape of TRT DynamicShapeI");
DEFINE_bool(trt_calib_mode,
            false,
            "If the model is produced by TRT offline quantitative calibration, "
            "trt_calib_mode need to set True");


cv::Mat img_sub;  // 订阅图像
cv::Mat depimg_sub;

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
      printf("class=%d confidence=%.4f rect=[%d %d %d %d]\n",
              item.class_id,
              item.confidence,
              item.rect[0],
              item.rect[1],
              item.rect[2],
              item.rect[3]);        
    // }
    //std::cout << " The number of detected box: " << detect_num << std::endl;
    // 可视化 result
    cv::Mat vis_img = PaddleDetection::VisualizeResult(im, im_result, labels, colormap, false);

    // 计算框的深度
    int point_count=0;
    float total_value=0.0;
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
    std::cout<<" find depth point number is :"<< point_count <<std::endl;
    std::cout<<" calculate depth is : "<< total_value/point_count <<std::endl;
    
    det_t[0] += det_times[0];
    det_t[1] += det_times[1];
    det_t[2] += det_times[2];
    det_times.clear();

    return vis_img;
  
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg,"bgr8");
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
  cv_ptr->image.copyTo(img_sub);

}

void depthCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::TYPE_32FC1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'TYPE_32FC1'.", msg->encoding.c_str());
  }
  cv_ptr->image.copyTo(depimg_sub);
  depimg_sub.convertTo(depimg_sub, CV_32F, 1); // 深度转换成米

}


int main(int argc, char *argv[]) {
    
    ROS_INFO("=== FIND CIRCLE ===");
    transform(FLAGS_device.begin(),
              FLAGS_device.end(),
              FLAGS_device.begin(),
              ::toupper);

    std::cout<<"1"<<std::endl;
    ros::init(argc, argv, "FindCircle_node");
    ros::NodeHandle nh;
    ros::Rate rate_ = ros::Rate(20.0);

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/airsim_node/drone_1/front_center/Scene", 1, imageCallback);  // 订阅 RGB
    image_transport::Subscriber dep_sub = it.subscribe("/airsim_node/drone_1/front_center/DepthPlanar", 1, depthCallback);  // 订阅 depth
    image_transport::Publisher pub = it.advertise("/paddleseg/image_show", 1);                    // 发布结果
    cv::Mat rgb_image_src; 
    cv::Mat depth_img_src;

    // Load model and create a object detector
    PaddleDetection::ObjectDetector det(FLAGS_model_dir,
                                        FLAGS_device,
                                        FLAGS_use_mkldnn,
                                        FLAGS_cpu_threads,
                                        FLAGS_run_mode,
                                        FLAGS_batch_size,
                                        FLAGS_gpu_id,
                                        FLAGS_trt_min_shape,
                                        FLAGS_trt_max_shape,
                                        FLAGS_trt_opt_shape,
                                        FLAGS_trt_calib_mode);

    while(img_sub.empty() || depimg_sub.empty())
    {
      ROS_INFO("*** no image or depth data ***");
      ros::spinOnce();
      rate_.sleep();
      continue;
    }

    while (ros::ok())
    { 
      // // Prepare data
      img_sub.copyTo(rgb_image_src);
      depimg_sub.copyTo(depth_img_src);

      cv::Mat output_img;
      output_img = PredictImage(rgb_image_src,depth_img_src,0.5,&det);
      
      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", output_img).toImageMsg();
      pub.publish(msg);
      ROS_INFO("=== finish filter ===");

      ros::spinOnce();
      rate_.sleep();
        
      }     

}
