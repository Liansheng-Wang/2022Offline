//
// 2021.10.28
// bornchow
// 采用深度图片进行壁障
//
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/Point.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;


//相机内参
const float depth_fx_ = 916.69999; 
const float depth_fy_ = 914.3686;

class DepthAvoid
{
private:
    float depth_camera_fx; //相机内参
    float depth_camera_fy;
    int thre_; // 二值化阈值
    float Max_depth_; //最大深度截断
    pcl::VoxelGrid<PointT> voxel_filter_;
    pcl::RadiusOutlierRemoval<PointT> rout_filter_;
public:
    DepthAvoid(/* args */);
    void getLocalTarget(cv::Mat img, geometry_msgs::Point &local_target);
    void depth2points(cv::Mat& depth, PointCloud::Ptr& cloud);
    ~DepthAvoid(){}
};

DepthAvoid::DepthAvoid(/* args */)
{
    depth_camera_fx = 387.229248046875 ; //相机内参
    depth_camera_fy = 387.229248046875 ;

    thre_ = 150;
    Max_depth_ = 7.0;

    voxel_filter_.setLeafSize(0.2f, 0.2f, 0.2f);
    rout_filter_.setRadiusSearch(0.01);
    rout_filter_.setMinNeighborsInRadius(10);
}

void DepthAvoid::getLocalTarget(cv::Mat img, geometry_msgs::Point &local_target){

    // get size
    int cols = img.cols;
    int rows = img.rows;
    std::cout << img.size() << std::endl;
    std::cout << " rows: " << img.rows << " " << "cols: " << img.cols << std::endl;
    cv::Rect rect(0, img.rows/2 - 50, img.cols, 100);
    cv::Mat roi_img = cv::Mat(img, rect);
    cv::imshow("roi_img", roi_img);
    cv::waitKey();
    std::cout << "111111111111111111" << std::endl;

    float value;
    cv::Mat roi_img_trun = cv::Mat::zeros(roi_img.size(), CV_32FC1);
    for (int m = 0; m < roi_img.rows; m++)
    {
        for (int n = 0; n < roi_img.cols; n++){
            value = roi_img.at<float>(m, n)*0.001;
            if (value > Max_depth_)
            {
                value = Max_depth_;
            }
            roi_img_trun.at<float>(m, n) = value;
        }
    }
    
    std::cout << "22222222222222222" << std::endl;
    double minVal = 0;
    double maxVal = 0;

    minMaxLoc(roi_img_trun, &minVal, &maxVal);
    cv::Mat img8bits = cv::Mat::zeros(roi_img_trun.size(), CV_8UC1);
    if (minVal != maxVal){
        roi_img_trun.convertTo(img8bits, CV_8U, 255.0 / (maxVal - minVal), -255.0*minVal / (maxVal - minVal));
    }

    std::cout << "3333333333333333333333333333" << std::endl;
    cv::Mat img_th;
    // cv::threshold(img8bits, img_th, thre_, 255, CV_THRESH_BINARY);  // 4m内无障碍  // 16上的问题
    cv::threshold(img8bits, img_th, thre_, 255, cv::THRESH_BINARY);


    cv::imshow("img_th", img_th);
    cv::waitKey();

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::Mat close_result;
    cv::morphologyEx(img_th, close_result, cv::MORPH_CLOSE, kernel);

    cv::Mat img_shaped = cv::Mat(close_result.size(), CV_8UC1, cv::Scalar(255));
    
    std::cout << "44444444444444444444" << std::endl;
    int has_obs_cont = 0;
    for(int n = 0; n < close_result.cols; n++){
        has_obs_cont = 0;
        for (int m = 0; m < close_result.rows; m++)
        {
            if (close_result.at<uchar>(m, n) == 0)
            {
                has_obs_cont += 1;
            }
        }

        if (has_obs_cont > close_result.rows*0.5)
        {
            for (int m = 0; m < close_result.rows; m++)
            {
                img_shaped.at<uchar>(m, n) = 0;
            }
        }
       
    }

    std::cout << "5555555555555555555555555" << std::endl;
    cv::imshow("img_shaped", img_shaped);
    cv::waitKey();

    cv::Mat close_result_show;
    // cv::cvtColor(img_shaped, close_result_show, CV_GRAY2BGR); 
    cv::cvtColor(img_shaped, close_result_show, cv::COLOR_BGR2GRAY);

    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    // cv::findContours(img_shaped, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);
    cv::findContours(img_shaped, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE);

    double maxArea = 0;
    int maxContourInd = 0;
    if (contours.size() == 0)
    {
        return;
    }

    std::cout << "66666666666666666666" << std::endl;
    
    for(int i = 0; i < contours.size(); i++){
        double area = cv::contourArea(contours[i]);
        if (area > maxArea)
        {
            maxArea = area;
            maxContourInd = i;
        }
    }

    cv::drawContours(close_result_show, contours, maxContourInd, cv::Scalar(255, 0, 0), 3);
    cv::imshow("close_result_show", close_result_show);
    cv::waitKey(1);

    cv::RotatedRect box = cv::minAreaRect(contours[maxContourInd]);
    float boxcenterx = box.center.x;

    cv::Mat depth_value_mat = roi_img_trun.row(int(roi_img_trun.rows/2)).clone();
    float value1;
    float sum = 0.0;
    int value_cnt = 0;
    float depth_value = Max_depth_*thre_/255.0;
    std::cout << "depth th : " << depth_value << std::endl;
    for (int n = 0; n < depth_value_mat.cols; n++){
        value1 = depth_value_mat.at<float>(0, n);
        if (value1 > depth_value)
        {
            continue;
        }else
        {
            sum += value1;
            value_cnt +=1;
        }
    }
    float detect_depth = 0.0;
    if (value_cnt > 0)
    {
        detect_depth = sum / value_cnt;
    }
    
    float positionerrox, positionerroy, positionerroz;
    positionerrox = detect_depth * (( img_shaped.cols / 2 ) - box.center.x) / depth_camera_fx ;   //(左右方向分辨率为640，中心点320，左飞为正)
    positionerroy = detect_depth * (( img_shaped.rows / 2 ) - box.center.y) / depth_camera_fy ;   //(上下方向分辨率为480，中心点240，上飞为正)
    positionerroz = detect_depth ;  

    local_target.x = positionerroz;
    local_target.y = positionerrox;
    local_target.z = positionerroy;

}


void DepthAvoid::depth2points(cv::Mat& depth, PointCloud::Ptr& cloud){
  cloud->clear();
  // cv::imwrite("/home/dasheng/Experiment/Outputs/eeeee/depth.png",depth);

  for (int m = 0; m < depth.rows; m++)
		for (int n = 0; n < depth.cols; n++)
		{
			double d = depth.ptr<float>(m)[n] * 0.001;
			if (d < 0.2 || d > 5.0)
				continue;
			PointT p;

			p.y = -d * (n - depth.cols/2) / depth_fx_;    // x
			p.z = -d * (m - depth.rows/2) / depth_fy_;    // y
			p.x = double(d);                              // z

      if(p.z > 2.5 || p.z < -2.5 ||p.y > 2.5 || p.y < -2.5) continue;
			cloud->points.push_back(p);
		}

	cloud->height = 1;
	cloud->width = cloud->points.size();
	cloud->is_dense = false;

  // rout_filter_.setInputCloud(cloud);
  // rout_filter_.filter(*cloud);
  voxel_filter_.setInputCloud(cloud);
  voxel_filter_.filter(*cloud);

  // pcl::io::savePCDFile("/home/dasheng/Experiment/Outputs/eeeee/cloud.pcd", *cloud);
}
