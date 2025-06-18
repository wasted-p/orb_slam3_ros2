// #include "rgbd-slam-node.hpp"

#include "message_filters/subscriber.hpp"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <iostream>
#include <opencv2/core/core.hpp>

#include "System.h"

#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp/logging.hpp>

#include "Frame.h"
#include "Map.h"
#include "System.h"
#include "Tracking.h"

#include "utility.hpp"

#define RGB_IMAGE_RAW_TOPIC "/camera/color/image_raw"
#define DEPTH_IMAGE_RAW_TOPIC "/camera/depth/image_raw"
class RgbdSlamNode : public rclcpp::Node {

private:
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> rgb_sub;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>>
      depth_sub;

public:
  RgbdSlamNode(ORB_SLAM3::System *pSLAM)
      : Node("ORB_SLAM3_ROS2"), m_SLAM(pSLAM) {
    rgb_sub = std::make_shared<message_filters::Subscriber<ImageMsg>>(
        this, RGB_IMAGE_RAW_TOPIC);
    depth_sub = std::make_shared<message_filters::Subscriber<ImageMsg>>(
        this, DEPTH_IMAGE_RAW_TOPIC);

    syncApproximate = std::make_shared<
        message_filters::Synchronizer<approximate_sync_policy>>(
        approximate_sync_policy(10), *rgb_sub, *depth_sub);
    syncApproximate->registerCallback(&RgbdSlamNode::GrabRGBD, this);
  }

  ~RgbdSlamNode() {
    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
  }

private:
  using ImageMsg = sensor_msgs::msg::Image;
  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::Image, sensor_msgs::msg::Image>
      approximate_sync_policy;

  ORB_SLAM3::System *m_SLAM;

  cv_bridge::CvImageConstPtr cv_ptrRGB;
  cv_bridge::CvImageConstPtr cv_ptrD;

  std::shared_ptr<message_filters::Synchronizer<approximate_sync_policy>>
      syncApproximate;

  void GrabRGBD(const ImageMsg::SharedPtr msgRGB,
                const ImageMsg::SharedPtr msgD) {
    // Copy the ros rgb image message to cv::Mat.
    try {
      cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    // Copy the ros depth image message to cv::Mat.
    try {
      cv_ptrD = cv_bridge::toCvShare(msgD);
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat clean_depth;
    cv_ptrD->image.copyTo(clean_depth);

    // Clamp range to [0.3m, 4.5m] — discard invalid depth values
    for (int i = 0; i < clean_depth.rows; ++i) {
      for (int j = 0; j < clean_depth.cols; ++j) {
        float &d = clean_depth.at<float>(i, j);
        if (!std::isfinite(d) || d < 0.3f || d > 4.5f) {
          d = 0.0f; // treat as invalid
        }
      }
    }

    // Logging the Depth range
    // NOTE: Remove this after debugging
    double minVal, maxVal;
    cv::minMaxLoc(clean_depth, &minVal, &maxVal);

    int invalid_pixels = cv::countNonZero(clean_depth == 0.0f);
    int total_pixels = clean_depth.rows * clean_depth.cols;
    float invalid_ratio = (float)invalid_pixels / total_pixels;

    m_SLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image,
                      Utility::StampToSec(msgRGB->header.stamp));

    int tracking_state = m_SLAM->GetTrackingState();

    switch (tracking_state) {
    case -1:
      RCLCPP_WARN(this->get_logger(), "SLAM system not ready.");
      break;
    case 0:
      RCLCPP_WARN(this->get_logger(), "No images received yet.");
      break;
    case 1:
      RCLCPP_INFO(this->get_logger(), "SLAM not initialized.");
      break;
    case 2:
      RCLCPP_INFO(this->get_logger(), "SLAM is tracking.");
      break;
    case 3:
      RCLCPP_WARN(this->get_logger(), "SLAM tracking lost.");
      break;
    default:
      RCLCPP_WARN(this->get_logger(), "Unknown tracking state: %d",
                  tracking_state);
      break;
    }
  }
};

int main(int argc, char **argv) {
  if (argc < 3) {
    std::cerr
        << "\nUsage: ros2 run orbslam rgbd path_to_vocabulary path_to_settings"
        << std::endl;
    return 1;
  }

  rclcpp::init(argc, argv);

  // malloc error using new.. try shared ptr
  // Create SLAM system. It initializes all system threads and gets ready to
  // process frames.

  bool visualization = true;
  ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::RGBD,
                         visualization);

  auto node = std::make_shared<RgbdSlamNode>(&SLAM);
  std::cout << "============================ " << std::endl;

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
