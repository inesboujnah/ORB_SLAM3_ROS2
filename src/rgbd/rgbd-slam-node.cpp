#include "rgbd-slam-node.hpp"

#include <opencv2/core/core.hpp>
#include <chrono>
#include <string>
#include <ctime>

using std::placeholders::_1;

RgbdSlamNode::RgbdSlamNode(ORB_SLAM3::System* pSLAM)
:   Node("ORB_SLAM3_ROS2"),
    m_SLAM(pSLAM)
{
    rgb_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(shared_ptr<rclcpp::Node>(this), "camera/rgb");
    depth_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(shared_ptr<rclcpp::Node>(this), "camera/depth");

    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy> >(approximate_sync_policy(10), *rgb_sub, *depth_sub);
    syncApproximate->registerCallback(&RgbdSlamNode::GrabRGBD, this);

}

RgbdSlamNode::~RgbdSlamNode()
{
    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory with timestamp
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    std::tm* tm_now = std::localtime(&time_t_now);
    char buffer[64];
    std::strftime(buffer, sizeof(buffer), "%Y-%m-%d_%H-%M-%S", tm_now);

    // Save TUM keyframe and full trajectory
    std::string kf_tum = "memory_register/orbslam_data/trajectory/rgbd_kf_tum_" + std::string(buffer) + ".txt";
    m_SLAM->SaveKeyFrameTrajectoryTUM(kf_tum);

    std::string full_tum = "memory_register/orbslam_data/trajectory/rgbd_full_tum_" + std::string(buffer) + ".txt";
    m_SLAM->SaveTrajectoryTUM(full_tum);

    // Save EuRoC keyframe and full trajectory
    std::string kf_euroc = "memory_register/orbslam_data/trajectory/rgbd_kf_euroc_" + std::string(buffer) + ".txt";
    m_SLAM->SaveKeyFrameTrajectoryEuRoC(kf_euroc);

    std::string full_euroc = "memory_register/orbslam_data/trajectory/rgbd_full_euroc_" + std::string(buffer) + ".txt";
    m_SLAM->SaveTrajectoryEuRoC(full_euroc);
}

void RgbdSlamNode::GrabRGBD(const ImageMsg::SharedPtr msgRGB, const ImageMsg::SharedPtr msgD)
{
    // Copy the ros rgb image message to cv::Mat.
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Copy the ros depth image message to cv::Mat.
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    m_SLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, Utility::StampToSec(msgRGB->header.stamp));
}
