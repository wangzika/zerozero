#pragma once

#include <array>
#include <iostream>
#include <mutex>
#include <thread>
#include <map>
#include <unordered_map>
#include <memory>
#include <string>

#include <opencv2/opencv.hpp>
#include <rerun.hpp>
#include <unistd.h>

#include <replaykit/replaykit.h>

#include "loop_closure_detector.hpp"
#include "captain_msgs.pb.h"
#include "rovio.pb.h"
#include "image.pb.h"

// Forward declare ReplayKit types to reduce header dependency (optional but cleaner)
namespace zz::replaykit {
    template<typename Topics, typename Commands>
    class ReplayKit;
}

// Assume these are defined in loopClosing
namespace rovio {
    class LoopClosureDetector;
    class CreateBOWVEC;
}
// using BriefVocabulary = DBoW2::TemplatedVocabulary<DBoW2::BriefDescriptor>;

// Define the ReplayKit type (must match exactly)
using ReplayKitType = ::zz::replaykit::ReplayKit<
    ::zz::replaykit::Topics<captain::SensorData, vision::StereoImage, rovio::InputInfoPack>,
    ::zz::replaykit::Commands<>
>;

class Loopdetection {
public:
    Loopdetection(const std::string& replay_file,
                  const std::string& replay_speed = "1x",
                  const std::string& replay_skip_time = "0s");
    ~Loopdetection();

    void RunPlay();
    void loadPattern(const std::string& pattern_file);
    void initialize_loop();
    void resetO1(double t);
    bool areAllO1Valid();
    void sensorDataCallback(const captain::SensorData& sensordata);
    void processImage(const captain::Image& image, int index);
    void processOdometry(const captain::Odometry& odometry);
    void halfSample(const cv::Mat& imgIn, cv::Mat& imgOut);
    void computeFromImageLevel1(const cv::Mat& img, bool useLevel0, cv::Mat& imgs_);
    bool hasMeasurementAt(double t);
    void finish_loop();
    bool isFrameEmpty();
    void O1updateAndPublish(double updateTime);

public:
    ReplayKitType replaykit_;
    std::string replay_file_;
    std::string replay_speed_;
    std::string replay_skip_time_;
    rerun::RecordingStream rec_;
    static constexpr int Omni_nCam_ = 4;
    std::mutex m_filter_;

    bool isValidO1Pyr_[Omni_nCam_];
    std::atomic<bool> loop_mode=false;
    std::atomic<bool> first_mode=false;
    std::atomic<bool> end_mode=false;
    bool loop_mode_ = false;
    bool first_loop_ = false;
    bool end_loop_ = false;
    std::array<cv::Mat, 4> O1pyr_;
    std::array<cv::Point2f, 4> centers_;
    int nLevels_ = 4;
    std::map<int, cv::Mat> safe_O1imgBackup_;
    Eigen::Quaterniond qwm_ = Eigen::Quaterniond::Identity();
    std::map<int, Eigen::Quaterniond> qcm_;
    Eigen::Vector3d WrWM_ = Eigen::Vector3d::Zero();
    std::map<int, Eigen::Vector3d> MrMC_;

    std::shared_ptr<rovio::LoopClosureDetector> LoopClosure_;
    std::shared_ptr<rovio::CreateBOWVEC> queue;

    std::string BRIEF_PATTERN_FILE = "/home/zerozero/Project/Omni-O1/offboard/stream_monitor/brief_pattern.yml";
    std::string VOCABULARY_FILE = "/home/zerozero/Project/Omni-O1/offboard/stream_monitor/brief_k10L6.bin";
    std::shared_ptr<const BriefVocabulary> voc;
    std::vector<int> pattern_x1, pattern_y1, pattern_x2, pattern_y2;
    // private:
    std::array<cv::Mat, Omni_nCam_> current_images_; // 存储当前帧的4张图像
    std::mutex images_mutex_; // 保护 current_images_
    std::vector<Eigen::Vector3d> trajectory_; // 存储历史轨迹点
};

class ReplayViewer {
public:
    ReplayViewer(const std::string& replay_file,
                 const std::string& replay_speed = "1x",
                 const std::string& replay_skip_time = "0s");
    ~ReplayViewer();
    void Play();


private:
    void visualizeOdometry(const captain::Odometry& proto_odom);
    void visualizeImageInRerun(const captain::Image& proto_image, int image_index);

    ReplayKitType replaykit_;
    std::string replay_file_;
    std::string replay_speed_;
    std::string replay_skip_time_;
    rerun::RecordingStream rec_;
    Eigen::Quaterniond qwm_ = Eigen::Quaterniond::Identity();
    std::map<int, Eigen::Quaterniond> qcm_;
    Eigen::Vector3d WrWM_ = Eigen::Vector3d::Zero();
    std::map<int, Eigen::Vector3d> MrMC_;
    std::vector<Eigen::Vector3d> trajectory_; // 存储历史轨迹点

};