
#pragma once
#include "keyframe.hpp"
#include "posegraph.hpp"
// #include "rovio_zzlog.h"
#include <rerun.hpp>
#include <condition_variable>
#include <thread>
namespace rovio {
class LoopClosureDetector {
 public:
  LoopClosureDetector();
  void setRerunStream(rerun::RecordingStream* stream) {
      rerun_stream_ = stream;
  }
  void initialize(std::shared_ptr<const BriefVocabulary> voc, std::vector<int> &pattern_x1,
                  std::vector<int> &pattern_y1, std::vector<int> &pattern_x2,
                  std::vector<int> &pattern_y2);
  void LoopClosure();
  void stop_LoopClosure();
  ~LoopClosureDetector();
  inline void run_loop(const std::map<int,cv::Mat> &image, std::map<int,Eigen::Quaterniond> &qcm,
                      std::map<int,Eigen::Vector3d> &tmc, const Eigen::Quaterniond &qwm,
                      const Eigen::Vector3d &twm, const double &timestamp) {
    {
      std::unique_lock<std::mutex> lclock(mutexLoopClosing_);
      accept_loop = true;
      for(const auto& it : image){
          poseInfo_O1.images[it.first] = it.second;
          poseInfo_O1.qCM[it.first] = qcm[it.first];
          poseInfo_O1.MrMC[it.first] = tmc[it.first]; 
      } 
      int OmniCamNum = 4;
      // poseInfo.image = image;
      poseInfo_O1.timestamp = timestamp;
      poseInfo_O1.quaternion = qwm; //imu->world, in world
      poseInfo_O1.translation = twm;//word->imu in world
      // poseInfo.qCM = qcm;//imu->camera in camera
      // poseInfo.MrMC = tmc;//imu->camera in imu
    // }
    loopcv_.notify_one();
#ifdef PC_VERSION
    loopcv1_.wait(lclock);
#endif
    // std::this_thread::sleep_for(std::chrono::milliseconds(15));  // wait for the last frame to be processed
    }
    notify_num++;

  }

 public:
  PoseGraph poseGraph_;
  BriefExtractor briefExtractor_;
  std::shared_ptr<KeyFrame> lastKeyFrame_ = nullptr;
  Eigen::Vector3d twm_last_;
  std::condition_variable loopcv_;
  std::condition_variable loopcv1_;
  std::mutex mutexLoopClosing_;
  std::thread loopthread;

  struct PoseInfo_O1 {
    std::map<int,cv::Mat> images;
    double timestamp;
    Eigen::Quaterniond quaternion;//imu->world, in world
    std::map<int,Eigen::Quaterniond> qCM;//imu->camera in camera
    Eigen::Vector3d translation;//world->imu in world
    std::map<int,Eigen::Vector3d> MrMC;//imu->camera in imu
  };
  PoseInfo_O1 poseInfo_O1;

  Eigen::Vector3d w_t_vio = Eigen::Vector3d::Zero();
  Eigen::Matrix3d w_r_vio = Eigen::Matrix3d::Identity();
  Eigen::Vector3d target_w_t_vio = Eigen::Vector3d::Zero();
  double target_w_r_vio = 0;
  Eigen::Vector3d diff_w_t_vio = Eigen::Vector3d::Zero();
  double diff_w_r_vio = 0;
  Eigen::Vector3d last_w_t_vio = Eigen::Vector3d::Zero();
  double last_w_r_vio = 0;
  double yaw_w_r_vio = 0;
  bool isld = false;
  float ldsum = 0;
  float ldstep = 0;
  bool stop_loop = false;  //发出终止回环线程标志
  std::atomic<bool> run_loop_closure = true;  //发出终止回环线程后线程函数结束标志
  int notify_num = 0;
  int Fnotify_num = 0;
  bool accept_loop = false;
  std::ofstream loop_key;
  #ifdef Looplog
    std::ofstream logfile;
  #endif
  rerun::RecordingStream* rerun_stream_ = nullptr;  // 可选：用 shared_ptr 更安全
};

class CreateBOWVEC {
 public:
 struct MultiImageFrame {
    double timestamp;
    std::map<int, cv::Mat> image_map;  // key是相机名，比如"cam0","cam1"

    void addImage(const int cam_name, const cv::Mat& img) {
        image_map[cam_name] = img;
    }
};
  explicit CreateBOWVEC();
  ~CreateBOWVEC();
  void push(std::map<int, cv::Mat> images, const double &timestamp);
  void push_map(const std::map<int,Eigen::Quaterniond> &qcm, const std::map<int,Eigen::Vector3d> &tmc,
              const Eigen::Quaterniond &qwm, const Eigen::Vector3d &twm, const double timestamp);
  bool pop(MultiImageFrame &MTimage);
  void CreateBOWVEO1(std::shared_ptr<LoopClosureDetector> LoopClosure);
  void stop();
  void Forced_stop();
 public:
  struct TimestampedImage {
    cv::Mat image;
    double timestamp;
  };
  std::queue<TimestampedImage> imageQueue;  // stack  queue
  std::mutex optiflow_mtx;
  std::condition_variable optiflow_cv;
  std::condition_variable image_cv;
  std::thread optiflowthread;
  bool stop_optiflowthread = false;  //发出终止光流线程标志
  std::atomic<bool> run_optiflowthread = true;  //发出终止光流线程后线程函数结束标志
  std::ofstream optiflow_key;
  std::queue<MultiImageFrame> MTimageQueue;  // stack  queue


  struct PoseInfo_O1 {
    Eigen::Quaterniond quaternion;
    std::map<int,Eigen::Quaterniond> qCM;
    Eigen::Vector3d translation;
    std::map<int,Eigen::Vector3d> MrMC;
  };
  std::map<double, PoseInfo_O1> poseMap;
  int OmniCamNum = 4;

};  // namespace rovio
}