#pragma once

#include "DBoW2.h"
#include "DVision.h"
#include "feature.hpp"
// #include "rovio/FeatureManager.hpp"
// #include "rovio/FilterStates.hpp"
// #include "rovio/MultiCamera.hpp"
#include "utility.h"
#include <unordered_set>
#include <Eigen/Dense>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/opencv.hpp>
#include <vector>

namespace rovio {

//剔除status为0的点
template <typename Derived>
static void reduceVector(std::vector<Derived> &v, std::vector<uchar> status) {
  int j = 0;
  for (int i = 0; i < int(v.size()); i++)
    if (status[i]) {
      v[j++] = v[i];
    }
  v.resize(j);
}

class PoseGraph;
/**
 * @class BriefExtractor
 * @Description 通过Brief模板文件，对图像的关键点计算Brief描述子
 */
const int PATCH_SIZE = 31;
const int HALF_PATCH_SIZE = 15;
const int EDGE_THRESHOLD = 19;
class BriefExtractor {
 public:
  BriefExtractor() {
    m_umax.resize(HALF_PATCH_SIZE + 1);
    int v, v0, vmax = cvFloor(HALF_PATCH_SIZE * sqrt(2.f) / 2 + 1);
    int vmin = cvCeil(HALF_PATCH_SIZE * sqrt(2.f) / 2);
    const double hp2 = HALF_PATCH_SIZE * HALF_PATCH_SIZE;
    for (v = 0; v <= vmax; ++v)
      m_umax[v] = cvRound(sqrt(hp2 - v * v));

    // Make sure we are symmetric
    for (v = HALF_PATCH_SIZE, v0 = 0; v >= vmin; --v) {
      while (m_umax[v0] == m_umax[v0 + 1])
        ++v0;
      m_umax[v] = v0;
      ++v0;
    }
  }
  static float IC_Angle(const cv::Mat &image, cv::Point2f pt, const std::vector<int> &u_max) {
    int img_row = image.rows;
    int img_col = image.cols;
    if (pt.x <= HALF_PATCH_SIZE || pt.x >= (img_col - HALF_PATCH_SIZE) || pt.y <= HALF_PATCH_SIZE ||
        pt.y >= (img_row - HALF_PATCH_SIZE)) {
      return 0.0;
    }

    int m_01 = 0, m_10 = 0;

    const uchar *center = &image.at<uchar>(cvRound(pt.y), cvRound(pt.x));

    // Treat the center line differently, v=0
    for (int u = -HALF_PATCH_SIZE; u <= HALF_PATCH_SIZE; ++u)
      m_10 += u * center[u];

    // Go line by line in the circuI853lar patch
    int step = (int)image.step1();
    for (int v = 1; v <= HALF_PATCH_SIZE; ++v) {
      // Proceed over the two lines
      int v_sum = 0;
      int d = u_max[v];
      for (int u = -d; u <= d; ++u) {
        int val_plus = center[u + v * step], val_minus = center[u - v * step];
        v_sum += (val_plus - val_minus);
        m_10 += u * (val_plus + val_minus);
      }
      m_01 += v * v_sum;
    }

    return cv::fastAtan2((float)m_01, (float)m_10);
  }
  static void computeOrientation(const cv::Mat &image, std::vector<cv::KeyPoint> &keypoints,
                                 const std::vector<int> &umax) {
    for (std::vector<cv::KeyPoint>::iterator keypoint = keypoints.begin(),
                                             keypointEnd = keypoints.end();
         keypoint != keypointEnd; ++keypoint) {
      keypoint->angle = IC_Angle(image, keypoint->pt, umax);
    }
  }

  void InitialPattern(std::vector<int> &pattern_x1, std::vector<int> &pattern_y1,
                      std::vector<int> &pattern_x2, std::vector<int> &pattern_y2);
  virtual void operator()(const cv::Mat &im, std::vector<cv::KeyPoint> &keys,
                          std::vector<DVision::BRIEF::bitset> &descriptors) const;
  BriefExtractor(const std::string &pattern_file);

  std::vector<int> m_umax;

  DVision::BRIEF m_brief;
};

/**
 * @class KeyFrame
 * @Description 构建关键帧，通过BRIEF描述子匹配关键帧和回环候选帧
 */
class KeyFrame {
 public:
  KeyFrame();
  KeyFrame(std::map<int,cv::Mat>images) {
  for(auto &it:images){
      m_point_2d_norm[it.first].clear();
      m_point_2d_uv[it.first].clear();
      m_point_3d[it.first].clear();
      // m_point_id[it.first].clear();
      m_pointsid[it.first].clear();
      m_keypoints[it.first].clear();
      m_keypoints_norm[it.first].clear();
      vio_map_brief_descriptors[it.first].clear();
      all_brief_descriptors.clear();
      images_[it.first] = it.second;
  }
}
  ~KeyFrame();
  bool isImgEmpty() {
    // m_isempty = isempty;
    for(auto &it:images_){
      if(it.second.empty()){
        return true;
      }}
      return false;
  }
  void init(BriefExtractor *extractor, double &t, std::map<int,Eigen::Quaterniond> &qcm, std::map<int,Eigen::Vector3d> &tmc,
          Eigen::Quaterniond &qwm, Eigen::Vector3d &twm, PoseGraph *posegraph);
  void init(BriefExtractor *extractor, double &t, Eigen::Quaterniond &qcm, Eigen::Vector3d &tmc,
            Eigen::Quaterniond &qwm, Eigen::Vector3d &twm, PoseGraph *posegraph);
  Eigen::Matrix<double, 3, 3> skew_x(const Eigen::Matrix<double, 3, 1> &w);
  void setPoseGraph(PoseGraph *posegraph) {
    mpPoseGraph = posegraph;
  }
  bool findConnection(std::shared_ptr<KeyFrame> old_kf);
  bool findConnection_M(std::shared_ptr<KeyFrame> old_kf,const int image_idx);
  void computeBRIEFPoint();
  void computeBRIEFPoint_m();
  void computeBRIEFPoint2();
  void computeBRIEFPoint2_m();
  // void computeBRIEFPoint(std::vector<int> tempidx);
  // void extractBrief();
  cv::Mat convertBriefDescriptors(const std::vector<DVision::BRIEF::bitset> &briefDescriptors);
  int HammingDis(const DVision::BRIEF::bitset &a, const DVision::BRIEF::bitset &b);
  void FundmantalMatrixRANSAC(const std::vector<cv::Point2f> &matched_2d_cur_norm,
                              const std::vector<cv::Point2f> &matched_2d_old_norm,
                              std::vector<uchar> &status,int img_id);
  bool PnPRANSAC(const std::vector<cv::Point2f> &matched_2d_old_norm,
                 const std::vector<cv::Point3f> &matched_3d, std::vector<uchar> &status,
                 Eigen::Vector3d &PnP_T_old, Eigen::Matrix3d &PnP_R_old,int img_id);
  void getVioPose(Eigen::Vector3d &_T_w_i, Eigen::Matrix3d &_R_w_i);
  void getPose(Eigen::Vector3d &_T_w_i, Eigen::Matrix3d &_R_w_i);
  void updatePose(const Eigen::Vector3d &_T_w_i, const Eigen::Matrix3d &_R_w_i);
  void updateVioPose(const Eigen::Vector3d &_T_w_i, const Eigen::Matrix3d &_R_w_i);

  Eigen::Vector3d getLoopPNPT();
  double getLoopRelativeYaw();
  Eigen::Quaterniond getLoopPNPQ();

  BriefExtractor *pExtractor = NULL;

  // MultiCamera *mpMultiCamera = NULL;
  PoseGraph *mpPoseGraph = NULL;

  double time_stamp;
  static int count;
  int MaxPoints=250;//最大特征点数
  int index;
  Eigen::Vector3d vio_T_w_i;//世界系到imu的平移 in world
  Eigen::Matrix3d vio_R_w_i;//IMU到世界系的姿态 in world
  Eigen::Vector3d T_w_i;//世界到imu系的平移 in world
  Eigen::Matrix3d R_w_i;//IMU到世界系的姿态 in world
  Eigen::Vector3d origin_vio_T;//世界到imu系的平移 in world
  Eigen::Matrix3d origin_vio_R;//IMU到世界系的姿态 in world
  int pnpCount = 0;
  std::vector<int> pointsid;

  std::map<int,std::vector<int>> m_pointsid;//每个摄像头的特征点id

  Eigen::Vector3d origin_vio_T_mc;//尾摄IMU到camera的平移 in IMU
  Eigen::Matrix3d origin_vio_R_mc;//尾摄camera到IMU的姿态 in IMU

  std::map<int,Eigen::Vector3d> origin_vio_T_mc_map;//尾摄IMU到camera的平移 in IMU
  std::map<int,Eigen::Matrix3d> origin_vio_R_mc_map;//尾摄camera到IMU的姿态 in IMU

  cv::Mat image;
  std::vector<cv::Point3f> point_3d;
  std::vector<cv::Point2f> point_2d_uv;
  std::vector<cv::Point2f> point_2d_norm;

  std::map<int,cv::Mat> images_;
  std::map<int,std::vector<cv::Point3f>> m_point_3d;
  std::map<int,std::vector<cv::Point2f>> m_point_2d_uv;
  std::map<int,std::vector<cv::Point2f>> m_point_2d_norm;

  std::vector<cv::KeyPoint> keypoints;//为什么没有重置？
  std::vector<cv::KeyPoint> keypoints_norm;//为什么没有重置

  std::map<int,std::vector<cv::KeyPoint>> m_keypoints;
  std::map<int,std::vector<cv::KeyPoint>> m_keypoints_norm;
  // std::map<int,std::vector<cv::KeyPoint>> m_vio_keypoints;
  // std::map<int, std::vector<DVision::BRIEF::bitset>> map_brief_descriptors;
  std::vector<DVision::BRIEF::bitset> brief_descriptors;
  std::map<int,std::vector<DVision::BRIEF::bitset>> vio_map_brief_descriptors;
  std::vector<DVision::BRIEF::bitset> all_brief_descriptors;
  // std::vector<DVision::BRIEF::bitset> vio_brief_descriptors;


  bool has_loop;
  int loop_index;//
  int cur_frame_idx;//当前帧第几个图像
  Eigen::Matrix<double, 8, 1> loop_info;
  int MIN_LOOP_NUM = 19;  // 10;25
};

}  // namespace rovio
