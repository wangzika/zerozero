#include "keyframe.hpp"

// #include "Turnback.hpp"
#include "posegraph.hpp"
// #include "rovio/FeatureManager.hpp"
// #include "rovio_zzlog.h"
#include "utility.h"
// #include "vioTrace.h"

#include <algorithm>
#include <ctime>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <string>
// #include "zzTimer.hpp"

// #include <future>
// #include <chrono>
// static bool file_created_lc = false;
// static FILE *fp_lc = NULL;

// #define USE_CUR_AS_REF3D 1
namespace rovio {

//读取 构建字典时使用的相同的Brief模板文件，构造BriefExtractor
BriefExtractor::BriefExtractor(const std::string &pattern_file) {
  // The DVision::BRIEF extractor computes a random pattern by default when
  // the object is created.
  // We load the pattern that we used to build the vocabulary, to make
  // the descriptors compatible with the predefined vocabulary

  // loads the pattern

  cv::FileStorage fs(pattern_file.c_str(), cv::FileStorage::READ);
  if (!fs.isOpened()){
    throw string("Could not open file ") + pattern_file;
  }

  std::vector<int> x1, y1, x2, y2;
  fs["x1"] >> x1;
  fs["x2"] >> x2;
  fs["y1"] >> y1;
  fs["y2"] >> y2;
  fs.release();
  m_brief.importPairs(x1, y1, x2, y2);
}
void BriefExtractor::InitialPattern(std::vector<int> &pattern_x1, std::vector<int> &pattern_y1,
                                    std::vector<int> &pattern_x2, std::vector<int> &pattern_y2) {
  m_brief.importPairs(pattern_x1, pattern_y1, pattern_x2, pattern_y2);
}
//计算Brief描述子
void BriefExtractor::operator()(const cv::Mat &im, std::vector<cv::KeyPoint> &keys,
                                std::vector<DVision::BRIEF::bitset> &descriptors) const {
  m_brief.compute(im, keys, descriptors);

  // computeOrientation(im, keys, m_umax);
  // m_brief.computeCombineAngle(im, keys, descriptors);
}

int KeyFrame::count = 0;
KeyFrame::KeyFrame() {

};
// KeyFrame::KeyFrame(MultiCamera *pMultiCamera, cv::Mat image_, bool isempty) {
//   mpMultiCamera = pMultiCamera;
//   point_3d.clear();
//   point_2d_uv.clear();
//   point_2d_norm.clear();
//   // point_id.clear();
//   if (!isempty) {
//     mpMultiCamera->cameras_[1].undistortImg(image_, image);
//   }
// }
// KeyFrame::KeyFrame(MultiCamera *pMultiCamera, std::map<int,cv::Mat>images_, bool isempty) {
//   mpMultiCamera = pMultiCamera;
//   for(auto &it:images_){
//       m_point_2d_norm[it.first].clear();
//       m_point_2d_uv[it.first].clear();
//       m_point_3d[it.first].clear();
//       // m_point_id[it.first].clear();
//       m_pointsid[it.first].clear();
//       m_keypoints[it.first].clear();
//       m_keypoints_norm[it.first].clear();
//       vio_map_brief_descriptors[it.first].clear();
//       all_brief_descriptors.clear();
//       mpMultiCamera->cameras_[it.first].undistortImg(it.second, images_[it.first]);
//   }
//   // point_3d.clear();
//   // point_2d_uv.clear();
//   // point_2d_norm.clear();
//   // point_id.clear();
//   // if (!isempty) {
//   //   mpMultiCamera->cameras_[1].undistortImg(image_, image);
//   // }
// }
KeyFrame::~KeyFrame() {}
void KeyFrame::init(BriefExtractor *extractor, double &t, std::map<int,Eigen::Quaterniond> &qcm,
                    std::map<int,Eigen::Vector3d> &tmc, Eigen::Quaterniond &qwm, Eigen::Vector3d &twm,
                    PoseGraph *posegraph) {
  
  pExtractor = extractor;
  index = count;
  count++;
  time_stamp = t;
  vio_T_w_i = twm;//world到IMU的平移 in world 
  vio_R_w_i = qwm.toRotationMatrix();//IMU到世界系的姿态 in world
  T_w_i = vio_T_w_i;//world到IMU的平移 in world 
  R_w_i = vio_R_w_i;//IMU到世界系的姿态 in world
  origin_vio_T = vio_T_w_i;//world到IMU的平移 in world
  origin_vio_R = vio_R_w_i;//IMU到世界系的姿态 in world

  for(auto &it:tmc){
      Eigen::Matrix3d Rmc = qcm[it.first].toRotationMatrix().transpose();
      origin_vio_T_mc_map[it.first] = it.second;//尾摄IMU到camera的平移 in IMU
      origin_vio_R_mc_map[it.first] = Rmc;//尾摄camera到IMU的姿态 in IMU

  }

  mpPoseGraph = posegraph;
  has_loop = false;
  loop_index = -1;
  loop_info << 0, 0, 0, 0, 0, 0, 0, 0;
}
void KeyFrame::init(BriefExtractor *extractor, double &t, Eigen::Quaterniond &qcm,
                    Eigen::Vector3d &tmc, Eigen::Quaterniond &qwm, Eigen::Vector3d &twm,
                    PoseGraph *posegraph) {
  
  pExtractor = extractor;
  index = count;
  count++;
  time_stamp = t;
  vio_T_w_i = twm;//world到IMU的平移 in world 
  vio_R_w_i = qwm.toRotationMatrix();//IMU到世界系的姿态 in world
  T_w_i = vio_T_w_i;//world到IMU的平移 in world 
  R_w_i = vio_R_w_i;//IMU到世界系的姿态 in world
  origin_vio_T = vio_T_w_i;//world到IMU的平移 in world
  origin_vio_R = vio_R_w_i;//IMU到世界系的姿态 in world

  origin_vio_T_mc = tmc;//尾摄IMU到camera的平移 in IMU
  origin_vio_R_mc = qcm.toRotationMatrix().transpose();//尾摄camera到IMU的姿态 in IMU
  mpPoseGraph = posegraph;
  has_loop = false;
  loop_index = -1;
  loop_info << 0, 0, 0, 0, 0, 0, 0, 0;
}
// void KeyFrame::init(BriefExtractor *extractor, FilterState *pFilterState, PoseGraph *posegraph) {
//   pExtractor = extractor;
//   index = count;
//   count++;

//   Eigen::Quaterniond qwm(pFilterState->state_.qWM().w(), pFilterState->state_.qWM().x(),
//                          pFilterState->state_.qWM().y(), pFilterState->state_.qWM().z());
//   Eigen::Vector3d twm = pFilterState->state_.WrWM();

//   Eigen::Quaterniond qcm(
//       pFilterState->state_.qCM(CAM_ID::TAIL).w(), pFilterState->state_.qCM(CAM_ID::TAIL).x(),
//       pFilterState->state_.qCM(CAM_ID::TAIL).y(), pFilterState->state_.qCM(CAM_ID::TAIL).z());
//   Eigen::Matrix3d Rmc = qcm.toRotationMatrix().transpose();
//   Eigen::Vector3d tmc = pFilterState->state_.MrMC(CAM_ID::TAIL);
//   time_stamp = pFilterState->t_;
//   vio_T_w_i = twm;
//   vio_R_w_i = qwm.toRotationMatrix();
//   T_w_i = vio_T_w_i;
//   R_w_i = vio_R_w_i;
//   origin_vio_T = vio_T_w_i;
//   origin_vio_R = vio_R_w_i;

//   origin_vio_T_mc = tmc;
//   origin_vio_R_mc = Rmc;
//   mpPoseGraph = posegraph;
//   has_loop = false;
//   loop_index = -1;
//   loop_info << 0, 0, 0, 0, 0, 0, 0, 0;
// }
Eigen::Matrix<double, 3, 3> KeyFrame::skew_x(const Eigen::Matrix<double, 3, 1> &w) {
  Eigen::Matrix<double, 3, 3> w_x;
  w_x << 0, -w(2), w(1), w(2), 0, -w(0), -w(1), w(0), 0;
  return w_x;
}
// 



//额外检测新特征点并计算所有特征点的brief描述子，为了回环检测,并统计每个3D点ID对应的图像坐标，归一化坐标、位姿等
//用于环绕第一圈
void KeyFrame::computeBRIEFPoint_m() {
  std::cout<<"computeBRIEFPoint_m: START"<<std::endl;
  for(const auto &it:images_){
    // cv::Mat mask = cv::Mat(it.second.rows, it.second.cols, CV_8UC1, cv::Scalar(255));
    // for(auto &pt : m_keypoints[it.first]) {
    //   cv::Point pt1 = cv::Point(int(pt.pt.x), (pt.pt.y));
    //   if (mask.at<uchar>(pt1) == 255){
    //     cv::circle(mask, pt1, 20, 0, -1);
    //   }
    // }
    //如果追踪后的点不到250个，用mask掩码提取新的特征点
    // int needPtsCnt = MaxPoints - static_cast<int>(m_keypoints[it.first].size());
    // std::cout<<"needPtsCnt: "<<needPtsCnt<<std::endl;
    // if (needPtsCnt > 0) {
    //   std::vector<cv::Point2f> tempPts;
    //   tempPts.reserve(needPtsCnt);

    //   // cv::goodFeaturesToTrack(it.second, tempPts, needPtsCnt, 0.01, 10, mask);
    //   std::cout<<"tempPts size: "<<tempPts.size()<<std::endl;
    //   for (auto &pt : tempPts) {
    //     m_keypoints[it.first].emplace_back(pt, 1.0f);
    //     m_pointsid[it.first].push_back(mpPoseGraph->keyptsidx);
    //     // mpPoseGraph->map_sequence_landMarkInfo[it.first][mpPoseGraph->keyptsidx] = LandMarkInfo();
    //     mpPoseGraph->keyptsidx++;
    //   }
    // }
    std::vector<cv::Point2f> tempPts;
    
    cv::goodFeaturesToTrack(it.second, tempPts, 250, 0.01, 10);
    // std::cout<<"it.second image"<<it.second.size();
    for (int i = 0; i < tempPts.size(); i++) {
      m_keypoints[it.first].emplace_back(tempPts[i], 1.0f);
    }

    // 对合并后的所有特征点计算BRIEF描述子
    (*pExtractor)(it.second, m_keypoints[it.first], vio_map_brief_descriptors[it.first]);
    // (*pExtractor)(image, keypoints, brief_descriptors);

    // 对每个特征点进行去畸变矫正，并记录归一化后的特征点和ID
    for (int i = 0; i < (int)m_keypoints[it.first].size(); i++) {
      Eigen::Vector3d tmp_p;
      // mpMultiCamera->cameras_[it.first].undistpixelToNormal(m_keypoints[it.first][i].pt, tmp_p);
      cv::KeyPoint tmp_norm;
      tmp_norm.pt = cv::Point2f(tmp_p.x() / tmp_p.z(), tmp_p.y() / tmp_p.z());
      m_keypoints_norm[it.first].push_back(tmp_norm);
      // FeatureInfo temp;
      // temp.c_ << m_keypoints[it.first][i].pt.x, m_keypoints[it.first][i].pt.y;
      // temp.cNorm_ << tmp_norm.pt.x, tmp_norm.pt.y;
      // temp.R_wc = vio_R_w_i * origin_vio_R_mc_map[it.first];
      // temp.t_wc = vio_R_w_i * origin_vio_T_mc_map[it.first] + vio_T_w_i;
      // //统计每个3D点ID对应的图像坐标，归一化坐标、位姿等
      // mpPoseGraph->map_sequence_landMarkInfo[it.first][m_pointsid[it.first][i]].addPixelPose(temp, time_stamp);
      // origin_vio_T_mc
    }
}}


//提取特征点计算BRIEF描述子,并记录归一化后的特征点，环绕用于第二圈第三圈
void KeyFrame::computeBRIEFPoint2_m() {
  std::cout<<"computeBRIEFPoint2_m: START"<<std::endl;
  for(auto &it:images_){
    std::vector<cv::Point2f> tempPts;
    cv::goodFeaturesToTrack(it.second, tempPts, 250, 0.01, 10);
    // std::cout<<"it.second image"<<it.second.size();
    std::cout<<"tempPts size: "<<tempPts.size()<<std::endl;
    for (int i = 0; i < tempPts.size(); i++) {
      m_keypoints[it.first].emplace_back(tempPts[i], 1.0f);
    }

    (*pExtractor)(it.second, m_keypoints[it.first], vio_map_brief_descriptors[it.first]);
    // std::cout<<"vio_map_brief_descriptors size: "<<vio_map_brief_descriptors[it.first].size()<<std::endl;

    //将特征点去畸变矫正
    for (int i = 0; i < (int)m_keypoints[it.first].size(); i++) {
      Eigen::Vector3d tmp_p;
      // mpMultiCamera->cameras_[it.first].undistpixelToNormal(m_keypoints[it.first][i].pt, tmp_p);
      cv::KeyPoint tmp_norm;
      tmp_norm.pt = cv::Point2f(tmp_p.x() / tmp_p.z(), tmp_p.y() / tmp_p.z());
      m_keypoints_norm[it.first].push_back(tmp_norm);
    }
  }
}
// //通过RANSAC的基本矩阵检验去除匹配异常的点
// void KeyFrame::FundmantalMatrixRANSAC(const std::vector<cv::Point2f> &matched_2d_cur_norm,
//                                       const std::vector<cv::Point2f> &matched_2d_old_norm,
//                                       std::vector<uchar> &status,int img_id) {
//   double fx = mpMultiCamera->cameras_[img_id].K_(0, 0);
//   double fy = mpMultiCamera->cameras_[img_id].K_(1, 1);
//   double cx = mpMultiCamera->cameras_[img_id].K_(0, 2);
//   double cy = mpMultiCamera->cameras_[img_id].K_(1, 2);

//   int n = (int)matched_2d_cur_norm.size();
//   for (int i = 0; i < n; i++) {
//     status.push_back(0);
//   }
//   if (n >= 8) {
//     std::vector<cv::Point2f> tmp_cur(n), tmp_old(n);
//     for (int i = 0; i < (int)matched_2d_cur_norm.size(); i++) {
//       double tmp_x, tmp_y;
//       tmp_x = fx * matched_2d_cur_norm[i].x + cx;
//       tmp_y = fy * matched_2d_cur_norm[i].y + cy;
//       tmp_cur[i] = cv::Point2f(tmp_x, tmp_y);

//       tmp_x = fx * matched_2d_old_norm[i].x + cx;
//       tmp_y = fy * matched_2d_old_norm[i].y + cy;
//       tmp_old[i] = cv::Point2f(tmp_x, tmp_y);
//     }
//     cv::findFundamentalMat(tmp_cur, tmp_old, cv::FM_RANSAC, 2.0, 0.95, status);//1,0.99
//   }
// }
//通过RANSAC的PNP检验去除匹配异常的点
// bool KeyFrame::PnPRANSAC(const std::vector<cv::Point2f> &matched_2d_old_norm,
//                          const std::vector<cv::Point3f> &matched_3d, std::vector<uchar> &status,
//                          Eigen::Vector3d &PnP_T_old, Eigen::Matrix3d &PnP_R_old,int img_id) {
//   // Eigen::Quaterniond qci(mpFilterState->state_.qCM().w(), mpFilterState->state_.qCM().x(),
//   // mpFilterState->state_.qCM().y(), mpFilterState->state_.qCM().z());
//   Eigen::Matrix3d qic = origin_vio_R_mc;  // qci.toRotationMatrix().transpose(); //origin_vio_R_mc
//   Eigen::Vector3d tic = origin_vio_T_mc;  // mpFilterState->state_.MrMC(); //origin_vio_T_mc

//   // for (int i = 0; i < matched_3d.size(); i++)
//   //	printf("3d x: %f, y: %f, z: %f\n",matched_3d[i].x, matched_3d[i].y, matched_3d[i].z );
//   // printf("match size %d \n", matched_3d.size());
//   cv::Mat r, rvec, t, D, tmp_r;
//   t.create(3, 1, CV_64F);
//   tmp_r.create(3, 3, CV_64F);
//   cv::Mat K = (cv::Mat_<double>(3, 3) << 1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0);
//   Eigen::Matrix3d R_inital;
//   Eigen::Vector3d P_inital;
//   Eigen::Matrix3d R_w_c = origin_vio_R * qic;
//   Eigen::Vector3d T_w_c = origin_vio_T + origin_vio_R * tic;

//   R_inital = R_w_c.inverse();
//   P_inital = -(R_inital * T_w_c);

//   //cv::eigen2cv(R_inital, tmp_r);
//   for (int i = 0; i < 3; ++i) {
//     for (int j = 0; j < 3; ++j) {
//       tmp_r.at<double>(i, j) = R_inital(i, j);
//     }
//   }
//   cv::Rodrigues(tmp_r, rvec);
//   t.at<double>(0, 0) = P_inital[0];
//   t.at<double>(1, 0) = P_inital[1];
//   t.at<double>(2, 0) = P_inital[2];
//   //cv::eigen2cv(P_inital, t);
//   cv::Mat inliers;
//   double fx = mpMultiCamera->cameras_[img_id].K_(0, 0);
//   double fy = mpMultiCamera->cameras_[img_id].K_(1, 1);
//   double focal_length = std::min(fx, fy);
//   //rvec·相机在世界坐标系下的位姿作
//   bool success = cv::solvePnPRansac(matched_3d, matched_2d_old_norm, K, D, rvec, t, false, 100,
//                                     10.0 / focal_length, 0.99, inliers, cv::SOLVEPNP_EPNP);

//   for (int i = 0; i < (int)matched_2d_old_norm.size(); i++) {
//     status.push_back(0);
//   }
//   if (success && inliers.rows > 0) {
//     for (int i = 0; i < inliers.rows; i++) {
//       int n = inliers.at<int>(i);
//       status[n] = 1;
//     }

//     cv::Rodrigues(rvec, r);
//     Eigen::Matrix3d R_pnp, R_w_c_old;
//     // cv::cv2eigen(r, R_pnp);
//     for (int m = 0; m < 3; ++m) {
//       for (int n = 0; n < 3; ++n) {
//         R_pnp(m, n) = r.at<double>(m, n);
//       }
//     }
//     R_w_c_old = R_pnp.transpose();
//     Eigen::Vector3d T_pnp, T_w_c_old;
//     // cv::cv2eigen(t, T_pnp);
//     T_pnp << t.at<double>(0, 0), t.at<double>(1, 0), t.at<double>(2, 0);
//     T_w_c_old = R_w_c_old * (-T_pnp);

//     PnP_R_old = R_w_c_old * qic.transpose();
//     // WrWM = WrWC - qWM * MrMC
//     //      = WrWC - WrMC
//     //      = WrWC + WrCM
//     PnP_T_old = T_w_c_old - PnP_R_old * tic;//世界系下 世界到相机系的位移减去 世界系下 imu到相机系的位移
//     return true;
//   }
//   return false;
// }
// int computeBaseline(const std::map<double, FeatureInfo> &map_pixel_pose_,
//                     const double th_baseline) {
//   // const int MAX_HISTORY = 600;
//   const int MIN_HISTORY = 20;
//   int winSize = map_pixel_pose_.size();
//   if (winSize < MIN_HISTORY) {
//     return winSize - 1;
//   }
//   auto iter = map_pixel_pose_.begin();
//   const M3D &Raw = iter->second.R_wc.inverse();
//   const V3D &WrWA = iter->second.t_wc;  // Anchor Position
//   iter++;
//   double maxBaseline = 0.0;
//   for (int i = 1; i < winSize; i++) {
//     const M3D &Rwc = iter->second.R_wc;
//     const V3D &WrWC = iter->second.t_wc;
//     // Compute baseline
//     V3D ArCA = Raw * (WrWA - WrWC);
//     // project ArCA to orthogonal plane of 0 0 1
//     V3D ArCA_proj(ArCA.x(), ArCA.y(), 0);
//     double p = ArCA_proj.norm();
//     if (p > th_baseline) {
//       return i;
//     }
//     iter++;
//   }
//   // zzlogi("[%f] Baseline too short: %f", filterState.t_, maxBaseline);
//   return winSize - 1;
// }


// //寻找并建立关键帧与回环帧之间的匹配关系
// bool KeyFrame::findConnection_M(std::shared_ptr<KeyFrame> old_kf,const int image_idx) {
// #ifdef PC_VERSION
//   int match_num=0;
//   int tri_num=0;
//   int f_ransac_num=0;
//   int p_ransac_num=0;

//   double loop_mode_t = 338.225308 ; //h
//   double first_mode_t = 358.791975;
//   double end_mode_t = 391.558668;
//   double time_diff = end_mode_t - first_mode_t;
//   double flag_time = first_mode_t+time_diff*0.25;
// #endif
// int Top_N=80;

  
// #ifdef Looplog
//   zzTimer::tic("computeAndMatch");
// #endif
//   //旧关帧赋值
//   old_kf->pointsid=old_kf->m_pointsid[image_idx];
//   old_kf->keypoints=old_kf->m_keypoints[image_idx];
//   old_kf->keypoints_norm=old_kf->m_keypoints_norm[image_idx];
//   old_kf->image=old_kf->images_[image_idx];
//   old_kf->origin_vio_R_mc=old_kf->origin_vio_R_mc_map[image_idx];
//   old_kf->origin_vio_T_mc=old_kf->origin_vio_T_mc_map[image_idx];
//   //当前关键帧赋值
//   image=images_[cur_frame_idx];
//   keypoints=m_keypoints[cur_frame_idx];
//   keypoints_norm=m_keypoints_norm[cur_frame_idx];

//   std::vector<cv::Point2f> matched_2d_cur, matched_2d_old;
//   std::vector<cv::Point2f> matched_2d_cur_norm, matched_2d_old_norm;
//   std::vector<cv::Point3f> matched_3d;
//   std::vector<int> matched_id;
//   std::vector<uchar> status;
//   matched_id = old_kf->pointsid;
//   for (const auto &kp : old_kf->keypoints) {
//     matched_2d_old.push_back(kp.pt);
//   }
//   for (const auto &kp : old_kf->keypoints_norm) {
//     matched_2d_old_norm.push_back(kp.pt);
//   }

//   auto matcher = cv::BFMatcher::create(cv::NORM_HAMMING, false);
//   status.resize(old_kf->keypoints.size());
//   matched_2d_cur.resize(old_kf->keypoints.size());
//   matched_2d_cur_norm.resize(old_kf->keypoints.size());
//   std::vector<cv::DMatch> matches;
//   cv::Mat old_descriptors, cur_descriptors;

//   // old_descriptors=convertBriefDescriptors(old_kf->brief_descriptors);
//   // cur_descriptors=convertBriefDescriptors(brief_descriptors);
// #ifdef Looplog
//   zzTimer::tic("compute");
// #endif
//   cv::Ptr<TEBLID> teblid = TEBLID::create(6.25f, TEBLID::SIZE_512_BITS);
//   for (int i = 0; i < old_kf->keypoints.size(); i++) {
//     old_kf->keypoints[i].size = 31;
//   }
//   for (int i = 0; i < keypoints.size(); i++) {
//     keypoints[i].size = 31;
//   }
//   teblid->compute(old_kf->image, old_kf->keypoints, old_descriptors);
//   teblid->compute(image, keypoints, cur_descriptors);
// #ifdef Looplog
//   zzTimer::toc("compute");
// #endif
// #ifdef Looplog
//   zzTimer::tic("match");
// #endif
//   matcher->match(old_descriptors, cur_descriptors, matches);
// #ifdef Looplog
//   zzTimer::toc("match");
// #endif
//   zzlogi("old_kf keypoints size %d ,keyframe keypoints size %d , matches size %d", old_kf->keypoints.size(),
//          keypoints.size(), matches.size());

//   std::unordered_set<int> used_train_indices;
//   cv::Point2f pt(0.f, 0.f);
//   cv::Point2f pt_norm(0.f, 0.f);

//   // 1. 先按匹配距离升序排序
//   std::sort(matches.begin(), matches.end(),
//             [](const cv::DMatch& a, const cv::DMatch& b) {
//                 return a.distance < b.distance;
//             });

//   // 2. 只保留距离最小的前 80 个
//   if (matches.size() > 80) {
//       matches.resize(80);
//   }


//   for (int i = 0; i < matches.size(); i++) {
//       int queryIdx = matches[i].queryIdx;
//       int trainIdx = matches[i].trainIdx;

//       if (matches[i].distance < 120 &&
//           used_train_indices.find(trainIdx) == used_train_indices.end()) {
//           status[queryIdx] = 1;
//           matched_2d_cur[queryIdx]      = keypoints[trainIdx].pt;
//           matched_2d_cur_norm[queryIdx] = keypoints_norm[trainIdx].pt;
//           used_train_indices.insert(trainIdx);
//       } else {
//           status[queryIdx]       = 0;
//           matched_2d_cur[queryIdx]      = pt;
//           matched_2d_cur_norm[queryIdx] = pt_norm;
//       }
//   }

//   used_train_indices.clear();
//   if (matched_2d_old.size() == 0) {
//     return false;
//   }
//   reduceVector(matched_2d_cur, status);
//   reduceVector(matched_2d_old, status);
//   reduceVector(matched_2d_cur_norm, status);
//   reduceVector(matched_2d_old_norm, status);
//   reduceVector(matched_3d, status);
//   reduceVector(matched_id, status);
//   match_num=std::count(status.begin(), status.end(), 1);
//   // status.clear();
// #ifdef PC_VERSION
//   std::vector<cv::Point2f> matched_2d_old_in;
//   for(auto m:matched_2d_old){
//     matched_2d_old_in.push_back(cv::Point2f(m.x, m.y));
//   }
// #endif

// #ifdef PC_VERSION
//   int gap = 10;
//   int ROW = image.rows;
//   int COL = image.cols;
//   cv::Mat gap_image(ROW, gap, CV_8UC1, cv::Scalar(255, 255, 255));
//   cv::Mat gray_img, loop_match_img;
//   cv::Mat old_img = old_kf->image;
//   cv::hconcat(image, gap_image, gap_image);
//   cv::hconcat(gap_image, old_img, gray_img);
//   cvtColor(gray_img, loop_match_img, CV_GRAY2RGB);
//   for (int i = 0; i < (int)matched_2d_cur.size(); i++) {
//     cv::Point2f cur_pt = matched_2d_cur[i];
//     cv::circle(loop_match_img, cur_pt, 5, cv::Scalar(0, 255, 0));
//   }
//   for (int i = 0; i < (int)old_kf->keypoints.size(); i++) {
//     cv::KeyPoint key_pt = old_kf->keypoints[i];
//     key_pt.pt.x += (COL + gap);
//     if(status[i]==1){
//       cv::circle(loop_match_img, key_pt.pt, 5, cv::Scalar(0, 255, 0));
//     }else{
//       cv::circle(loop_match_img, key_pt.pt, 5, cv::Scalar(255, 0, 0));
//     }
//   }
//   for (int i = 0; i < (int)matched_2d_cur.size(); i++) {
//     cv::Point2f old_pt = matched_2d_old[i];
//     old_pt.x += (COL + gap);
//     cv::line(loop_match_img, matched_2d_cur[i], old_pt, cv::Scalar(0, 255, 0), 1, 8, 0);

//   }
//     cv::Mat notation(50, COL + gap + COL, CV_8UC3, cv::Scalar(255, 255, 255));
//     cv::putText(notation, "current frame: " + to_string(time_stamp), cv::Point2f(20, 30),
//                 CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255), 1.5);
//     cv::putText(notation,
//                 "previous frame: " + to_string(old_kf->time_stamp) +
//                     " : " + to_string((int)matched_2d_cur.size()),
//                 cv::Point2f(20 + COL, 30), CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255), 1.5);
//     cv::vconcat(notation, loop_match_img, loop_match_img);
//     cv::imwrite(
//     "./matchimages/" + std::to_string(time_stamp) + "_"+std::to_string(MaxPoints)+".png",
//     loop_match_img);
// #endif
//   status.clear();

// #ifdef Looplog
//   zzTimer::toc("computeAndMatch");
// #endif
// #if 1
// #ifdef Looplog
//   zzTimer::tic("FundmantalMatrixRANSAC");
// #endif
//   FundmantalMatrixRANSAC(matched_2d_cur_norm, matched_2d_old_norm, status,image_idx);
//   f_ransac_num=std::count(status.begin(), status.end(), 1);
//   reduceVector(matched_2d_cur, status);
//   reduceVector(matched_2d_old, status);
//   reduceVector(matched_2d_cur_norm, status);
//   reduceVector(matched_2d_old_norm, status);
//   reduceVector(matched_3d, status);
//   reduceVector(matched_id, status);
//   mpPoseGraph->matchptsCount += status.size();
//   mpPoseGraph->fundCount += matched_2d_cur.size();
// #ifdef Looplog
//   zzTimer::toc("FundmantalMatrixRANSAC");
// #endif
//   // status.clear();
// #endif
// {
// #ifdef PC_VERSION
//   int gap = 10;
//   int ROW = image.rows;
//   int COL = image.cols;
//   cv::Mat gap_image(ROW, gap, CV_8UC1, cv::Scalar(255, 255, 255));
//   cv::Mat gray_img, loop_match_img;
//   cv::Mat old_img = old_kf->image;
//   cv::hconcat(image, gap_image, gap_image);
//   cv::hconcat(gap_image, old_img, gray_img);
//   cvtColor(gray_img, loop_match_img, CV_GRAY2RGB);
//   for (int i = 0; i < (int)matched_2d_cur.size(); i++) {
//     cv::Point2f cur_pt = matched_2d_cur[i];
//     cv::circle(loop_match_img, cur_pt, 5, cv::Scalar(0, 255, 0));
//   }
//   // 1. 全部旧点 → 蓝色
//   for (auto &kp : matched_2d_old_in) {
//       cv::Point2f pt = kp;
//       pt.x += (COL + gap);
//       cv::circle(loop_match_img, pt, 5, cv::Scalar(255, 0, 0));
//   }

//   for (auto &pt : matched_2d_old) {
//       cv::Point2f old_pt = pt;
//       old_pt.x += (COL + gap);
//       cv::circle(loop_match_img, old_pt, 5, cv::Scalar(0, 255, 0));
//   }
//   // for (int i = 0; i < (int)old_kf->keypoints.size(); i++) {
//   //   cv::KeyPoint key_pt = old_kf->keypoints[i];
//   //   key_pt.pt.x += (COL + gap);
//   //   if(status[i]==1){
//   //     cv::circle(loop_match_img, key_pt.pt, 5, cv::Scalar(0, 255, 0));
//   //   }else{
//   //     cv::circle(loop_match_img, key_pt.pt, 5, cv::Scalar(255, 0, 0));
//   //   }
//   //   // cv::circle(loop_match_img, key_pt.pt, 5, cv::Scalar(0, 255, 0));
//   // }
//   for (int i = 0; i < (int)matched_2d_cur.size(); i++) {
//     cv::Point2f old_pt = matched_2d_old[i];
//     old_pt.x += (COL + gap);
//     cv::line(loop_match_img, matched_2d_cur[i], old_pt, cv::Scalar(0, 255, 0), 1, 8, 0);

//   }
//     cv::Mat notation(50, COL + gap + COL, CV_8UC3, cv::Scalar(255, 255, 255));
//     cv::putText(notation, "current frame: " + to_string(time_stamp), cv::Point2f(20, 30),
//                 CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255), 1.5);
//     cv::putText(notation,
//                 "previous frame: " + to_string(old_kf->time_stamp) +
//                     " : " + to_string((int)matched_2d_cur.size()),
//                 cv::Point2f(20 + COL, 30), CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255), 1.5);
//     cv::vconcat(notation, loop_match_img, loop_match_img);
//     cv::imwrite(
//     "./Fimages/" + std::to_string(time_stamp) + "_"+std::to_string(MaxPoints)+".png",
//     loop_match_img);
// #endif
// }
//   status.clear();

//   // #ifdef TRIANGULATION

//   if (mpPoseGraph) {
//     for (int i = 0; i < matched_id.size(); i++) {
//       LandMarkInfo info;
//       if (mpPoseGraph->getLandMarkInfo(
//               matched_id[i],
//               info))  //遍历回环帧的特征点，如果有特征点有足够的观测，则准备三角化。（为什么不使用vio的点深度呢）
//       {
//         if (!info.alreadysampled) {
//           SampleFeatureMap(info.map_pixel_pose_);
//           info.alreadysampled = true;
//         }
//         Eigen::Vector3d pt_in_cur;
//         if (info.alreadyTri_) {
//           if (info.success_) {
//             matched_3d.push_back(
//                 cv::Point3f(info.pt_word_tri_[0], info.pt_word_tri_[1], info.pt_word_tri_[2]));
//             status.push_back(1);
//           } else {
//             status.push_back(0);
//             matched_3d.push_back(cv::Point3f(0, 0, 0));
//           }

//         } else if (linearTriangulation(info.map_pixel_pose_, pt_in_cur)  //如果三角化了就别三角化了
//                    && gaussnewtonTriangulation(info.map_pixel_pose_, pt_in_cur)) {

//           Eigen::Vector3d pt_w = std::prev(info.map_pixel_pose_.end())->second.R_wc * pt_in_cur +
//                                  std::prev(info.map_pixel_pose_.end())->second.t_wc;
//           // zzlogi("x:%f, y:%f, z:%f",pt_w[0], pt_w[1], pt_w[2]);
//           matched_3d.push_back(cv::Point3f(pt_w[0], pt_w[1], pt_w[2]));
//           // matched_3d[i] = cv::Point3f(pt_w[0], pt_w[1], pt_w[2]);
//           mpPoseGraph->setLandMarkStatus(matched_id[i], true, true, pt_w);
//           status.push_back(1);
//         } else {
//           mpPoseGraph->setLandMarkStatus(matched_id[i], true);
//           status.push_back(0);
//           matched_3d.push_back(cv::Point3f(0, 0, 0));
//         }
//       } else {
//         matched_3d.push_back(cv::Point3f(0, 0, 0));
//         status.push_back(0);
//       }
//     }
//     tri_num=std::count(status.begin(), status.end(), 1);
//     reduceVector(matched_2d_cur, status);
//     reduceVector(matched_2d_old, status);
//     reduceVector(matched_2d_cur_norm, status);
//     reduceVector(matched_2d_old_norm, status);
//     reduceVector(matched_3d, status);
//     reduceVector(matched_id, status);
//     mpPoseGraph->triCount += matched_2d_cur.size();
// #ifdef PC_VERSION
//     int gap = 10;
//     int ROW = image.rows;
//     int COL = image.cols;
//     cv::Mat gap_image(ROW, gap, CV_8UC1, cv::Scalar(255, 255, 255));
//     cv::Mat gray_img, loop_match_img;
//     cv::Mat old_img = old_kf->image;
//     cv::hconcat(image, gap_image, gap_image);
//     cv::hconcat(gap_image, old_img, gray_img);
//     cvtColor(gray_img, loop_match_img, CV_GRAY2RGB);
//     for (int i = 0; i < (int)matched_2d_cur.size(); i++) {
//       cv::Point2f cur_pt = matched_2d_cur[i];
//       cv::circle(loop_match_img, cur_pt, 5, cv::Scalar(0, 255, 0));
//     }
//     for (int i = 0; i < (int)matched_2d_old.size(); i++) {
//       cv::Point2f old_pt = matched_2d_old[i];
//       old_pt.x += (COL + gap);
//       cv::circle(loop_match_img, old_pt, 5, cv::Scalar(0, 255, 0));
//     }
//     for (int i = 0; i < (int)matched_2d_cur.size(); i++) {
//       cv::Point2f old_pt = matched_2d_old[i];
//       old_pt.x += (COL + gap);
//       cv::line(loop_match_img, matched_2d_cur[i], old_pt, cv::Scalar(0, 255, 0), 1, 8, 0);
//     }
//     for (int i = 0; i < (int)matched_2d_old.size(); i++) {
//       cv::Point2f old_pt = matched_2d_old[i];
//       old_pt.x += (COL + gap);
//       Eigen::Vector3d pt_w{double(matched_3d[i].x), double(matched_3d[i].y),
//                            double(matched_3d[i].z)};
//       Eigen::Vector3d pt_c = (old_kf->vio_R_w_i * old_kf->origin_vio_R_mc).inverse() *
//                              (pt_w - (old_kf->origin_vio_T + old_kf->origin_vio_T_mc));
//       cv::putText(loop_match_img, "d:" + to_string(pt_c.z()), old_pt, CV_FONT_HERSHEY_SIMPLEX, 0.5,
//                   cv::Scalar(0, 255, 0), 1);
//     }
//     cv::Mat notation(50, COL + gap + COL, CV_8UC3, cv::Scalar(255, 255, 255));
//     cv::putText(notation, "current frame: " + to_string(time_stamp), cv::Point2f(20, 30),
//                 CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255), 1.5);
//     cv::putText(notation,
//                 "previous frame: " + to_string(old_kf->time_stamp) +
//                     " : " + to_string((int)matched_2d_cur.size()),
//                 cv::Point2f(20 + COL, 30), CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255), 1.5);
//     cv::vconcat(notation, loop_match_img, loop_match_img);
//     cv::imwrite("./triimages/"+std::to_string(time_stamp)+"_"+std::to_string(MaxPoints)+".png",loop_match_img);
// #endif
//   }
//   // #endif

//   // #if 0

//   // #endif

//   Eigen::Vector3d PnP_T_old; // WrWM
//   Eigen::Matrix3d PnP_R_old;
//   Eigen::Quaterniond pnp_q;
//   double relative_yaw;

//   //若达到最小回环匹配点数
//   //后续可能被pnp全剃掉
//   bool pnp_success = false;
//   if ((int)matched_2d_cur.size() > MIN_LOOP_NUM) {
//     // RANSAC PnP检测去除误匹配的点
//     status.clear();
//     pnp_success = PnPRANSAC(matched_2d_cur_norm, matched_3d, status, PnP_T_old,
//                             PnP_R_old,image_idx);  //返回被优化后的位姿
// #ifdef PC_VERSION
//     if(time_stamp<flag_time && time_stamp>loop_mode_t){
//       //vio

//       Eigen::Matrix3d R_w_cam_cur_vio = vio_R_w_i * origin_vio_R_mc;
//       const auto& WrWM = origin_vio_T;
//       const auto& MrMC = origin_vio_T_mc;
//       const auto& qWM = vio_R_w_i;
//       // WrWM + WrMC = WrWC
//       Eigen::Vector3d WrWC = WrWM + qWM * MrMC; // WrWC

//       //pnp
//       Eigen::Matrix3d R_w_cur_pnp = PnP_R_old * origin_vio_R_mc; // 当前相机旋转
//       Eigen::Vector3d t_w_cur_pnp = PnP_T_old + PnP_R_old * MrMC; // 世界系下 世界到imu系的位移加上 世界系下 imu到相机系的位移

//       // Eigen::Matrix3d R_dif = R_w_cur_pnp.transpose() * R_w_cam_cur_vio;

//       // double x=Utility::normalizeAngle(Utility::R2ypr(R_w_cur_pnp).x() - Utility::R2ypr(R_w_cam_cur_vio).x());
//       // double y=Utility::normalizeAngle(Utility::R2ypr(PnP_R_old).x() - Utility::R2ypr(vio_R_w_i).x());
//       // Eigen::Vector3d ypr_dif = Utility::R2ypr(R_dif);
//       // zzlogi("===cam2world yaw: %f, ===imu2w:%f ,ypr_diff:%f,yaw_diff_deg:%f",x,y,ypr_dif.x(),Utility::normalizeAngle(ypr_dif.x()));
//       // --- debug snippet ---
//       auto deg = [](double rad){ return rad * 180.0 / M_PI; };
//       auto rad = [](double deg){ return deg * M_PI / 180.0; };

//       Eigen::Matrix3d R_vio = R_w_cam_cur_vio;   // world <- cam (你的变量)
//       Eigen::Matrix3d R_pnp = R_w_cur_pnp;

//       Eigen::Matrix3d R_dif = R_pnp * R_vio.transpose(); //  vio -> pnp

//       // 1) 打印旋转矩阵
//       zzlogi("R_vio:\n%f %f %f\n%f %f %f\n%f %f %f",
//             R_vio(0,0),R_vio(0,1),R_vio(0,2),
//             R_vio(1,0),R_vio(1,1),R_vio(1,2),
//             R_vio(2,0),R_vio(2,1),R_vio(2,2));
//       zzlogi("R_pnp:\n%f %f %f\n%f %f %f\n%f %f %f",
//             R_pnp(0,0),R_pnp(0,1),R_pnp(0,2),
//             R_pnp(1,0),R_pnp(1,1),R_pnp(1,2),
//             R_pnp(2,0),R_pnp(2,1),R_pnp(2,2));
//       zzlogi("R_dif:\n%f %f %f\n%f %f %f\n%f %f %f",
//             R_dif(0,0),R_dif(0,1),R_dif(0,2),
//             R_dif(1,0),R_dif(1,1),R_dif(1,2),
//             R_dif(2,0),R_dif(2,1),R_dif(2,2));

//       // 2) 矩阵直接提取 yaw（推荐）
//       double yaw_dif_rad = std::atan2(R_dif(1,0), R_dif(0,0));
//       double yaw_dif_deg = deg(yaw_dif_rad);

//       // 3) 用角轴表示整体旋转量（查看旋转大小 & 轴）
//       Eigen::AngleAxisd aa(R_dif);
//       double aa_angle_deg = deg(aa.angle());
//       Eigen::Vector3d aa_axis = aa.axis();

//       // 4) 用 Utility::R2ypr 分解（如果 R2ypr 返回的是 (yaw,pitch,roll)）
//       Eigen::Vector3d ypr_vio = Utility::R2ypr(R_vio);
//       Eigen::Vector3d ypr_pnp = Utility::R2ypr(R_pnp);
//       Eigen::Vector3d ypr_dif_from_func = Utility::R2ypr(R_dif);

//       // 5) 直接相减比较（注意单位）
//       double yaw_diff_direct = Utility::normalizeAngle(ypr_pnp.x() - ypr_vio.x()); // 你原来用的
//       double yaw_diff_imu = Utility::normalizeAngle(Utility::R2ypr(PnP_R_old).x() - Utility::R2ypr(vio_R_w_i).x());

//       // 打印对比
//       zzlogi("yaw_dif_deg (atan2 from R_dif) = %.6f deg", yaw_dif_deg);
//       zzlogi("ypr_dif_from_func.x() = %.6f", ypr_dif_from_func.x());
//       zzlogi("yaw_diff_direct (ypr_pnp - ypr_vio) = %.6f", yaw_diff_direct);
//       zzlogi("yaw_diff_imu (PnP_R_old - vio_R_w_i) = %.6f", yaw_diff_imu);
//       zzlogi("AngleAxis angle = %.6f deg, axis = [%f, %f, %f]", aa_angle_deg, aa_axis.x(), aa_axis.y(), aa_axis.z());
//       zzlogi("ypr_vio = [%f, %f, %f], ypr_pnp = [%f, %f, %f]", ypr_vio.x(), ypr_vio.y(), ypr_vio.z(), ypr_pnp.x(), ypr_pnp.y(), ypr_pnp.z());


//       Eigen::Vector3d t_dif = t_w_cur_pnp - WrWC;

//       zzlogi("time: %f, t_dif x: %f, y: %f, z: %f,distance: %f,R_dif: %f",time_stamp,t_dif.x(),t_dif.y(),t_dif.z(),t_dif.norm(),Utility::R2ypr(R_dif).x());


//     }else{
//       zzlogi("flag time: %f, time: %f,loop_mode_t: %f",flag_time,time_stamp,loop_mode_t);
//     }
// #endif
//     reduceVector(matched_2d_cur, status);
//     reduceVector(matched_2d_old, status);
//     reduceVector(matched_2d_cur_norm, status);
//     reduceVector(matched_2d_old_norm, status);
//     reduceVector(matched_3d, status);
//     reduceVector(matched_id, status);
//     p_ransac_num=std::count(status.begin(), status.end(), 1);
// #ifdef PC_VERSION
//     int gap = 10;
//     int ROW = image.rows;
//     int COL = image.cols;
//     cv::Mat gap_image(ROW, gap, CV_8UC1, cv::Scalar(255, 255, 255));
//     cv::Mat gray_img, loop_match_img;
//     cv::Mat old_img = old_kf->image;
//     cv::hconcat(image, gap_image, gap_image);
//     cv::hconcat(gap_image, old_img, gray_img);

//     //灰度图gray_img转换成RGB图loop_match_img
//     cvtColor(gray_img, loop_match_img, CV_GRAY2RGB);
//     for (int i = 0; i < (int)matched_2d_cur.size(); i++) {
//       cv::Point2f cur_pt = matched_2d_cur[i];
//       cv::circle(loop_match_img, cur_pt, 5, cv::Scalar(0, 255, 0));
//     }
//     for (int i = 0; i < (int)matched_2d_old.size(); i++) {
//       cv::Point2f old_pt = matched_2d_old[i];
//       old_pt.x += (COL + gap);
//       cv::circle(loop_match_img, old_pt, 5, cv::Scalar(0, 255, 0));
//     }
//     for (int i = 0; i < (int)matched_2d_cur.size(); i++) {
//       cv::Point2f old_pt = matched_2d_old[i];
//       old_pt.x += (COL + gap);
//       cv::line(loop_match_img, matched_2d_cur[i], old_pt, cv::Scalar(0, 255, 0), 1, 8, 0);
//     }
//     for (int i = 0; i < (int)matched_2d_old.size(); i++) {
//       cv::Point2f old_pt = matched_2d_old[i];
//       old_pt.x += (COL + gap);
//       Eigen::Vector3d pt_w{double(matched_3d[i].x), double(matched_3d[i].y),
//                            double(matched_3d[i].z)};
//       Eigen::Vector3d pt_c = (old_kf->vio_R_w_i * old_kf->origin_vio_R_mc).inverse() *
//                              (pt_w - (old_kf->origin_vio_T + old_kf->origin_vio_T_mc));
//       cv::putText(loop_match_img, "d:" + to_string(pt_c.z()), old_pt, CV_FONT_HERSHEY_SIMPLEX, 0.5,
//                   cv::Scalar(0, 255, 0), 1);
//     }
//     cv::Mat notation(50, COL + gap + COL, CV_8UC3, cv::Scalar(255, 255, 255));
//     cv::putText(notation, "current frame: " + to_string(time_stamp), cv::Point2f(20, 30),
//                 CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255), 1.5);
//     cv::putText(notation,
//                 "previous frame: " + to_string(old_kf->time_stamp) +
//                     " : " + to_string((int)matched_2d_cur.size()),
//                 cv::Point2f(20 + COL, 30), CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255), 1.5);
//     cv::vconcat(notation, loop_match_img, loop_match_img);
//     cv::imwrite("./pnpimages/"+std::to_string(time_stamp)+"_"+std::to_string(MaxPoints)+".png",loop_match_img);
// #endif
//   }
// zzlogi("match num: %d, f_ransac_num: %d, pnp_ransac_num: %d, tri_num: %d",match_num,f_ransac_num,p_ransac_num,tri_num);
//   //若达到最小回环匹配点数
//   if ((int)matched_2d_cur.size() > MIN_LOOP_NUM && pnp_success) {
//     pnpCount = (int)matched_2d_cur.size();
//     mpPoseGraph->pnpCount += matched_2d_cur.size();

//     pnp_q = PnP_R_old;
//     relative_yaw =
//         Utility::normalizeAngle(Utility::R2ypr(PnP_R_old).x() - Utility::R2ypr(origin_vio_R).x());//？两次

//     //相对位姿检验
//     // if (abs(relative_yaw) < 30.0 && relative_t.norm() < 20.0)
//     {

//       has_loop = true;
//       loop_index = old_kf->index;
//       loop_info << PnP_T_old.x(), PnP_T_old.y(), PnP_T_old.z(), pnp_q.w(), pnp_q.x(),
//           pnp_q.y(), pnp_q.z(), relative_yaw;
//       zzlogi("KeyFrame loop relative t: x %f, y %f, z %f, norm %f", PnP_T_old.x(), PnP_T_old.y(),
//              PnP_T_old.z(), PnP_T_old.norm());
//       zzlogi("KeyFrame loop relative yaw: %f", relative_yaw);
//       return true;
//     }
//   }
//   // printf("loop final use num %d %lf--------------- \n", (int)matched_2d_cur.size(),
//   // t_match.toc());
//   return false;
// }

//格式转换成opencv
cv::Mat KeyFrame::convertBriefDescriptors(
    const std::vector<DVision::BRIEF::bitset> &briefDescriptors) {
  if (briefDescriptors.empty()) {
    return cv::Mat();
  }
  const int descriptorSize = briefDescriptors[0].size();
  const int byteSize = (descriptorSize + 7) / 8;
  cv::Mat descriptorsMat(briefDescriptors.size(), byteSize, CV_8U);
  for (size_t i = 0; i < briefDescriptors.size(); ++i) {
    const DVision::BRIEF::bitset &bitset = briefDescriptors[i];
    uchar *rowPtr = descriptorsMat.ptr<uchar>(i);
    for (int j = 0; j < byteSize; ++j) {
      uchar byte = 0;
      for (int k = 0; k < 8 && j * 8 + k < descriptorSize; ++k) {
        byte |= (bitset[j * 8 + k] << k);
      }
      rowPtr[j] = byte;
    }
  }
  return descriptorsMat;
}
//计算两个描述子之间的汉明距离
int KeyFrame::HammingDis(const DVision::BRIEF::bitset &a, const DVision::BRIEF::bitset &b) {
  DVision::BRIEF::bitset xor_of_bitset = a ^ b;
  int dis = xor_of_bitset.count();
  return dis;
}

void KeyFrame::getVioPose(Eigen::Vector3d &_T_w_i, Eigen::Matrix3d &_R_w_i) {
  _T_w_i = vio_T_w_i;
  _R_w_i = vio_R_w_i;
}

void KeyFrame::getPose(Eigen::Vector3d &_T_w_i, Eigen::Matrix3d &_R_w_i) {
  _T_w_i = T_w_i;
  _R_w_i = R_w_i;
}

void KeyFrame::updatePose(const Eigen::Vector3d &_T_w_i, const Eigen::Matrix3d &_R_w_i) {
  T_w_i = _T_w_i;
  R_w_i = _R_w_i;
}

void KeyFrame::updateVioPose(const Eigen::Vector3d &_T_w_i, const Eigen::Matrix3d &_R_w_i) {
  vio_T_w_i = _T_w_i;
  vio_R_w_i = _R_w_i;
  T_w_i = vio_T_w_i;
  R_w_i = vio_R_w_i;
}

Eigen::Vector3d KeyFrame::getLoopPNPT() {
  return Eigen::Vector3d(loop_info(0), loop_info(1), loop_info(2));
}

Eigen::Quaterniond KeyFrame::getLoopPNPQ() {
  return Eigen::Quaterniond(loop_info(3), loop_info(4), loop_info(5), loop_info(6));
}

double KeyFrame::getLoopRelativeYaw() {
  return loop_info(7);
}

}  // namespace rovio