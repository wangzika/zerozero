#pragma once

#include "DBoW2.h"
#include "DVision.h"
#include "TemplatedDatabase.h"
#include "TemplatedVocabulary.h"
#include "keyframe.hpp"
// #include "rovio/FeatureManager.hpp"
// #include "rovio/MultiCamera.hpp"
#include "utility.h"

#include <assert.h>
#include <Eigen/Dense>
#include <map>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <queue>
#include <stdio.h>
#include <string>
#include <thread>

namespace rovio {

/**
 * @class PoseGraph
 * @Description 构建位姿图
 */
class PoseGraph {
 public:
  PoseGraph();
  ~PoseGraph();

  // MultiCamera* mpMultiCamera;
  // vio ----> cur(闭环后的)
  Eigen::Vector3d t_drift;
  double yaw_drift;
  Eigen::Matrix3d r_drift;
  int TP = 0;
  int FP = 0;
  int FN = 0;
  int TN = 0;
  float precision;
  float recall;
  bool ismatch = false;
  cv::TermCriteria criteria;
  int loopcount = 0;
  int matchptsCount = 0;
  int fundCount = 0;
  int triCount = 0;
  int pnpCount = 0;
  int bigchange = 0;
  // cur sequence frame  ---->  world frame( base sequence or first sequence)
  Eigen::Vector3d w_t_vio = Eigen::Vector3d::Zero();
  double w_r_vio = 0;

  std::shared_ptr<KeyFrame> lastkeyframe = nullptr;
  std::list<std::shared_ptr<KeyFrame>> keyframelist;
  std::unordered_map<int,std::shared_ptr<KeyFrame>> map_sequence_keyframe;//存储每个序列的关键帧列表

  int global_index;
  int img_global_index;
  int loop_index=-1;
  int curframe_idx=-1;
  struct LoopResult {
    int old_frame_id = -1;  // 帧号
    int old_img_id = -1;  // 图号
    int cur_frame_id=-1; //当前帧的帧号
    int cur_img_id=-1; //当前帧的图号
    double frame_score = 0;  // 匹配得分
    double img_score = 0;  // 图匹配得分
    // bool is_valid() const { return refined_id != -1; }
  };


  BriefDatabase db;
  std::unordered_map<int, BriefDatabase> map_db;
  int keyptsidx;

  std::shared_ptr<KeyFrame> getKeyFrame(int index);
  std::shared_ptr<KeyFrame> getKeyFrame(LoopResult loop_res);
  bool addKeyFrame(std::shared_ptr<KeyFrame> cur_kf);
  void reset(std::shared_ptr<KeyFrame> cur_kf);
  bool addcheckKeyFrame(std::shared_ptr<KeyFrame> cur_kf);
  bool addoneKeyFrame(std::shared_ptr<KeyFrame> cur_kf);
  bool addallKeyFrame(std::shared_ptr<KeyFrame> cur_kf);

  std::shared_ptr<const BriefVocabulary> voc_;
  void InitialVocabulary(std::shared_ptr<const BriefVocabulary> voc);

  void updateKeyFrameLoop(int index, Eigen::Matrix<double, 8, 1>& _loop_info);
  int detectLoop(std::shared_ptr<KeyFrame> keyframe, int frame_index);
  LoopResult detectLoopRefine(std::shared_ptr<KeyFrame> keyframe, int frame_index);
  void addKeyFrameIntoVoc(std::shared_ptr<KeyFrame> keyframe);



};

}  // namespace rovio