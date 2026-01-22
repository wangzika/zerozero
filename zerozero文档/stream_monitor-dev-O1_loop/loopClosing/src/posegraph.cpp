#include "posegraph.hpp"

// #include "Turnback.hpp"
// #include "vioTrace.h"

namespace rovio {

PoseGraph::PoseGraph() {
  t_drift = Eigen::Vector3d(0, 0, 0);
  yaw_drift = 0;
  r_drift = Eigen::Matrix3d::Identity();
  w_t_vio = Eigen::Vector3d(0, 0, 0);
  w_r_vio = 0;
  global_index = 0;
  keyptsidx = 0;
  criteria = cv::TermCriteria((cv::TermCriteria::COUNT) + (cv::TermCriteria::EPS), 10, 0.03);
}

PoseGraph::~PoseGraph() {
  for (std::list<std::shared_ptr<KeyFrame>>::iterator itr = keyframelist.begin();
       itr != keyframelist.end(); itr++) {
    std::shared_ptr<KeyFrame> temp = (*itr);
    temp.reset();
  }
  keyframelist.clear();
  // delete voc;
  //voc = nullptr;
  int index = 0;
  for (const auto& ptr : keyframelist) {
    if (ptr.use_count() != 0) {
      ++index;
    }
  }
}

//加载Brief字典
void PoseGraph::InitialVocabulary(std::shared_ptr<const BriefVocabulary> voc) {
  voc_=voc;
  db.setVocabulary(voc, false, 0);
}

/**
 * @brief   添加关键帧，进行回环检测与闭环
 * @Description
 * @return      void
 */
bool PoseGraph::addKeyFrame(std::shared_ptr<KeyFrame> cur_kf) {
  
  // shift to base frame
  Eigen::Vector3d vio_P_cur;
  Eigen::Matrix3d vio_R_cur;

  //获取当前帧的位姿vio_P_cur、vio_R_cur
  cur_kf->getVioPose(vio_P_cur, vio_R_cur);
  cur_kf->index = global_index;
  global_index++;
  //添加当前关键帧到字典数据库中
  // TicToc t_add;
  // std::vector<cv::Point2f> tempPts;
  // tempPts.reserve(250);
  // std::vector<cv::KeyPoint> keypoints;
  // std::vector<DVision::BRIEF::bitset> brief_descriptors;
  // cv::goodFeaturesToTrack(cur_kf->image, tempPts, 250, 0.01, 10);
  // for (int i = 0; i < tempPts.size(); i++) {
  //   keypoints.emplace_back(tempPts[i], 1.0f);
  // }

  // (*cur_kf->pExtractor)(cur_kf->image, keypoints, brief_descriptors);
  db.add(cur_kf->brief_descriptors);
  keyframelist.push_back(cur_kf);

  return 0;
}
//添加所有关键帧(四张图片)到数据库中
bool PoseGraph::addallKeyFrame(std::shared_ptr<KeyFrame> cur_kf) {
  
  // shift to base frame
  Eigen::Vector3d vio_P_cur;
  Eigen::Matrix3d vio_R_cur;
  double time_stamp = cur_kf->time_stamp;
  double last_time_stamp = 0;
  cur_kf->getVioPose(vio_P_cur, vio_R_cur);
  cur_kf->index = global_index;
  global_index++;
  map_db[cur_kf->index].setVocabulary(voc_, false, 0);

  for (const auto& kv : cur_kf->vio_map_brief_descriptors) {
      const auto& vec = kv.second;  // 取出每个map键对应的vector
      map_db[cur_kf->index].add(kv.second);//将四张图片的描述子分别加入到数据库中
      cur_kf->all_brief_descriptors.insert(
          cur_kf->all_brief_descriptors.end(),  // 插入到末尾
          vec.begin(),
          vec.end()
      );
  }
  //   for (const auto &it:cur_kf->vio_map_brief_descriptors){
  //   // cur_kf->all_brief_descriptors=
  //   map_db[cur_kf->index][it.first].add(it.second);

  // }

  db.add(cur_kf->all_brief_descriptors);
  map_sequence_keyframe[cur_kf->index] = cur_kf;
  // addoneKeyFrame(cur_kf);
  return 0;
}


void reset(std::shared_ptr<KeyFrame> cur_kf)
{
  cur_kf->all_brief_descriptors.clear();  // 清空目标向量，避免重复追加
  cur_kf->vio_map_brief_descriptors.clear();
  // cur_kf->map_brief_descriptors.clear();
  

}

bool PoseGraph::addcheckKeyFrame(std::shared_ptr<KeyFrame> cur_kf) {
    std::cout << "addcheckKeyFrame:start" << std::endl;
    
    Eigen::Vector3d vio_P_cur;
    Eigen::Matrix3d vio_R_cur;
    cur_kf->getVioPose(vio_P_cur, vio_R_cur);
    cur_kf->index = global_index;
    global_index++;
    
    auto loop_res = detectLoopRefine(cur_kf, cur_kf->index);
    
    std::cout << "loop old_frame_id: " << loop_res.old_frame_id 
              << " loop old_img_id: " << loop_res.old_img_id 
              << " loop cur_frame_id: " << loop_res.cur_frame_id 
              << " loop cur_img_id: " << loop_res.cur_img_id << std::endl;

    // === 1. 保存四图拼接（2×4）并添加顶部文字 ===
    if (loop_res.old_frame_id != -1) {
        std::map<int, cv::Mat> cur_images = cur_kf->images_;
        std::map<int, cv::Mat> old_images = map_sequence_keyframe[loop_res.old_frame_id]->images_;

        for (int i = 0; i < 4; ++i) {
            if (cur_images.count(i) == 0 || old_images.count(i) == 0) {
                std::cerr << "Missing image with index " << i << " in keyframes!" << std::endl;
                return false;
            }
        }

        cv::Mat cur0 = cur_images.at(0);
        cv::Mat cur1 = cur_images.at(1);
        cv::Mat cur2 = cur_images.at(2);
        cv::Mat cur3 = cur_images.at(3);
        cv::Mat old0 = old_images.at(0);
        cv::Mat old1 = old_images.at(1);
        cv::Mat old2 = old_images.at(2);
        cv::Mat old3 = old_images.at(3);

        cv::Size target_size = cur0.size();
        auto resize_if_needed = [&](cv::Mat& img) {
            if (img.size() != target_size) {
                cv::resize(img, img, target_size);
            }
        };
        resize_if_needed(cur1); resize_if_needed(cur2); resize_if_needed(cur3);
        resize_if_needed(old0); resize_if_needed(old1); resize_if_needed(old2); resize_if_needed(old3);

        std::vector<cv::Mat> cur_row = {cur0, cur1, cur2, cur3};
        std::vector<cv::Mat> old_row = {old0, old1, old2, old3};
        cv::Mat top_row, bottom_row;
        cv::hconcat(cur_row, top_row);
        cv::hconcat(old_row, bottom_row);
        cv::Mat stitched_image;
        cv::vconcat(top_row, bottom_row, stitched_image);

        // ===== 添加顶部空白并绘制文字 =====
        std::string info_text = "Current: F" + std::to_string(loop_res.cur_frame_id) +
                                "/" + std::to_string(loop_res.cur_img_id) +
                                " | Loop: F" + std::to_string(loop_res.old_frame_id) +
                                "/" + std::to_string(loop_res.old_img_id) +
                                " | Time: " + std::to_string(cur_kf->time_stamp)+ " | Score: " + std::to_string(loop_res.frame_score);

        int font_face = cv::FONT_HERSHEY_SIMPLEX;
        double font_scale = 0.6;   // 字体较小
        int thickness = 1;
        cv::Scalar text_color(255, 255, 255);      // 白色文字
        cv::Scalar bg_color(20, 20, 20);           // 深灰背景

        // 计算文字尺寸
        int baseline = 0;
        cv::Size text_size = cv::getTextSize(info_text, font_face, font_scale, thickness, &baseline);
        int top_margin = text_size.height + baseline + 16; // 留出上下边距

        // 创建带顶部空白的新图像
        cv::Mat final_image(stitched_image.rows + top_margin, stitched_image.cols, stitched_image.type(), bg_color);
        stitched_image.copyTo(final_image(cv::Rect(0, top_margin, stitched_image.cols, stitched_image.rows)));

        // 文字位置：左对齐，垂直居中于顶部空白区
        cv::Point text_org(10, top_margin - (top_margin - text_size.height) / 2 - 2);
        cv::putText(final_image, info_text, text_org, font_face, font_scale, text_color, thickness);

        // 保存
        std::string filename = "./frames/" + std::to_string(cur_kf->time_stamp) + 
                               "_cur" + std::to_string(loop_res.cur_frame_id) + 
                               "_" + std::to_string(loop_res.cur_img_id) +
                               "_old" + std::to_string(loop_res.old_frame_id) + 
                               "_" + std::to_string(loop_res.old_img_id) + ".png";
        cv::imwrite(filename, final_image);
    }

    // === 2. 保存单对图像拼接（垂直）并添加顶部文字 ===
    if (loop_res.old_img_id != -1) {
        cv::Mat cur_img = cur_kf->images_[loop_res.cur_img_id];
        cv::Mat old_img = map_sequence_keyframe[loop_res.old_frame_id]->images_[loop_res.old_img_id];
        if (cur_img.empty() || old_img.empty()) {
            std::cerr << "2 Missing images in keyframes!" << std::endl;
            return false;
        }

        if (old_img.size() != cur_img.size()) {
            cv::resize(old_img, old_img, cur_img.size());
        }

        cv::Mat stitched_image;
        cv::vconcat(cur_img, old_img, stitched_image);

        // ===== 添加顶部空白并绘制文字 =====
        std::string info_text = "Current: F" + std::to_string(loop_res.cur_frame_id) + 
                                "/" + std::to_string(loop_res.cur_img_id) +
                                "  vs  Loop: F" + std::to_string(loop_res.old_frame_id) + 
                                "/" + std::to_string(loop_res.old_img_id)+ " | " + std::to_string(loop_res.img_score) ;

        int font_face = cv::FONT_HERSHEY_SIMPLEX;
        double font_scale = 0.3;
        int thickness = 1;
        cv::Scalar text_color(255, 255, 255);
        cv::Scalar bg_color(20, 20, 20);

        int baseline = 0;
        cv::Size text_size = cv::getTextSize(info_text, font_face, font_scale, thickness, &baseline);
        int top_margin = text_size.height + baseline + 16;

        cv::Mat final_image(stitched_image.rows + top_margin, stitched_image.cols, stitched_image.type(), bg_color);
        stitched_image.copyTo(final_image(cv::Rect(0, top_margin, stitched_image.cols, stitched_image.rows)));

        cv::Point text_org(10, top_margin - (top_margin - text_size.height) / 2 - 2);
        cv::putText(final_image, info_text, text_org, font_face, font_scale, text_color, thickness);

        std::string filename = "./images/" + std::to_string(cur_kf->time_stamp) + 
                               "_cur" + std::to_string(loop_res.cur_frame_id) + 
                               "_" + std::to_string(loop_res.cur_img_id) +
                               "_old" + std::to_string(loop_res.old_frame_id) + 
                               "_" + std::to_string(loop_res.old_img_id) + ".png";
        cv::imwrite(filename, final_image);

        bool isMatch = true; // 实际应进行 PnP 验证
        if (isMatch) {
            return isMatch;
        }
    }

    return false;
}



//返回索引为index的关键帧
std::shared_ptr<KeyFrame> PoseGraph::getKeyFrame(int index) {
  std::list<std::shared_ptr<KeyFrame>>::iterator it = keyframelist.begin();
  for (; it != keyframelist.end(); it++) {
    if ((*it)->index == index) {
      break;
    }
  }
  if (it != keyframelist.end()) {
    return *it;
  } else {
    return NULL;
  }
}

//返回索引为index的关键帧
std::shared_ptr<KeyFrame> PoseGraph::getKeyFrame(LoopResult index) {
  auto &keyframe = map_sequence_keyframe[index.old_frame_id];
  if(keyframe){
    return keyframe;
  }else{
    return NULL;
  }

}

//回环检测
int PoseGraph::detectLoop(std::shared_ptr<KeyFrame> keyframe, int frame_index) {
  // put image into image_pool; for visualization
  // TicToc tmp_t;
  DBoW2::QueryResults ret;
  // TicToc t_query;

  //查询字典数据库，得到与每一帧的相似度评分ret
  //两步骤，第一步在addKeyFrame中已经将当前帧的描述子存入字典数据库
  for(auto &it: keyframe->vio_map_brief_descriptors)
  {
    keyframe->all_brief_descriptors.insert(
        keyframe->all_brief_descriptors.end(),  // 插入到末尾
        it.second.begin(),
        it.second.end()
    );

  }
  db.query(keyframe->all_brief_descriptors, ret, 10, frame_index - 10);  //最后一维是最大搜索idx

  DBoW2::QueryResults ret_frame;

  if(ret[0].Score>0.015){
    map_db[ret[0].Id].query(keyframe->all_brief_descriptors, ret_frame, 10, frame_index - 10);  //最后一维是最大搜索idx
  }else{
    return -1;
  }

  //第二步在这里进行查询，找出最相似的关键帧
  // printf("query time: %f", t_query.toc());
  // cout << "Searching for Image " << frame_index << ". " << ret << endl;

  // ret[0] is the nearest neighbour's score. threshold change with neighour score
  bool find_loop = false;
  if (ret_frame[0].Score > 0.015) {
    find_loop = true;
  }
  //返回评分大于0.015最相似的关键帧
  if (find_loop) {
    loopcount++;
    return ret_frame[0].Id;
  } else
    return -1;
}

//回环检测
PoseGraph::LoopResult PoseGraph::detectLoopRefine(std::shared_ptr<KeyFrame> keyframe, int frame_index) {
  // put image into image_pool; for visualization
  // TicToc tmp_t;
  std::cout << "detectLoopRefine" << std::endl;
  DBoW2::QueryResults ret;
  // TicToc t_query;
  LoopResult loop_result;

  //查询字典数据库，得到与每一帧的相似度评分ret
  //两步骤，第一步在addKeyFrame中已经将当前帧的描述子存入字典数据库
  // std::cout<<"vio_map_brief_descriptors.size(): "<<keyframe->vio_map_brief_descriptors.size()<<std::endl;

  for(auto &it: keyframe->vio_map_brief_descriptors)
  {
    // std::cout<<"it.second.size(): "<<it.second.size()<<std::endl;
    keyframe->all_brief_descriptors.insert(
        keyframe->all_brief_descriptors.end(),  // 插入到末尾
        it.second.begin(),
        it.second.end()
    );

  }
  db.query(keyframe->all_brief_descriptors, ret, 10, frame_index - 10);  //最后一维是最大搜索idx
  std::cout<<"ret[0]: "<<ret[0].Score<<std::endl;
  std::map<int,DBoW2::QueryResults> ret_frame;
  // std::cout<<"1"<<std::endl;
  int cur_max_id=-1;
  if(ret[0].Score>0.010){
    //  std::cout<<"2"<<std::endl;

    for(auto &it:keyframe->vio_map_brief_descriptors){
      // std::cout<<"3"<<std::endl;

      map_db[ret[0].Id].query(it.second, ret_frame[it.first], 1, frame_index - 1);  //最后一维是最大搜索idx
    }
    double maxscore=0;

    for(auto &it:ret_frame){
      if (!it.second.empty() && it.second[0].Score > maxscore) {
          maxscore = it.second[0].Score;
          cur_max_id = it.first;
      }
    }
  }else{
    std::cout<<"ret[0].Score<0.010"<<std::endl;
    // loop_result.old_frame_id = ret[0].Id;
    // loop_result.cur_frame_id=frame_index;

    return loop_result;
  }

  //第二步在这里进行查询，找出最相似的关键帧
  bool find_loop = false;
  if(cur_max_id!=-1){
    std::cout<<"cur_max_id: "<<cur_max_id<<std::endl;
    if ( ret_frame[cur_max_id][0].Score > 0.010) {
      find_loop = true;
      loop_result.old_img_id = ret_frame[cur_max_id][0].Id;
      loop_result.cur_img_id=cur_max_id;
      loop_result.img_score=ret_frame[cur_max_id][0].Score;
    }

  }

  loop_result.old_frame_id = ret[0].Id;
  loop_result.cur_frame_id=frame_index;
  loop_result.frame_score=ret[0].Score;

  //返回评分大于0.015最相似的关键帧
  if (find_loop) {
    loopcount++;
    return loop_result;
  } else
    return loop_result;
}


//将当前帧的描述子存入字典数据库
void PoseGraph::addKeyFrameIntoVoc(std::shared_ptr<KeyFrame> keyframe) {
  // put image into image_pool; for visualization
  cv::Mat compressed_image;
  db.add(keyframe->brief_descriptors);
}

}  // namespace rovio