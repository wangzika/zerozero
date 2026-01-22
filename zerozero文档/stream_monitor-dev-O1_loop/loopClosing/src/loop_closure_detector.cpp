#include "loop_closure_detector.hpp"
// #include "lightweight_filtering/common.hpp"
// #include "zzTimer.hpp"

// #include "vioTrace.h"
#include <iostream>
#include <opencv2/video/tracking.hpp>
namespace rovio {
LoopClosureDetector::LoopClosureDetector() {

}

LoopClosureDetector::~LoopClosureDetector() {
  if (loopthread.joinable()) {
    loopthread.join();
  }
  std::cout << "LoopClosureDetector free"<<std::endl;
}
void LoopClosureDetector::initialize(std::shared_ptr<const BriefVocabulary> voc,
                                     std::vector<int> &pattern_x1, std::vector<int> &pattern_y1,
                                     std::vector<int> &pattern_x2, std::vector<int> &pattern_y2) {
  poseGraph_.InitialVocabulary(voc);
  briefExtractor_.InitialPattern(pattern_x1,pattern_y1,pattern_x2,pattern_y2);
}
void LoopClosureDetector::stop_LoopClosure() {
  {
    std::unique_lock<std::mutex> lock(mutexLoopClosing_);
    stop_loop = true;
  }
  loopcv_.notify_all();
}
void LoopClosureDetector::LoopClosure() {
  while (1) {
    
    std::unique_lock<std::mutex> lclock(mutexLoopClosing_);
    // std::cout << "LoopClosureDetector wait" << std::endl;
    // loopcv_.wait(lclock);
    loopcv_.wait(lclock, [this] { return stop_loop || accept_loop ; });
    if (stop_loop) {
      printf("LOOP Condition variable thread terminated\n");
      // std::cout << "LoopClosureDetector notify_num"<< notify_num<<"  "<Fnotify_num<<std::endl;
      run_loop_closure.store(false);
      break;
    }
    // std::cout<<"LoopClosureDetector++"<<std::endl;
    std::map<int,cv::Mat> images = poseInfo_O1.images;
    Eigen::Quaterniond qwm = poseInfo_O1.quaternion;//imu->world, in world
    std::map<int,Eigen::Quaterniond> qcm = poseInfo_O1.qCM;//imu->camera in camera
    Eigen::Vector3d WrWM = poseInfo_O1.translation;//word->imu in world
    std::map<int,Eigen::Vector3d> MrMC = poseInfo_O1.MrMC;//imu->camera in imu
    double t = poseInfo_O1.timestamp;
    Fnotify_num++;
    lclock.unlock();
    printf("accept_loop: %d, notify_num %d, Fnotify_num %d \n", static_cast<int>(accept_loop), notify_num, Fnotify_num);
    // std::cout<<"WrWM: "<<WrWM.transpose()<<std::endl;
    // Eigen::Vector3d twm_c = w_r_vio * WrWM + w_t_vio; //这里为什么只乘一部分的旋转偏移？
    Eigen::Vector3d twm_c = WrWM + w_t_vio; //这里为什么只乘一部分的旋转偏移？
    std::shared_ptr<KeyFrame> pkeyframe =
        std::make_shared<KeyFrame>(images);
    bool isempty =pkeyframe->isImgEmpty();

    pkeyframe->init(&briefExtractor_, t, qcm, MrMC, qwm, WrWM, &poseGraph_);
    // std::cout<<"twm_c: ======="<<twm_c.norm()<<std::endl;
    if (lastKeyFrame_ == nullptr) {
      if (twm_c.norm() > 0.005)  //可以考虑是否换成DSO那样，根据光度变化决定是否生成关键帧
      {
        if (!isempty) {
          std::cout<<"first keyframe"<<std::endl;
          pkeyframe->computeBRIEFPoint2_m();
          poseGraph_.addcheckKeyFrame(pkeyframe);

        }
        lastKeyFrame_ = pkeyframe;
        twm_last_ = twm_c;
        loop_key<<pkeyframe->time_stamp<<" "<<poseGraph_.loop_index<<std::endl;
      }
    } else {
      double dt = (twm_c - twm_last_).norm();
      if (dt > 0.005) {
        // std::cout<<"twm_c: "<<twm_c.norm()<<" dt: "<<dt<<std::endl;
        // KeyFrame* pkeyframe = new KeyFrame(&briefExtractor_, &safe_, &multiCamera_, &poseGraph_);
        pkeyframe->init(&briefExtractor_, t, qcm, MrMC, qwm, WrWM, &poseGraph_);

        if (!isempty) {
          pkeyframe->computeBRIEFPoint2_m();
          poseGraph_.addcheckKeyFrame(pkeyframe);

        }
        lastKeyFrame_ = pkeyframe;
        twm_last_ = twm_c;
        loop_key<<pkeyframe->time_stamp<<" "<<poseGraph_.loop_index<<std::endl;
      }
    }
    accept_loop = false;
    // printf("=====loop_close:w_t_vio: %f, %f, %f \n", w_t_vio(0), w_t_vio(1), w_t_vio(2));
    // printf("w_r_vio: %f \n", Utility::R2ypr(w_r_vio)(0));
    // loopdone = true;

  }
}

CreateBOWVEC::CreateBOWVEC() {
  #ifdef DEBUG_LOOP
  system("rm -f /hover/log/latest/vio_hover_log/optiflow_key.txt");
  optiflow_key.open("/hover/log/latest/vio_hover_log/optiflow_key.txt");
  #endif
}

CreateBOWVEC::~CreateBOWVEC() {
  if (optiflowthread.joinable()) {
    optiflowthread.join();
  }
  printf("CreateBOWVEC free\n");
}

void CreateBOWVEC::push(std::map<int, cv::Mat> images, const double &timestamp) {
  MultiImageFrame timestampedImage;
  timestampedImage.timestamp = timestamp;
  
  for (size_t i = 0; i < OmniCamNum; i++)
  {
    timestampedImage.image_map[i] = images[i];
    std::unique_lock<std::mutex> lock(optiflow_mtx);
  }
  MTimageQueue.push(timestampedImage);

  

}

bool CreateBOWVEC::pop(MultiImageFrame &MTimage) {
  std::unique_lock<std::mutex> lock(optiflow_mtx);
  image_cv.wait(lock, [this] { return !MTimageQueue.empty() || stop_optiflowthread; });
  if (!MTimageQueue.empty()) {
    // std::cout << "MTimageQueue num"<<MTimageQueue.size()<<std::endl;
    MultiImageFrame OImage = MTimageQueue.front();
    MTimage.timestamp = OImage.timestamp;
    MTimage.image_map = OImage.image_map; 
    MTimageQueue.pop();
    return true;
  }
  return false;
}

void CreateBOWVEC::push_map(const std::map<int,Eigen::Quaterniond> &qcm, const std::map<int,Eigen::Vector3d> &tmc,
                          const Eigen::Quaterniond &qwm, const Eigen::Vector3d &twm,
                          const double timestamp) {
  std::unique_lock<std::mutex> lock(optiflow_mtx);
  if (poseMap.size() > 10) {
    auto it = poseMap.begin();
    poseMap.erase(it);
  }
  poseMap[timestamp].quaternion = qwm;
  poseMap[timestamp].translation = twm;
  for(const auto& pair : qcm) {
      poseMap[timestamp].qCM[pair.first] = pair.second;
  }
  for(const auto& pair : tmc) {
      poseMap[timestamp].MrMC[pair.first] = pair.second;
  }
}

//生成DB_all,以及每帧的DB_frame
void CreateBOWVEC::CreateBOWVEO1(std::shared_ptr<LoopClosureDetector> LoopClosure) {
  while (1) {
    
    MultiImageFrame frame;
    pop(frame);
    {
      std::unique_lock<std::mutex> lock(optiflow_mtx);
      if (stop_optiflowthread) {
        printf("CreateBOWVECingO1 Image is empty thread terminated\n");
        run_optiflowthread.store(false);
        break;
      }
    }
    // std::cout<<"CreateBOWVECingO1:==="<<std::endl;

    // 假设 frame 已经添加了若干图像

    if (frame.image_map.size() != OmniCamNum) {
        std::cout << "Warning: Not enough images! Expected " << OmniCamNum << ", got "
                  << frame.image_map.size() << std::endl;
        // 可以选择跳过处理
        // return;
        continue;
    }else {
        std::cout << "Got " << frame.image_map.size() << " images" << std::endl;
    }
    // std::cout<<"CreateBOWVECingO1:++"<<std::endl;
    std::map<int, cv::Mat> images;
    for(const auto& img_pair : frame.image_map) {
        images[img_pair.first] = img_pair.second;
    }
    std::shared_ptr<KeyFrame> pkeyframe =
        std::make_shared<KeyFrame>(images);

      std::unique_lock<std::mutex> lclock(optiflow_mtx);
      double timestamp = frame.timestamp;
      optiflow_cv.wait(lclock, [this, timestamp] {
        if (stop_optiflowthread) {
          return true;
        }
        if (!poseMap.empty()) {
          auto lastIt = poseMap.rbegin();
          double latestTimestamp = lastIt->first;
          return latestTimestamp >= timestamp;
        }
        if (!MTimageQueue.empty()){
          return true;
        }
        return false;
      });
      if (stop_optiflowthread) {
        printf("CreateBOWVEC Condition variable thread terminated\n");
        run_optiflowthread.store(false);
        break;
      }
      if(poseMap.empty()&&!MTimageQueue.empty()){
        continue;
      }
      if (!poseMap.count(timestamp) > 0) {
        continue;
      }
      PoseInfo_O1 Info=poseMap[timestamp];
      optiflow_mtx.unlock();
      Eigen::Vector3d twm_c =Info.translation + LoopClosure->w_t_vio;
      if (!pkeyframe->isImgEmpty()) {//zika
        // std::cout<<"start init keyframe"<<std::endl;
        pkeyframe->init(&LoopClosure->briefExtractor_, frame.timestamp, Info.qCM, Info.MrMC, Info.quaternion, Info.translation,
                        &LoopClosure->poseGraph_);  //提取描述子
        // if(timestamp<LoopClosure->backtime_){
        LoopClosure->poseGraph_.lastkeyframe = pkeyframe;
        if (LoopClosure->lastKeyFrame_ == nullptr) {
          if (twm_c.norm() > 0.005)  //可以考虑是否换成DSO那样，根据光度变化决定是否生成关键帧
          {
            std::cout<<"first keyframe"<<std::endl;
            pkeyframe->computeBRIEFPoint_m();
            // std::cout<<"computeBRIEFPoint_m finish \t add keyframe"<<std::endl;
            LoopClosure->poseGraph_.addallKeyFrame(pkeyframe);  //主体在这里
            
  #ifdef PRCURVE
            poseGraph_.tlast = twm_c;
  #endif
            LoopClosure->lastKeyFrame_ = pkeyframe;
            LoopClosure->twm_last_ = twm_c;
            optiflow_key<<pkeyframe->time_stamp<<std::endl;
          }
        } else {
          double dt = (twm_c - LoopClosure->twm_last_).norm();
          if (dt > 0.005) {
            pkeyframe->computeBRIEFPoint_m();
  #ifdef PRCURVE
            poseGraph_.tnow = twm_c;
            poseGraph_.totaldis += (poseGraph_.tnow - poseGraph_.tlast).norm();
            poseGraph_.tlast = poseGraph_.tnow;
  #endif
            LoopClosure->poseGraph_.ismatch = LoopClosure->poseGraph_.addallKeyFrame(pkeyframe);
            LoopClosure->lastKeyFrame_ = pkeyframe;
            LoopClosure->twm_last_ = twm_c;
            optiflow_key<<pkeyframe->time_stamp<<std::endl;
          }
        }
        //}
      } else
        printf("image is empty\n");
  
}}
    

void CreateBOWVEC::stop() {
  {
    std::unique_lock<std::mutex> lock(optiflow_mtx);
    if (MTimageQueue.empty()) {
      stop_optiflowthread = true;
    }
  }
  if (stop_optiflowthread) {
    optiflow_cv.notify_one();
    image_cv.notify_one();
  }
}
void CreateBOWVEC::Forced_stop() {
  {
    std::unique_lock<std::mutex> lock(optiflow_mtx);
    stop_optiflowthread = true;
  }
  if (stop_optiflowthread) {
    optiflow_cv.notify_one();
    image_cv.notify_one();
  }
}
}  // namespace rovio