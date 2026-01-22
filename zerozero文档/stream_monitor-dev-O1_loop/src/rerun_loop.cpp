#include "rerun_loop.hpp"
#include <replaykit/replaykit.h>
#include <cmath>      // for pow

// ==================== Loopdetection Implementation ====================

Loopdetection::Loopdetection(
    const std::string& replay_file,
    const std::string& replay_speed,
    const std::string& replay_skip_time)
    : replay_file_(replay_file),
      replay_speed_(replay_speed),
      replay_skip_time_(replay_skip_time),
      rec_(rerun::RecordingStream("loop_detection")) {

    auto spawn_result = rec_.spawn();
    if (spawn_result.is_err()) {
        std::cerr << "Failed to spawn Rerun loop detection: " << spawn_result.description << std::endl;
        exit(1);
    }
    voc = std::make_shared<const BriefVocabulary>(VOCABULARY_FILE);
    loadPattern(BRIEF_PATTERN_FILE);
    RunPlay();
    cv::namedWindow("Camera Feed", cv::WINDOW_AUTOSIZE);
    std::cout << "Replay file loaded Finish: " << replay_file_ << std::endl;
}

Loopdetection::~Loopdetection() {
    cv::destroyWindow("Camera Feed");
}

void Loopdetection::RunPlay() {
    std::cout << "Starting replay..." << std::endl;

    replaykit_.Subscribe<0>([&](double now_time, const captain::SensorData& sensor_data) {
        // std::cout << "Timestamp: " << now_time << std::endl;
        // std::cout << "Odometry timestamp: " << sensor_data.odometry().timestamp_sec() << "s "
        //           << sensor_data.odometry().timestamp_nsec() << "ns" << std::endl;
        // for (int i = 0; i < sensor_data.images_size(); ++i) {
        //     std::cout << "Image " << i << " timestamp: "
        //               << sensor_data.images(i).timestamp_sec() << "s "
        //               << sensor_data.images(i).timestamp_nsec() << "ns" << std::endl;
        // }
        // 循环检测flag
        double image_time = sensor_data.images(0).timestamp_sec() +
                            sensor_data.images(0).timestamp_nsec() * 1e-9;
        static std::atomic<bool> loop_mode(false);
        static bool loop_triggered = false;
        static bool first_triggered = false;
        static bool end_triggered = false;

        double loop_mode_t = 1762408232 ; //10
        double first_mode_t = 1762408271;
        double end_mode_t = 1762408308;

        // double loop_mode_t = 1762312678 ; //13
        // double first_mode_t = 1762312715;
        // double end_mode_t = 1762312752;

        if (!loop_triggered && image_time > loop_mode_t) {
            cout << "loop_mode:true" << endl;
            loop_mode.exchange(true);
            loop_triggered = true;  // 避免重复执行
        }
        else if (!first_triggered && image_time > first_mode_t) {
            cout << "first_loop:true" << endl;
            first_mode.exchange(true);
            first_triggered = true;
        }
        else if (!end_triggered && image_time > end_mode_t) {
            cout << "end_loop:true" << endl;
            end_mode.exchange(true);
            end_triggered = true;
        }
    
        if (loop_mode.exchange(false)) {
        loop_mode_ = true;
        initialize_loop();
        }
        if (first_mode.exchange(false)) {
        first_loop_ = true;
        }
        if (end_mode.exchange(false)) {
        end_loop_ = true;
        }
        sensorDataCallback(sensor_data);
    });

    zz::replaykit::FileReplayReader<ReplayKitType> reader(replay_file_, replaykit_);
    reader.SetStartTime(std::stod(replay_skip_time_));
    std::thread replay_thread([&]() { replaykit_.Start(); });
    reader.Start();
    replay_thread.join();
    std::cout << "Replay finished." << std::endl;
}

void Loopdetection::loadPattern(const std::string& pattern_file) {
    cv::FileStorage fs(pattern_file, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        throw std::runtime_error("Could not open file " + pattern_file);
    }
    fs["x1"] >> pattern_x1;
    fs["x2"] >> pattern_x2;
    fs["y1"] >> pattern_y1;
    fs["y2"] >> pattern_y2;
    fs.release();
}

void Loopdetection::initialize_loop() {
    system("rm -rf ./frames/");
    system("mkdir -p ./frames/");
    system("rm -rf ./images/");
    system("mkdir -p ./images/");
    // system("rm -rf ./triimages/");
    // system("mkdir -p ./triimages/");
    std::cout << "Loop detection initialization..." << std::endl;
    LoopClosure_ = std::make_shared<rovio::LoopClosureDetector>();
    queue = std::make_shared<rovio::CreateBOWVEC>();
    // 👇 关键：传递 Rerun streamm
    LoopClosure_->setRerunStream(&rec_);

    LoopClosure_->initialize(voc, pattern_x1, pattern_y1, pattern_x2, pattern_y2);
    LoopClosure_->loopthread = std::thread(&rovio::LoopClosureDetector::LoopClosure, LoopClosure_.get());
    queue->optiflowthread = std::thread(&rovio::CreateBOWVEC::CreateBOWVEO1, queue.get(), LoopClosure_);
}

void Loopdetection::resetO1(double t) {
    for (int i = 0; i < Omni_nCam_; i++) {
        isValidO1Pyr_[i] = false;
    }
}

bool Loopdetection::areAllO1Valid() {
    for (int i = 0; i < Omni_nCam_; i++) {
        if (!isValidO1Pyr_[i]) return false;
    }
    return true;
}

void Loopdetection::sensorDataCallback(const captain::SensorData& sensordata) {
    std::lock_guard<std::mutex> lock(m_filter_);
    if (!sensordata.has_odometry() || sensordata.images_size() != Omni_nCam_) {
        printf("procSensorData(): sensordata is incomplete!\n");
        return;
    }

    if (loop_mode_) {
        // std::cout << "Loop detection processing..." << std::endl;
        processOdometry(sensordata.odometry());
        for (int i = 0; i < sensordata.images_size(); ++i) {
            processImage(sensordata.images(i), i);
        }
        double imageTime = sensordata.images(0).timestamp_sec() +
                           sensordata.images(0).timestamp_nsec() * 1e-9;
        // std::cout << "imageTime: " << imageTime << std::endl;
        if (areAllO1Valid()) {
            resetO1(imageTime);
            O1updateAndPublish(imageTime);
        }
    }
}
void Loopdetection::processImage(const captain::Image& proto_image, int index) {
    if (proto_image.data().empty()) {
        printf("RovioNode::processImage(): img is empty!\n");
        return;
    }

    cv::Mat cv_img;

    if (proto_image.encoding() == "jpeg") {
        std::vector<uint8_t> jpeg_data(
            reinterpret_cast<const uint8_t*>(proto_image.data().data()),
            reinterpret_cast<const uint8_t*>(proto_image.data().data()) + proto_image.data().size()
        );

        // 解码 JPEG 为 BGR 格式
        cv::Mat decoded_image = cv::imdecode(jpeg_data, cv::IMREAD_COLOR);
        if (decoded_image.empty()) {
            std::cerr << "Failed to decode JPEG image" << std::endl;
            return;
        }

        // 转为灰度图（单通道）
        cv::cvtColor(decoded_image, cv_img, cv::COLOR_BGR2GRAY);
    } 
    else {
        std::cerr << "Unsupported image encoding: " << proto_image.encoding() << std::endl;
        return;
    }

    if (index < 0 || index >= Omni_nCam_) {
        printf("RovioNode::processImage(): img index out of range!\n");
        return;
    }

    // ✅ 存储灰度图
    {
        std::lock_guard<std::mutex> img_lock(images_mutex_);
        current_images_[index] = cv_img.clone(); // 深拷贝
    }

    // ✅ 用灰度图计算
    // computeFromImageLevel1(cv_img, true, O1pyr_[index]);
    O1pyr_[index]=current_images_[index];
    isValidO1Pyr_[index] = true;
}

void Loopdetection::processOdometry(const captain::Odometry& odometry) {
    // std::lock_guard<std::mutex> lock(m_filter_);
    if (!odometry.has_pose() || !odometry.has_twist()) {
        printf("procOdometry(): odometry is incomplete!\n");
        return;
    }
    // std::cout << "processOdometry()" << std::endl;

    Eigen::Quaterniond qwm;
    qwm.w() = odometry.pose().orientation().w();
    qwm.x() = odometry.pose().orientation().x();
    qwm.y() = odometry.pose().orientation().y();
    qwm.z() = odometry.pose().orientation().z();
    Eigen::Vector3d WrWM;
    WrWM.x() = odometry.pose().position().x();
    WrWM.y() = odometry.pose().position().y();
    // WrWM.z() = odometry.pose().position().z();
    WrWM.z() = 0;
    qwm_ = qwm;
    WrWM_ = WrWM;
    // std::cout << "WrWM: " << WrWM_.transpose() << std::endl;
    // std::cout << "qwm: " << qwm_.w() << " " << qwm_.x() << " " << qwm_.y() << " " << qwm_.z() << std::endl;

    trajectory_.push_back(WrWM_);

 
    if (trajectory_.size() > 1) {
        // 转换为 Rerun 格式
        std::vector<rerun::datatypes::Vec3D> points;
        for (const auto& pt : trajectory_) {
            points.emplace_back(pt.x(), pt.y(), pt.z());
        }

        double time = odometry.timestamp_sec() + odometry.timestamp_nsec() * 1e-9;
        rec_.set_time_seconds("sensor_time", time);

        // 发送轨迹
        rerun::archetypes::LineStrips3D line_strip(
            std::vector<rerun::Collection<rerun::datatypes::Vec3D>>{rerun::Collection<rerun::datatypes::Vec3D>(points)}
        );
        rec_.log("trajectory/robot_path", line_strip);
    }
}


void Loopdetection::halfSample(const cv::Mat& imgIn, cv::Mat& imgOut) {
    imgOut.create(imgIn.rows / 2, imgIn.cols / 2, imgIn.type());
    const int refStepIn = imgIn.step.p[0];
    const int refStepOut = imgOut.step.p[0];
    uint8_t* imgPtrInTop;
    uint8_t* imgPtrInBot;
    uint8_t* imgPtrOut;
    for (int y = 0; y < imgOut.rows; ++y) {
        imgPtrInTop = imgIn.data + 2 * y * refStepIn;
        imgPtrInBot = imgIn.data + (2 * y + 1) * refStepIn;
        imgPtrOut = imgOut.data + y * refStepOut;
        for (int x = 0; x < imgOut.cols; ++x, ++imgPtrOut, imgPtrInTop += 2, imgPtrInBot += 2) {
            *imgPtrOut = (imgPtrInTop[0] + imgPtrInTop[1] + imgPtrInBot[0] + imgPtrInBot[1]) / 4;
        }
    }
}


bool Loopdetection::hasMeasurementAt(double t) {
    return O1pyr_.size() == 4;
}

bool Loopdetection::isFrameEmpty() {
    return O1pyr_.size() != 4;
}

void Loopdetection::O1updateAndPublish(double updateTime) {
    // ✅ 发送4张图像到 Rerun（在锁外）
    {
        std::lock_guard<std::mutex> img_lock(images_mutex_);
        for (int i = 0; i < Omni_nCam_; ++i) {
            if (!current_images_[i].empty()) {
                // 转 RGB
                cv::Mat rgb_img = current_images_[i];
                if (current_images_[i].channels() == 1) {
                    cv::cvtColor(current_images_[i], rgb_img, cv::COLOR_GRAY2RGB);
                } else if (current_images_[i].channels() == 3) {
                    cv::cvtColor(current_images_[i], rgb_img, cv::COLOR_BGR2RGB);
                }

                std::vector<uint8_t> data(rgb_img.data, rgb_img.data + rgb_img.total() * rgb_img.elemSize());
                rerun::WidthHeight res{static_cast<uint32_t>(rgb_img.cols), static_cast<uint32_t>(rgb_img.rows)};
                std::string path = "camera/" + std::to_string(i) + "/image";
                rec_.set_time_seconds("sensor_time", updateTime);
                rec_.log(path, rerun::archetypes::Image::from_rgb24(data, res));
            }
        }
    }

    if (loop_mode_ && end_loop_ && LoopClosure_) {
        LoopClosure_->stop_LoopClosure();
        if (queue->run_optiflowthread.load() == 1) {
            queue->Forced_stop();
        }
    }
    if (loop_mode_) {
        bool isframeempty = !hasMeasurementAt(updateTime);
        if (!isframeempty) {
            for (size_t i = 0; i < Omni_nCam_; i++) {
                safe_O1imgBackup_[i] = O1pyr_[i];
            }
        }
        if (!isframeempty && !first_loop_ && queue) {
            queue->push(safe_O1imgBackup_, updateTime);
            queue->image_cv.notify_one();
        }
    }

    if (loop_mode_ && LoopClosure_ && LoopClosure_->run_loop_closure.load() == 0) {
        finish_loop();
        loop_mode_ = false;
        first_loop_ = false;
        end_loop_ = false;
    }

    if (loop_mode_ && queue && LoopClosure_) {
        if (!first_loop_ && !isFrameEmpty()) {
            queue->push_map(qcm_, MrMC_, qwm_, WrWM_, updateTime);
            queue->optiflow_cv.notify_all();
        }
        if (first_loop_) {
            queue->stop();
        }
        if (queue->run_optiflowthread.load() == 0 && first_loop_) {
            LoopClosure_->run_loop(safe_O1imgBackup_, qcm_, MrMC_, qwm_, WrWM_, updateTime);
        }
    }
}

void Loopdetection::finish_loop() {
    LoopClosure_.reset();
    queue.reset();
}

// ==================== ReplayViewer Implementation ====================

ReplayViewer::ReplayViewer(
    const std::string& replay_file,
    const std::string& replay_speed,
    const std::string& replay_skip_time)
    : replay_file_(replay_file),
      replay_speed_(replay_speed),
      replay_skip_time_(replay_skip_time),
      rec_(rerun::RecordingStream("replay_viewer")) {

    auto spawn_result = rec_.spawn();
    if (spawn_result.is_err()) {
        std::cerr << "Failed to spawn Rerun viewer: " << spawn_result.description << std::endl;
        exit(1);
    }
    cv::namedWindow("Camera Feed", cv::WINDOW_AUTOSIZE);
    std::cout << "Replay file loaded Finish: " << replay_file_ << std::endl;
}

ReplayViewer::~ReplayViewer() {
    cv::destroyWindow("Camera Feed");
}

void ReplayViewer::Play() {
    replaykit_.Subscribe<0>([&](double now_time, const captain::SensorData& sensor_data) {
        std::cout << "==========Odometry timestamp: " << sensor_data.odometry().timestamp_sec() << "s "
                  << sensor_data.odometry().timestamp_nsec() << "ns" << std::endl;
        std::cout << "Image " << 1 << " timestamp: "
            << sensor_data.images(1).timestamp_sec() << "s "
            << sensor_data.images(1).timestamp_nsec() << "ns" << std::endl;
        for (int i = 0; i < sensor_data.images_size(); ++i) {
            // std::cout << "Image " << i << " timestamp: "
            //           << sensor_data.images(i).timestamp_sec() << "s "
            //           << sensor_data.images(i).timestamp_nsec() << "ns" << std::endl;
            visualizeImageInRerun(sensor_data.images(i), i);
        }
        visualizeOdometry(sensor_data.odometry());
    });

    std::cout << "Starting replay..." << std::endl;
    zz::replaykit::FileReplayReader<ReplayKitType> reader(replay_file_, replaykit_);
    reader.SetStartTime(std::stod(replay_skip_time_));
    std::thread replay_thread([&]() { replaykit_.Start(); });
    reader.Start();
    replay_thread.join();
    std::cout << "Replay finished." << std::endl;
}


void ReplayViewer::visualizeOdometry(const captain::Odometry& odometry) {
    // std::lock_guard<std::mutex> lock(m_filter_);
    if (!odometry.has_pose() || !odometry.has_twist()) {
        printf("procOdometry(): odometry is incomplete!\n");
        return;
    }
    // std::cout << "processOdometry()" << std::endl;

    Eigen::Quaterniond qwm;
    qwm.w() = odometry.pose().orientation().w();
    qwm.x() = odometry.pose().orientation().x();
    qwm.y() = odometry.pose().orientation().y();
    qwm.z() = odometry.pose().orientation().z();
    Eigen::Vector3d WrWM;
    WrWM.x() = odometry.pose().position().x();
    WrWM.y() = odometry.pose().position().y();
    // WrWM.z() = odometry.pose().position().z();
    WrWM.z() = 0;
    qwm_ = qwm;
    WrWM_ = WrWM;
    // std::cout << "WrWM: " << WrWM_.transpose() << std::endl;
    // std::cout << "qwm: " << qwm_.w() << " " << qwm_.x() << " " << qwm_.y() << " " << qwm_.z() << std::endl;

    trajectory_.push_back(WrWM_);

 
    if (trajectory_.size() > 1) {
        // 转换为 Rerun 格式
        std::vector<rerun::datatypes::Vec3D> points;
        for (const auto& pt : trajectory_) {
            points.emplace_back(pt.x(), pt.y(), pt.z());
        }

        double time = odometry.timestamp_sec() + odometry.timestamp_nsec() * 1e-9;
        rec_.set_time_seconds("sensor_time", time);

        // 发送轨迹
        rerun::archetypes::LineStrips3D line_strip(
            std::vector<rerun::Collection<rerun::datatypes::Vec3D>>{rerun::Collection<rerun::datatypes::Vec3D>(points)}
        );
        rec_.log("trajectory/robot_path", line_strip);
    }
}

// void ReplayViewer::visualizeOdometry(const captain::Odometry& proto_odom) {
//     auto timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
//         std::chrono::seconds(proto_odom.timestamp_sec()) +
//         std::chrono::nanoseconds(proto_odom.timestamp_nsec())
//     );

//     rerun::datatypes::Vec3D position(
//         proto_odom.pose().position().x(),
//         proto_odom.pose().position().y(),
//         proto_odom.pose().position().z()
//     );

//     rerun::datatypes::Quaternion quaternion = rerun::datatypes::Quaternion::from_xyzw(
//         proto_odom.pose().orientation().x(),
//         proto_odom.pose().orientation().y(),
//         proto_odom.pose().orientation().z(),
//         proto_odom.pose().orientation().w()
//     );

//     rerun::archetypes::Transform3D transform =
//         rerun::archetypes::Transform3D::from_translation_rotation(position, quaternion);

//     rec_.set_time_nanos("timestamp", timestamp.count());
//     rec_.log("world/robot", transform);
// }

void ReplayViewer::visualizeImageInRerun(const captain::Image& proto_image, int image_index) {
    auto timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::seconds(proto_image.timestamp_sec()) +
        std::chrono::nanoseconds(proto_image.timestamp_nsec())
    );
    rec_.set_time_nanos("timestamp", timestamp.count());

    std::string entity_path = "camera/image";
    if (image_index > 0) {
        entity_path += "_" + std::to_string(image_index);
    }

    rerun::WidthHeight resolution{proto_image.width(), proto_image.height()};

    if (proto_image.encoding() == "jpeg") {
        std::vector<uint8_t> jpeg_data(
            reinterpret_cast<const uint8_t*>(proto_image.data().data()),
            reinterpret_cast<const uint8_t*>(proto_image.data().data()) + proto_image.data().size()
        );
        cv::Mat decoded_image = cv::imdecode(jpeg_data, cv::IMREAD_COLOR);
        if (decoded_image.empty()) {
            std::cerr << "Failed to decode JPEG image" << std::endl;
            return;
        }
        cv::Mat rgb_image;
        cv::cvtColor(decoded_image, rgb_image, cv::COLOR_BGR2RGB);
        resolution = rerun::WidthHeight{static_cast<uint32_t>(rgb_image.cols),
                                        static_cast<uint32_t>(rgb_image.rows)};
        std::vector<uint8_t> image_data(rgb_image.data,
                                        rgb_image.data + rgb_image.total() * rgb_image.elemSize());
        rerun::archetypes::Image image = rerun::archetypes::Image::from_rgb24(image_data, resolution);
        rec_.log(entity_path, image);
        // std::cout<<"image_type: jpeg"<<std::endl;

        return;
    }

    if (proto_image.encoding() == "rgb8") {
        std::vector<uint8_t> image_data(
            reinterpret_cast<const uint8_t*>(proto_image.data().data()),
            reinterpret_cast<const uint8_t*>(proto_image.data().data()) + proto_image.data().size()
        );
        rerun::archetypes::Image image = rerun::archetypes::Image::from_rgb24(image_data, resolution);
        rec_.log(entity_path, image);
        // std::cout<<"image_type: rgb8"<<std::endl;

    } else if (proto_image.encoding() == "bgr8") {
        cv::Mat bgr_image(proto_image.height(), proto_image.width(), CV_8UC3,
                          (void*)proto_image.data().data());
        cv::Mat rgb_image;
        cv::cvtColor(bgr_image, rgb_image, cv::COLOR_BGR2RGB);
        std::vector<uint8_t> image_data(rgb_image.data,
                                        rgb_image.data + rgb_image.total() * rgb_image.elemSize());
        rerun::archetypes::Image image = rerun::archetypes::Image::from_rgb24(image_data, resolution);
        rec_.log(entity_path, image);
        // std::cout<<"image_type: bgr8"<<std::endl;

    } else if (proto_image.encoding() == "mono8") {
        std::vector<uint8_t> image_data(
            reinterpret_cast<const uint8_t*>(proto_image.data().data()),
            reinterpret_cast<const uint8_t*>(proto_image.data().data()) + proto_image.data().size()
        );
        rerun::archetypes::Image image = rerun::archetypes::Image::from_greyscale8(image_data, resolution);
        rec_.log(entity_path, image);
        // std::cout<<"image_type: mono8"<<std::endl;
    }

    if (!proto_image.frame_id().empty()) {
        rec_.log(entity_path + "/frame_info",
                 rerun::archetypes::TextLog(proto_image.frame_id())
                     .with_level(rerun::components::TextLogLevel::Info));
    }
}