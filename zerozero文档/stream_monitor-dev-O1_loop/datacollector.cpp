#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <rerun.hpp>
#include <Eigen/Dense>

#include "build/captain_msgs.pb.h"
#include "network_manager.h"
#include "captain_msgs.pb.h"
#include "rovio.pb.h"
#include "image.pb.h"
#include "rerun/datatypes/quaternion.hpp"
#include <google/protobuf/message.h>
#include <unistd.h>
#include <replaykit/replaykit.h>
#include <chrono>
#include <string>
#include <thread>

// using ReplayKitType =
//     ::zz::replaykit::ReplayKit<::zz::replaykit::Topics<captain::Odometry, captain::Image>,
//                                ::zz::replaykit::Commands<>>;

using ReplayKitType =
    ::zz::replaykit::ReplayKit<::zz::replaykit::Topics<captain::SensorData,vision::StereoImage, rovio::InputInfoPack>,
                               ::zz::replaykit::Commands<>>;

class DataCollector {
public:
    DataCollector( ReplayKitType& replay_kit)
        :replay_kit_(replay_kit) {
        // if (config_.publishTCP) {
        //     if (!network_manager_.addSocket("sensor_data", {SocketType::SUBSCRIBER, "tcp://10.10.30.149:5555"})) {
        //     std::cerr << "Failed to add socket configuration" << std::endl;
        //     return;
        // }
        
        // if (!network_manager_.addSocket("sensor_data", {SocketType::SUBSCRIBER, "tcp://10.10.30.149:5555"})) {
        if (!network_manager_.addSocket("sensor_data", {SocketType::SUBSCRIBER, "tcp://172.20.10.2:5555"})) {
            std::cerr << "Failed to add socket configuration" << std::endl;
            return;
        }
        
        if (!network_manager_.initializeAll()) {
            std::cerr << "Failed to initialize network sockets" << std::endl;
            return;
        }
        
        std::cout << "DataCollector  initialized, waiting for data..." << std::endl;
        // 启动数据接收循环
        // startReceiving();
    }

    void startReceiving() {
        while (true) {
            // 接收数据
            std::cout << "DataCollector Waiting for sensor data..." << std::endl;
            std::string data = network_manager_.receiveData("sensor_data");
            std::cout << "Received data: " << data.size() << " bytes" << std::endl;
            
            if (!data.empty()) {
                processSensorData(data);
            }
            
            // 控制接收频率
            // std::this_thread::sleep_for(std::chrono::milliseconds(33)); // ~30 Hz
        }
    }

    ~DataCollector() {
        network_manager_.cleanup();
    }
private:
    // Config config_;
    ReplayKitType& replay_kit_;
    NetworkManager network_manager_;




    void processSensorData(const std::string& data) {
        captain::SensorData sensor_data;
        
        if (!sensor_data.ParseFromString(data)) {
            std::cerr << "Failed to parse SensorData protobuf" << std::endl;
            return;
        }
        if(sensor_data.has_odometry() && sensor_data.images_size()==4){
            std::cout << "Processing SensorData with odometry and " << sensor_data.images_size() << " images." << std::endl;
            std::cout << "Odometry timestamp: " << sensor_data.odometry().timestamp_sec() << "s " << sensor_data.odometry().timestamp_nsec() << "ns" << std::endl;
            std::cout << "First image timestamp: " << sensor_data.images(0).timestamp_sec() << "s " << sensor_data.images(0).timestamp_nsec() << "ns" << std::endl;

            replay_kit_.Publish<0>(replay_kit_.Now(), sensor_data);
        } else {
            std::cout << "SensorData missing odometry or images." << std::endl;
            return;
        }
        
        // // 处理 Odometry 数据
        // if (sensor_data.has_odometry()) {
        //     ProcessOdometry(sensor_data.odometry());
        // }
        
        // // 处理图像数据
        // for (int i = 0; i < sensor_data.images_size(); ++i) {
        //     processImage(sensor_data.images(i), i);
        // }
        
    }

    // void ProcessOdometry(const captain::Odometry& proto_odom) {
    //     std::mutex proto_odom_mutex;
    //     // 创建时间戳
    //     std::lock_guard<std::mutex> lock(proto_odom_mutex);
    //     replay_kit_.Publish<0>(replay_kit_.Now(), proto_odom);
    // }

    // void processImage(const captain::Image& proto_image, int image_index) {
    //     try {
    //         std::mutex proto_image_mutex;
    //         std::lock_guard<std::mutex> lock(proto_image_mutex);
    //         replay_kit_.Publish<1>(replay_kit_.Now(), proto_image);
    //     } catch (const std::exception& e) {
    //         std::cerr << "Error processing image " << image_index << ": " << e.what() << std::endl;
    //     }
    // }
};


class StreamMonitor {
public:
    StreamMonitor() : rec_(rerun::RecordingStream("stream_monitor")) {
        // 初始化 Rerun
        auto spawn_result = rec_.spawn();
        if (spawn_result.is_err()) {
            std::cerr << "Failed to spawn Rerun viewer: " << spawn_result.description << std::endl;
            return;
        }
        
        if (!network_manager_.addSocket("sensor_data", {SocketType::SUBSCRIBER, "tcp://172.20.10.2:5555"})) {
            std::cerr << "Failed to add socket configuration" << std::endl;
            return;
        }
        
        if (!network_manager_.initializeAll()) {
            std::cerr << "Failed to initialize network sockets" << std::endl;
            return;
        }
        
        std::cout << "Stream monitor initialized, waiting for data..." << std::endl;
        
        // 创建 OpenCV 窗口
        cv::namedWindow("Camera Feed", cv::WINDOW_AUTOSIZE);
        
        // 启动数据接收循环
        startReceiving();
    }
    
    ~StreamMonitor() {
        cv::destroyAllWindows();
        network_manager_.cleanup();
    }

private:
    std::vector<Eigen::Vector3d> trajectory_; // 存储历史轨迹点


    NetworkManager network_manager_;
    rerun::RecordingStream rec_;
    
    void startReceiving() {
        while (true) {
            // 接收数据
            std::cout << "Waiting for sensor data..." << std::endl;
            std::string data = network_manager_.receiveData("sensor_data");
            std::cout << "Received data: " << data.size() << " bytes" << std::endl;
            
            if (!data.empty()) {
                processSensorData(data);
            }
            
            // 控制接收频率
            // std::this_thread::sleep_for(std::chrono::milliseconds(33)); // ~30 Hz
        }
    }
    
    void processSensorData(const std::string& data) {
        captain::SensorData sensor_data;
        
        if (!sensor_data.ParseFromString(data)) {
            std::cerr << "Failed to parse SensorData protobuf" << std::endl;
            return;
        }
        
        // 处理 Odometry 数据
        if (sensor_data.has_odometry()) {
            // std::lock_guard<std::mutex> lock(mutex_);
            visualizeOdometry(sensor_data.odometry());
        }
        
        // 处理图像数据
        for (int i = 0; i < sensor_data.images_size(); ++i) {
            // std::lock_guard<std::mutex> lock(mutex_);
            processImage(sensor_data.images(i), i);
        }
        
    }
    

    void visualizeOdometry(const captain::Odometry& proto_odom) {
        // 创建时间戳
        auto timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::seconds(proto_odom.timestamp_sec()) + 
            std::chrono::nanoseconds(proto_odom.timestamp_nsec())
        );
        


        // === 新增：将位置加入轨迹 ===
        Eigen::Vector3d WrWM;
        WrWM.x() = proto_odom.pose().position().x();
        WrWM.y() = proto_odom.pose().position().y();
        // WrWM.z() = proto_odom.pose().position().z();
        WrWM.z() =0;
        // qwm_ = qwm;
        Eigen::Vector3d WrWM_ = WrWM;
        trajectory_.push_back(WrWM_);
            
        if (trajectory_.size() > 1) {
            // 转换为 Rerun 格式
            std::vector<rerun::datatypes::Vec3D> points;
            for (const auto& pt : trajectory_) {
                points.emplace_back(pt.x(), pt.y(), pt.z());
            }

            double time = proto_odom.timestamp_sec() + proto_odom.timestamp_nsec() * 1e-9;
            rec_.set_time_seconds("sensor_time", time);

            // 发送轨迹
            rerun::archetypes::LineStrips3D line_strip(
                std::vector<rerun::Collection<rerun::datatypes::Vec3D>>{rerun::Collection<rerun::datatypes::Vec3D>(points)}
            );
            rec_.log("trajectory/robot_path", line_strip);
        }
        // // 记录当前位置
        // rerun::datatypes::Vec3D position(
        //     proto_odom.pose().position().x(),
        //     proto_odom.pose().position().y(),
        //     proto_odom.pose().position().z()
        // );
        // // 记录姿态（四元数）
        // rerun::datatypes::Quaternion quaternion = rerun::datatypes::Quaternion::from_xyzw(
        //     proto_odom.pose().orientation().x(),
        //     proto_odom.pose().orientation().y(),
        //     proto_odom.pose().orientation().z(),
        //     proto_odom.pose().orientation().w()
        // );
        
        // // 组合为Transform3D
        // rerun::archetypes::Transform3D transform = rerun::archetypes::Transform3D::from_translation_rotation(
        //     position, quaternion
        // );
        
        // // 设置时间并记录机器人当前位姿
        // rec_.set_time_nanos("timestamp", timestamp.count());
        // rec_.log("world/robot", transform);


        // ========== 速度可视化（保持不变）==========
        // rerun::datatypes::Vec3D linear_velocity(
        //     proto_odom.twist().linear().x(),
        //     proto_odom.twist().linear().y(),
        //     proto_odom.twist().linear().z()
        // );
        
        // rerun::datatypes::Vec3D angular_velocity(
        //     proto_odom.twist().angular().x(),
        //     proto_odom.twist().angular().y(),
        //     proto_odom.twist().angular().z()
        // );
        
        // double linear_magnitude = std::sqrt(
        //     linear_velocity.x() * linear_velocity.x() + 
        //     linear_velocity.y() * linear_velocity.y() + 
        //     linear_velocity.z() * linear_velocity.z()
        // );
        
        // double angular_magnitude = std::sqrt(
        //     angular_velocity.x() * angular_velocity.x() + 
        //     angular_velocity.y() * angular_velocity.y() + 
        //     angular_velocity.z() * angular_velocity.z()
        // );
        
        // if (linear_magnitude > 0.01) {
        //     rec_.log("world/robot/linear_velocity", 
        //         rerun::archetypes::Arrows3D::from_vectors({linear_velocity})
        //             .with_colors({rerun::datatypes::Rgba32(0, 255, 0, 255)}));
        // }
        
        // if (angular_magnitude > 0.01) {
        //     rec_.log("world/robot/angular_velocity", 
        //         rerun::archetypes::Arrows3D::from_vectors({angular_velocity})
        //             .with_colors({rerun::datatypes::Rgba32(255, 0, 0, 255)}));
        // }
        
        // std::cout << "Visualized odometry: pos(" 
        //           << position.x() << ", "
        //           << position.y() << ", "
        //           << position.z() << ")" << std::endl;
    }


    // void visualizeOdometry(const captain::Odometry& proto_odom) {
    //     // 创建时间戳
    //     auto timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
    //         std::chrono::seconds(proto_odom.timestamp_sec()) + 
    //         std::chrono::nanoseconds(proto_odom.timestamp_nsec())
    //     );
        
    //     // 记录位置
    //     rerun::datatypes::Vec3D position(
    //         proto_odom.pose().position().x(),
    //         proto_odom.pose().position().y(),
    //         proto_odom.pose().position().z()
    //     );
        
    //     // 记录姿态（四元数） - 使用正确的构造方式
    //     rerun::datatypes::Quaternion quaternion = rerun::datatypes::Quaternion::from_xyzw(
    //         proto_odom.pose().orientation().x(),
    //         proto_odom.pose().orientation().y(),
    //         proto_odom.pose().orientation().z(),
    //         proto_odom.pose().orientation().w()
    //     );
        
    //     // 组合为Transform3D
    //     rerun::archetypes::Transform3D transform = rerun::archetypes::Transform3D::from_translation_rotation(
    //         position, quaternion
    //     );
        
    //     // 记录到Rerun
    //     rec_.set_time_nanos("timestamp", timestamp.count());
    //     rec_.log("world/robot", transform);
        
    //     // 记录速度向量
    //     rerun::datatypes::Vec3D linear_velocity(
    //         proto_odom.twist().linear().x(),
    //         proto_odom.twist().linear().y(),
    //         proto_odom.twist().linear().z()
    //     );
        
    //     rerun::datatypes::Vec3D angular_velocity(
    //         proto_odom.twist().angular().x(),
    //         proto_odom.twist().angular().y(),
    //         proto_odom.twist().angular().z()
    //     );
        
    //     // 计算速度大小
    //     double linear_magnitude = std::sqrt(linear_velocity.x() * linear_velocity.x() + 
    //                                       linear_velocity.y() * linear_velocity.y() + 
    //                                       linear_velocity.z() * linear_velocity.z());
        
    //     double angular_magnitude = std::sqrt(angular_velocity.x() * angular_velocity.x() + 
    //                                        angular_velocity.y() * angular_velocity.y() + 
    //                                        angular_velocity.z() * angular_velocity.z());
        
    //     // 可视化速度向量
    //     if (linear_magnitude > 0.01) { // 只在有明显速度时显示
    //         rec_.log("world/robot/linear_velocity", 
    //                 rerun::archetypes::Arrows3D::from_vectors({linear_velocity})
    //                     .with_colors({rerun::datatypes::Rgba32(0, 255, 0, 255)}));
    //     }
        
    //     if (angular_magnitude > 0.01) { // 只在有明显角速度时显示
    //         rec_.log("world/robot/angular_velocity", 
    //                 rerun::archetypes::Arrows3D::from_vectors({angular_velocity})
    //                     .with_colors({rerun::datatypes::Rgba32(255, 0, 0, 255)}));
    //     }
        
    //     std::cout << "Visualized odometry: pos(" 
    //               << proto_odom.pose().position().x() << ", "
    //               << proto_odom.pose().position().y() << ", "
    //               << proto_odom.pose().position().z() << ")" << std::endl;
    // }
    
    void processImage(const captain::Image& proto_image, int image_index) {
        try {
            // 创建 OpenCV 图像
            // cv::Mat image;
            
            // if (proto_image.encoding() == "rgb8") {
            //     cv::Mat temp(proto_image.height(), proto_image.width(), CV_8UC3, 
            //                 (void*)proto_image.data().data());
            //     cv::cvtColor(temp, image, cv::COLOR_RGB2BGR);
            // } else if (proto_image.encoding() == "bgr8") {
            //     image = cv::Mat(proto_image.height(), proto_image.width(), CV_8UC3, 
            //                    (void*)proto_image.data().data()).clone();
            // } else if (proto_image.encoding() == "mono8") {
            //     image = cv::Mat(proto_image.height(), proto_image.width(), CV_8UC1, 
            //                    (void*)proto_image.data().data()).clone();
            // } else {
            //     std::cerr << "Unsupported image encoding: " << proto_image.encoding() << std::endl;
            //     return;
            // }
            
            // // OpenCV 显示
            // std::string window_name = "Camera Feed";
            // if (image_index > 0) {
            //     window_name += " " + std::to_string(image_index);
            //     cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
            // }
            
            // // 添加时间戳信息到图像上
            // std::string timestamp_text = "Time: " + std::to_string(proto_image.timestamp_sec()) + 
            //                            "." + std::to_string(proto_image.timestamp_nsec());
            // cv::putText(image, timestamp_text, cv::Point(10, 30), 
            //            cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
            
            // cv::imshow(window_name, image);
            // cv::waitKey(1);
            
            // Rerun 可视化
            visualizeImageInRerun(proto_image, image_index);
            
        } catch (const std::exception& e) {
            std::cerr << "Error processing image " << image_index << ": " << e.what() << std::endl;
        }
    }
    
    void visualizeImageInRerun(const captain::Image& proto_image, int image_index) {
        // 创建时间戳
        auto timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::seconds(proto_image.timestamp_sec()) + 
            std::chrono::nanoseconds(proto_image.timestamp_nsec())
        );
        
        rec_.set_time_nanos("timestamp", timestamp.count());
        
        // 构建entity path
        std::string entity_path = "camera/image";
        if (image_index > 0) {
            entity_path += "_" + std::to_string(image_index);
        }
        
        // 创建图像尺寸 - 修正命名空间
        rerun::WidthHeight resolution{proto_image.width(), proto_image.height()};

        // 检查是否为JPEG编码
        if (proto_image.encoding() == "jpeg") {
            // 从memory buffer解码JPEG数据
            std::vector<uint8_t> jpeg_data(
                reinterpret_cast<const uint8_t*>(proto_image.data().data()),
                reinterpret_cast<const uint8_t*>(proto_image.data().data()) + proto_image.data().size()
            );
            
            // 使用OpenCV解码JPEG
            cv::Mat decoded_image = cv::imdecode(jpeg_data, cv::IMREAD_COLOR);
            
            // 检查解码是否成功
            if (decoded_image.empty()) {
                std::cerr << "Failed to decode JPEG image" << std::endl;
                return;
            }
            
            // 转换为RGB (OpenCV默认解码为BGR)
            cv::Mat rgb_image;
            cv::cvtColor(decoded_image, rgb_image, cv::COLOR_BGR2RGB);
            
            // 将解码后的图像发送到Rerun
            std::vector<uint8_t> image_data(
                rgb_image.data,
                rgb_image.data + rgb_image.total() * rgb_image.elemSize()
            );
            
            // 更新分辨率为实际解码后的尺寸
            resolution = rerun::WidthHeight{static_cast<uint32_t>(rgb_image.cols), 
                                            static_cast<uint32_t>(rgb_image.rows)};
            
            rerun::archetypes::Image image = rerun::archetypes::Image::from_rgb24(
                image_data, resolution
            );
            rec_.log(entity_path, image);
            return;
        }
        
        // 根据编码格式创建图像数据
        if (proto_image.encoding() == "rgb8") {
            // RGB8格式 - 使用vector创建Collection
            std::vector<uint8_t> image_data(
                reinterpret_cast<const uint8_t*>(proto_image.data().data()),
                reinterpret_cast<const uint8_t*>(proto_image.data().data()) + proto_image.data().size()
            );
            
            rerun::archetypes::Image image = rerun::archetypes::Image::from_rgb24(
                image_data, resolution
            );
            rec_.log(entity_path, image);
        } else if (proto_image.encoding() == "bgr8") {
            // BGR8需要转换为RGB8
            cv::Mat bgr_image(proto_image.height(), proto_image.width(), CV_8UC3, 
                             (void*)proto_image.data().data());
            cv::Mat rgb_image;
            cv::cvtColor(bgr_image, rgb_image, cv::COLOR_BGR2RGB);
            
            std::vector<uint8_t> image_data(
                rgb_image.data, 
                rgb_image.data + rgb_image.total() * rgb_image.elemSize()
            );
            
            rerun::archetypes::Image image = rerun::archetypes::Image::from_rgb24(
                image_data, resolution
            );
            rec_.log(entity_path, image);
        } else if (proto_image.encoding() == "mono8") {
            // 单通道灰度图 - 使用vector创建Collection
            std::vector<uint8_t> image_data(
                reinterpret_cast<const uint8_t*>(proto_image.data().data()),
                reinterpret_cast<const uint8_t*>(proto_image.data().data()) + proto_image.data().size()
            );
            
            rerun::archetypes::Image image = rerun::archetypes::Image::from_greyscale8(
                image_data, resolution
            );
            rec_.log(entity_path, image);
        }
        
        // 记录相机的frame_id作为注释
        if (!proto_image.frame_id().empty()) {
            rec_.log(entity_path + "/frame_info", 
                    rerun::archetypes::TextLog(proto_image.frame_id())
                        .with_level(rerun::components::TextLogLevel::Info));
        }
    }
};

// 假设 DataCollector 已经有 startReceiving() 公有函数
int main(int argc, char** argv) {
    std::cout << "Start data collector..." << std::endl;
    // 创建 ReplayKit 实例
    std::mutex replaykit_mutex;
    ReplayKitType replaykit;

    // 创建保存目录
    system("mkdir -p /home/zerozero/Project/Omni-O1/offboard/stream_monitor/data/");
    std::string replay_file = "/home/zerozero/Project/Omni-O1/offboard/stream_monitor/data/Omni.replay";

    // 初始化 ReplayKit
    auto recorder = std::make_shared<zz::replaykit::FileRecorder>(replay_file);
    replaykit.SetRecorder(recorder);
    replaykit.EnableRecord();

    // 初始化数据采集器
    DataCollector collector(replaykit);

    // 启动 ReplayKit 线程
    std::thread t1([&]() {
        std::cout << "[Thread1] ReplayKit started" << std::endl;
        replaykit.Start();
    });

    // 启动数据采集线程
    std::thread t2([&]() {
        std::lock_guard<std::mutex> lock(replaykit_mutex);
        std::cout << "[Thread2] DataCollector started" << std::endl;
        collector.startReceiving();
    });

    // 启动可视化监控线程
    std::thread t3([]() {
        std::cout << "[Thread3] StreamMonitor started" << std::endl;
        try {
            StreamMonitor monitor;
        } catch (const std::exception& e) {
            std::cerr << "Stream monitor error: " << e.what() << std::endl;
        }
    });

    // 等待线程结束（除非你打算后台运行）
    t1.join();
    t2.join();
    t3.join();

    std::cout << "All threads finished.\n";
    return 0;
}
