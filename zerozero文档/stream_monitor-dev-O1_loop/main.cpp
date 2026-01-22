#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <rerun.hpp>
#include "network_manager.h"
#include "captain_msgs.pb.h"
#include "rerun/datatypes/quaternion.hpp"
#include <google/protobuf/message.h>
#include <chrono>
#include <thread>

class StreamMonitor {
public:
    StreamMonitor() : rec_(rerun::RecordingStream("stream_monitor")) {
        // 初始化 Rerun
        auto spawn_result = rec_.spawn();
        if (spawn_result.is_err()) {
            std::cerr << "Failed to spawn Rerun viewer: " << spawn_result.description << std::endl;
            return;
        }
        
        // if (!network_manager_.addSocket("sensor_data", {SocketType::SUBSCRIBER, "tcp://10.10.30.149:5555"})) {
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
            visualizeOdometry(sensor_data.odometry());
        }
        
        // 处理图像数据
        for (int i = 0; i < sensor_data.images_size(); ++i) {
            processImage(sensor_data.images(i), i);
        }
        
    }
    
    void visualizeOdometry(const captain::Odometry& proto_odom) {
        // 创建时间戳
        auto timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::seconds(proto_odom.timestamp_sec()) + 
            std::chrono::nanoseconds(proto_odom.timestamp_nsec())
        );
        
        // 记录位置
        rerun::datatypes::Vec3D position(
            proto_odom.pose().position().x(),
            proto_odom.pose().position().y(),
            proto_odom.pose().position().z()
        );
        
        // 记录姿态（四元数） - 使用正确的构造方式
        rerun::datatypes::Quaternion quaternion = rerun::datatypes::Quaternion::from_xyzw(
            proto_odom.pose().orientation().x(),
            proto_odom.pose().orientation().y(),
            proto_odom.pose().orientation().z(),
            proto_odom.pose().orientation().w()
        );
        
        // 组合为Transform3D
        rerun::archetypes::Transform3D transform = rerun::archetypes::Transform3D::from_translation_rotation(
            position, quaternion
        );
        
        // 记录到Rerun
        rec_.set_time_nanos("timestamp", timestamp.count());
        rec_.log("world/robot", transform);
        
        // 记录速度向量
        rerun::datatypes::Vec3D linear_velocity(
            proto_odom.twist().linear().x(),
            proto_odom.twist().linear().y(),
            proto_odom.twist().linear().z()
        );
        
        rerun::datatypes::Vec3D angular_velocity(
            proto_odom.twist().angular().x(),
            proto_odom.twist().angular().y(),
            proto_odom.twist().angular().z()
        );
        
        // 计算速度大小
        double linear_magnitude = std::sqrt(linear_velocity.x() * linear_velocity.x() + 
                                          linear_velocity.y() * linear_velocity.y() + 
                                          linear_velocity.z() * linear_velocity.z());
        
        double angular_magnitude = std::sqrt(angular_velocity.x() * angular_velocity.x() + 
                                           angular_velocity.y() * angular_velocity.y() + 
                                           angular_velocity.z() * angular_velocity.z());
        
        // 可视化速度向量
        if (linear_magnitude > 0.01) { // 只在有明显速度时显示
            rec_.log("world/robot/linear_velocity", 
                    rerun::archetypes::Arrows3D::from_vectors({linear_velocity})
                        .with_colors({rerun::datatypes::Rgba32(0, 255, 0, 255)}));
        }
        
        if (angular_magnitude > 0.01) { // 只在有明显角速度时显示
            rec_.log("world/robot/angular_velocity", 
                    rerun::archetypes::Arrows3D::from_vectors({angular_velocity})
                        .with_colors({rerun::datatypes::Rgba32(255, 0, 0, 255)}));
        }
        
        std::cout << "Visualized odometry: pos(" 
                  << proto_odom.pose().position().x() << ", "
                  << proto_odom.pose().position().y() << ", "
                  << proto_odom.pose().position().z() << ")" << std::endl;
    }
    
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

int main(int argc, char** argv) {
    std::cout << "Starting Stream Monitor..." << std::endl;
    
    try {
        StreamMonitor monitor;
        // 程序将在StreamMonitor的构造函数中运行主循环
    } catch (const std::exception& e) {
        std::cerr << "Stream monitor error: " << e.what() << std::endl;
        return -1;
    }
    
    return 0;
}
