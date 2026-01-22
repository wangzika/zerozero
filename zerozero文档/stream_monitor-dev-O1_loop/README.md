# Stream Monitor

这个项目从 `tcp://*:5555` 端口接收 SensorData (protobuf格式)，并使用 Rerun 和 OpenCV 进行可视化。

## 功能特性

- 接收包含图像和Odometry数据的SensorData
- 使用OpenCV显示实时图像
- 使用Rerun进行3D可视化：
  - 机器人位置和姿态
  - 速度向量可视化
  - 图像数据流

## 依赖项

- OpenCV (图像处理)
- Protocol Buffers (数据序列化)
- nanomsg (网络通信)
- Rerun C++ SDK (3D可视化)

## 编译

```bash
mkdir build && cd build
cmake ..
make
```

## 安装依赖

```bash
# Ubuntu/Debian
sudo apt-get install libopencv-dev libprotobuf-dev protobuf-compiler libnanomsg-dev

# 或使用包管理器如 vcpkg, conan 等
```

## 运行

```bash
./stream_monitor
```

程序将：
1. 启动Rerun查看器窗口进行3D可视化
2. 显示OpenCV窗口显示接收到的图像
3. 在终端输出处理状态信息

## Rerun可视化内容

- `world/robot`: 机器人的位置和姿态
- `world/robot/linear_velocity`: 线性速度向量(绿色)
- `world/robot/angular_velocity`: 角速度向量(红色)  
- `camera/image`: 相机图像流
- `camera/image_N`: 多相机情况下的额外图像流

## 网络协议

程序监听 `tcp://*:5555` 端口，期望接收序列化的 `captain::SensorData` protobuf消息。
