#ifndef NETWORK_MANAGER_H
#define NETWORK_MANAGER_H

#include <string>
#include <atomic>
#include <unordered_map>
#include <memory>

enum class SocketType {
    PUBLISHER,
    SUBSCRIBER,
    SERVER,
    CLIENT
};

struct SocketConfig {
    SocketType type;
    std::string endpoint;
};

class NetworkManager {
public:
    NetworkManager();
    ~NetworkManager();
    
    // 添加socket配置
    bool addSocket(const std::string& socket_id, const SocketConfig& config);
    
    // 初始化所有已添加的socket
    bool initializeAll();
    
    // 移除socket
    void removeSocket(const std::string& socket_id);
    
    void cleanup();
    
    // 通用发送和接收接口
    bool sendData(const std::string& socket_id, const std::string& data);
    std::string receiveData(const std::string& socket_id);
    

private:
    struct SocketInfo {
        int socket_fd;
        SocketConfig config;
        
        SocketInfo() : socket_fd(-1) {}
    };
    
    std::unordered_map<std::string, std::unique_ptr<SocketInfo>> sockets_;
    
    // Helper methods
    bool initializeSocket(SocketInfo& socket_info);
    void closeSocket(SocketInfo& socket_info);
    
    bool sendDataInternal(int socket, const std::string& data);
    std::string receiveDataInternal(int socket);
    
    int getSocketTypeFlag(SocketType type) const;

    bool validateEndpoint(const std::string& endpoint, SocketType type);
};

#endif // NETWORK_MANAGER_H
