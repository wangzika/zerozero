#include "network_manager.h"
#include <nanomsg/nn.h>
#include <nanomsg/tcp.h>
#include <nanomsg/pubsub.h>
#include <nanomsg/reqrep.h>

NetworkManager::NetworkManager() {
}

NetworkManager::~NetworkManager() {
    cleanup();
}

bool NetworkManager::addSocket(const std::string& socket_id, const SocketConfig& config) {
    if (sockets_.find(socket_id) != sockets_.end()) {
        printf("Socket %s already exists, replacing it", socket_id.c_str());
        removeSocket(socket_id);
    }
    
    auto socket_info = std::make_unique<SocketInfo>();
    socket_info->config = config;
    sockets_[socket_id] = std::move(socket_info);
    
    printf("Added socket configuration: %s on %s", 
             socket_id.c_str(), config.endpoint.c_str());
    return true;
}

bool NetworkManager::initializeAll() {
    bool all_success = true;
    
    for (auto& [socket_id, socket_info] : sockets_) {
        if (!initializeSocket(*socket_info)) {
            printf("Failed to initialize socket: %s", socket_id.c_str());
            all_success = false;
        }
    }
    
    if (all_success) {
        printf("NetworkManager: All sockets initialized successfully");
    } else {
        printf("NetworkManager: Some sockets failed to initialize");
    }
    
    return all_success;
}

void NetworkManager::removeSocket(const std::string& socket_id) {
    auto it = sockets_.find(socket_id);
    if (it != sockets_.end()) {
        closeSocket(*it->second);
        sockets_.erase(it);
        printf("Removed socket: %s", socket_id.c_str());
    }
}

void NetworkManager::cleanup() {
    for (auto& [socket_id, socket_info] : sockets_) {
        closeSocket(*socket_info);
    }
    sockets_.clear();
    printf("NetworkManager cleanup completed");
}

bool NetworkManager::sendData(const std::string& socket_id, const std::string& data) {
    auto it = sockets_.find(socket_id);
    if (it == sockets_.end()) {
        printf("Socket not found: %s", socket_id.c_str());
        return false;
    }
    
    if (it->second->config.type != SocketType::PUBLISHER) {
        printf("Socket %s is not a publisher", socket_id.c_str());
        return false;
    }
    
    return sendDataInternal(it->second->socket_fd, data);
}

std::string NetworkManager::receiveData(const std::string& socket_id) {
    auto it = sockets_.find(socket_id);
    if (it == sockets_.end()) {
        printf("Socket not found: %s", socket_id.c_str());
        return "";
    }
    
    if (it->second->config.type != SocketType::SUBSCRIBER) {
        printf("Socket %s is not a subscriber", socket_id.c_str());
        return "";
    }

    if (it->second->socket_fd < 0) {
        printf("Socket %s not initialized or already closed\n", socket_id.c_str());
        return "";
    }
    
    return receiveDataInternal(it->second->socket_fd);
}


bool NetworkManager::initializeSocket(SocketInfo& socket_info) {
    const auto& config = socket_info.config;

    // Create socket
    int socket_type_flag = getSocketTypeFlag(config.type);
    socket_info.socket_fd = nn_socket(AF_SP, socket_type_flag);
    if (socket_info.socket_fd < 0) {
        printf("Failed to create socket: %s\n", nn_strerror(nn_errno()));
        return false;
    }

    // For SUB sockets, set subscription option (subscribe to all topics)
    if (config.type == SocketType::SUBSCRIBER) {
        if (nn_setsockopt(socket_info.socket_fd, NN_SUB, NN_SUB_SUBSCRIBE, "", 0) < 0) {
            printf("Failed to set socket subscription option: %s\n", nn_strerror(nn_errno()));
            nn_close(socket_info.socket_fd);
            socket_info.socket_fd = -1;
            return false;
        }
    }

    if (!validateEndpoint(config.endpoint, config.type)) {
        printf("Invalid endpoint format: %s for socket type: %d\n", 
               config.endpoint.c_str(), static_cast<int>(config.type));
        nn_close(socket_info.socket_fd);
        socket_info.socket_fd = -1;
        return false;
    }

    // Decide whether to bind or connect
    bool success = false;
    bool is_bind = false;

    switch (config.type) {
        case SocketType::PUBLISHER:
        case SocketType::SERVER:
            success = (nn_bind(socket_info.socket_fd, config.endpoint.c_str()) >= 0);
            is_bind = true;
            break;

        case SocketType::SUBSCRIBER:
        case SocketType::CLIENT:
            success = (nn_connect(socket_info.socket_fd, config.endpoint.c_str()) >= 0);
            is_bind = false;
            break;

        default:
            printf("Unknown socket type\n");
            nn_close(socket_info.socket_fd);
            socket_info.socket_fd = -1;
            return false;
    }

    if (!success) {
        printf("Failed to %s socket to %s: %s\n",
               is_bind ? "bind" : "connect",
               config.endpoint.c_str(),
               nn_strerror(nn_errno()));
        nn_close(socket_info.socket_fd);
        socket_info.socket_fd = -1;
        return false;
    }

    printf("Socket %s successful on %s\n",
           is_bind ? "bind" : "connect",
           config.endpoint.c_str());

    return true;
}


void NetworkManager::closeSocket(SocketInfo& socket_info) {
    if (socket_info.socket_fd >= 0) {
        nn_close(socket_info.socket_fd);
        printf("Socket closed");
        socket_info.socket_fd = -1;
    }
}

bool NetworkManager::sendDataInternal(int socket, const std::string& data) {
    int bytes = nn_send(socket, data.c_str(), data.length(), NN_DONTWAIT);
    if (bytes < 0) {
        if (nn_errno() != EAGAIN) {
            printf("Failed to send data: %s", nn_strerror(nn_errno()));
        }
        return false;
    }
    return true;
}

std::string NetworkManager::receiveDataInternal(int socket) {
    // Check if socket is valid
    if (socket < 0) {
        printf("Invalid socket file descriptor\n");
        return "";
    }

    printf("Waiting to receive data on socket %d\n", socket);
    char* buf = nullptr;
    int bytes = nn_recv(socket, &buf, NN_MSG, 0);
    if (bytes < 0 && nn_errno() == EAGAIN) {
        printf("No data available to receive\n");
        return "";
    }

    
    if (bytes < 0) {
        printf("Failed to receive data: %s \n", nn_strerror(nn_errno()));
        return "";
    }
    printf("Received %d bytes of data\n", bytes);
    
    std::string result(buf, bytes);
    nn_freemsg(buf);
    return result;
}

int NetworkManager::getSocketTypeFlag(SocketType type) const {
    switch (type) {
        case SocketType::PUBLISHER:
            return NN_PUB;
        case SocketType::SUBSCRIBER:
            return NN_SUB;
        case SocketType::SERVER:
            return NN_REP;
        case SocketType::CLIENT:
            return NN_REQ;
        default:
            printf("Unsupported socket type: %d\n", static_cast<int>(type));
            return -1;
    }
}

// 在 NetworkManager 类中添加私有方法
bool NetworkManager::validateEndpoint(const std::string& endpoint, SocketType type) {
    // 检查基本格式
    if (endpoint.find("tcp://") != 0) {
        return false;
    }
    
    // 对于 SUBSCRIBER/CLIENT，不能使用通配符地址
    if ((type == SocketType::SUBSCRIBER || type == SocketType::CLIENT)) {
        if (endpoint.find("tcp://*:") == 0 || endpoint.find("tcp://0.0.0.0:") == 0) {
            printf("SUBSCRIBER/CLIENT cannot use wildcard address: %s\n", endpoint.c_str());
            return false;
        }
    }
    
    // 对于 PUBLISHER/SERVER，建议使用通配符地址
    if ((type == SocketType::PUBLISHER || type == SocketType::SERVER)) {
        if (endpoint.find("tcp://localhost:") == 0) {
            printf("WARNING: PUBLISHER/SERVER using localhost may limit connections\n");
        }
    }
    
    return true;
}
