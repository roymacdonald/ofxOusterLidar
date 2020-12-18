#pragma once

//#include <json/json.h>
#include "ofJson.h"
#include "compat.h"
#include "stdio.h"

namespace ouster {
namespace sensor {
struct client {
    SOCKET lidar_fd;
    SOCKET imu_fd;
    std::string hostname;
    ofJson meta;
    ~client() {
        socket_close(lidar_fd);
        socket_close(imu_fd);
    }
};
}  // namespace sensor
}  // namespace ouster
