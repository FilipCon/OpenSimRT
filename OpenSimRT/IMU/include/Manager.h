#ifndef MANAGER_H
#define MANAGER_H

#include "CircularBuffer.h"
#include "Simulation.h"
#include "internal/IMUExports.h"
#include "ip/UdpSocket.h"
#include "osc/OscTypes.h"

#include <iostream>
#include <map>
#include <string>
#include <vector>

#define CIRCULAR_BUFFER_SIZE 1

namespace OpenSimRT {

// forward declaration
template <typename T> class ListenerAdapter;

// IMU data container
struct IMU_API IMUData {
    struct Quaternion {
        double q1, q2, q3, q4;
        osc::uint64 timeStamp;
    };
    struct Sensors {
        struct Acceleration {
            double ax, ay, az;
        };
        struct Gyroscope {
            double gx, gy, gz;
        };
        struct Magnetometer {
            double mx, my, mz;
        };
        struct Barometer {
            double barometer;
        };
        Acceleration acceleration; // in g
        Gyroscope gyroscope;       // in circ/sec
        Magnetometer magnetometer;
        Barometer barometer;
        osc::uint64 timeStamp;
    };
    Quaternion quaternion;
    Sensors sensors;
};

/*******************************************************************************/

/**
 * @brief Base class for different manager implementations.
 *
 */
template <typename T> class IMU_API Manager {
    friend class ListenerAdapter<T>;

 public:
    // setup listening sockets
    virtual void setupListeners(const std::vector<std::string>& ips,
                                const std::vector<int>& ports) = 0;

    // setup transmitting messages, i.e. sensor settings
    virtual void setupTransmitters(const std::vector<std::string>& remoteIPs,
                                   const std::vector<int>& remotePorts,
                                   const std::string& localIP,
                                   const std::vector<int>& localPorts) = 0;

    // start listening to sockets
    virtual void startListeners() = 0;

    // stop listening to sockets
    virtual void stopListeners() = 0;

    // obtain observations from IMU data.
    virtual std::pair<double, std::vector<T>> getObservations() = 0;

 protected:
    Manager() noexcept = default;
    Manager& operator=(const Manager&) = delete;
    Manager(const Manager&) = delete;
    virtual ~Manager() = default;

    // pointers to generic listeners using the adapter interface class
    std::vector<ListenerAdapter<T>*> listeners;

    // data buffer for IMU data from each port.
    std::map<int, CircularBuffer<CIRCULAR_BUFFER_SIZE, T>*> buffer;
};

/**
 * @brief xio NGIMU Manager implementation
 */
class IMU_API NGIMUManager : public Manager<IMUData> {
 public:
    NGIMUManager() = default;
    NGIMUManager(const std::vector<std::string>&, const std::vector<int>&);

    // setup listening sockets
    virtual void setupListeners(const std::vector<std::string>& ips,
                                const std::vector<int>& ports) override;

    // setup transmitting messages to IMU
    virtual void setupTransmitters(const std::vector<std::string>& remoteIPs,
                                   const std::vector<int>& remotePorts,
                                   const std::string& localIP,
                                   const std::vector<int>& localPorts) override;

    virtual void startListeners() override;
    virtual void stopListeners() override;
    virtual std::pair<double, std::vector<IMUData>> getObservations() override;

 private:
    SocketReceiveMultiplexer mux;
    std::vector<UdpSocket*> udpSockets;
    double initFrameTime;
};
} // namespace OpenSimRT
#endif
