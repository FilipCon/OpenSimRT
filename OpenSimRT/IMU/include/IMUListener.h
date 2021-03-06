/**
 * @file IMUListener.h
 *
 * @brief Implements a generic interface for IMU Listeners. It is based on the
 * Adapter Design Pattern to decouple the implementation of IMU data acquisition
 * drivers from the IMU dependencies (e.g., Wi-Fi, Bluetooth, OSC protocol and
 * oscpack library, etc).
 *
 * @author Filip Konstantinos <filip.k@ece.upatras.gr>
 */
#pragma once
#include "InputDriver.h"
#include "internal/IMUExports.h"

#include <memory>
#include <string>

namespace OpenSimRT {

/**
 * @brief Interface class for IMU listener implementations.
 *
 */
template <typename T> class ListenerAdapter {
 public:
    ListenerAdapter() = default;
    virtual ~ListenerAdapter() = default;
    std::shared_ptr<InputDriver<T>> driver; // pointer to base class imu driver
    std::string name;       // imu name
    std::string ip;         // imu ip
    int port;               // imu port

    /**
     * Push IMU data (indexed by the listener's port) to the driver's buffer.
     */
    void pushDataToManagerBuffer(const int& port, const T& input) {
        driver->buffer[port]->add(input);
    }
};

} // namespace OpenSimRT