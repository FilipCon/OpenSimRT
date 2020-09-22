#pragma once
#include "InputDriver.h"
#include "internal/IMUExports.h"

#include <string>

namespace OpenSimRT {

/**
 * @brief Interface class for IMU implementations.
 *
 */
template <typename T> class ListenerAdapter {
 public:
    virtual ~ListenerAdapter() = default;
    InputDriver<T>* driver; // pointer to base class manager
    std::string name;
    std::string ip;
    int port;

    // push data to base class manager buffer
    void pushDataToManagerBuffer(const int& port, const T& input) {
        driver->buffer[port]->add(input);
    }
};

} // namespace OpenSimRT
