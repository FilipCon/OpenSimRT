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
    InputDriver<T>* driver; // pointer to base class manager
    int port;
    std::string ip;
    std::string name;

    // push data to base class manager buffer
    void pushDataToManagerBuffer(const int& port, const T& input) {
        driver->buffer[port]->add(input);
    }

 protected:
    virtual ~ListenerAdapter(){};
};

} // namespace OpenSimRT
