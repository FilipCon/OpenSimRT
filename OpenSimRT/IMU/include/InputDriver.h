#pragma once

#include "CircularBuffer.h"
#include "internal/IMUExports.h"

#include <Common/TimeSeriesTable.h>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#define CIRCULAR_BUFFER_SIZE 1

namespace OpenSimRT {

// forward declaration
template <typename T> class ListenerAdapter;

/*******************************************************************************/
    template <typename T>
class IMU_API Driver {

    };

/**
 * @brief Base class for different manager implementations.
 *
 */
    template <typename T> class IMU_API InputDriver : Driver<T>{
    friend class ListenerAdapter<T>;

 public:
    // setup listening sockets
    virtual void setupInput(const std::vector<std::string>& names,
                            const std::vector<std::string>& ips,
                            const std::vector<int>& ports) = 0;

    // setup transmitting messages, i.e. sensor settings
    virtual void setupTransmitters(const std::vector<std::string>& remoteIPs,
                                   const std::vector<int>& remotePorts,
                                   const std::string& localIP,
                                   const std::vector<int>& localPorts) = 0;

    // start listening to sockets
    virtual void startListening() = 0;

    // stop listening to sockets
    virtual void stopListening() = 0;

    // obtain observations from IMU data.
    virtual std::pair<double, std::vector<T>> getFrame() = 0;

    // create data logger
    virtual OpenSim::TimeSeriesTable initializeLogger() = 0;

 protected:
    InputDriver() noexcept = default;
    InputDriver& operator=(const InputDriver&) = delete;
    InputDriver(const InputDriver&) = delete;
    virtual ~InputDriver() = default;

    // pointers to generic listeners using the adapter interface class
    std::vector<ListenerAdapter<T>*> listeners;

    // data buffer for IMU data from each port.
    std::map<int, CircularBuffer<CIRCULAR_BUFFER_SIZE, T>*> buffer;
};

} // namespace OpenSimRT
