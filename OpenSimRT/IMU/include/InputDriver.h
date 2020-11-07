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

/**
 * @brief Base class for different manager implementations.
 *
 */
template <typename T> class IMU_API InputDriver {
    friend class ListenerAdapter<T>;

 public:
    // start listening to sockets
    virtual void startListening() = 0;

    // stop listening to sockets
    virtual void stopListening() = 0;

    // obtain observations from IMU data.
    virtual std::vector<T> getFrame() = 0;

 protected:
    InputDriver() noexcept {};
    InputDriver& operator=(const InputDriver&) = delete;
    InputDriver(const InputDriver&) = delete;
    virtual ~InputDriver() = default;

    // pointers to generic listeners using the adapter interface class
    std::vector<ListenerAdapter<T>*> listeners;

    // data buffer for IMU data from each port.
    std::map<int, CircularBuffer<CIRCULAR_BUFFER_SIZE, T>*> buffer;
};

} // namespace OpenSimRT
