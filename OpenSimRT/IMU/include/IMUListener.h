#ifndef LISTENER_H
#define LISTENER_H

#include "Manager.h"
#include "internal/IMUExports.h"
#include "osc/OscPacketListener.h"
#include "osc/OscTypes.h"

#include <iostream>

namespace OpenSimRT {

/**
 * @brief Interface class for IMU implementations.
 *
 */
template <typename T> class ListenerAdapter {
 public:
    Manager<T>* manager; // pointer to base class manager
    int port;
    std::string ip;

    // push data to base class manager buffer
    void pushDataToManagerBuffer(const int& port, const T& input) {
        manager->buffer[port]->add(input);
    }

 protected:
    virtual ~ListenerAdapter(){};
};

/**
 * @brief xio NGIMU listener implementation
 *
 */
class IMU_API NGIMUListener : public osc::OscPacketListener,
                              public ListenerAdapter<IMUData> {
 public:
    osc::uint64 timeTag; // TODO time is measured from 1970

    //// TODO: add more pointers to imu data if required
    IMUData::Quaternion* quaternion = nullptr;
    IMUData::Sensors* sensors = nullptr;

 protected:
    void ProcessBundle(const osc::ReceivedBundle&,
                       const IpEndpointName&) override;
    void ProcessMessage(const osc::ReceivedMessage& m,
                        const IpEndpointName& remoteEndpoint) override;

};

} // namespace OpenSimRT
#endif
