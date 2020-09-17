#pragma once

#include "IMUListener.h"
#include "NGIMUData.h"
#include "osc/OscOutboundPacketStream.h"
#include "osc/OscReceivedElements.h"
#include "osc/OscTypes.h"

#include <chrono>
#include <osc/OscPacketListener.h>

namespace OpenSimRT {
/**
 * @brief xio NGIMU listener implementation
 *
 */
class IMU_API NGIMUListener : public osc::OscPacketListener,
                              public ListenerAdapter<NGIMUData> {
 public:
    osc::uint64 timeTag; // TODO time is measured from 1970

    //// TODO: add more pointers to imu data if required
    NGIMUData::Quaternion* quaternion = nullptr;
    NGIMUData::Sensors* sensors = nullptr;

 protected:
    void ProcessBundle(const osc::ReceivedBundle&,
                       const IpEndpointName&) override;
    void ProcessMessage(const osc::ReceivedMessage& m,
                        const IpEndpointName& remoteEndpoint) override;
};
} // namespace OpenSimRT
