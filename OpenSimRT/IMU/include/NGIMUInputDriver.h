#pragma once
#include "NGIMUData.h"
#include "InputDriver.h"
#include "ip/UdpSocket.h"

/**
 * @brief xio NGIMU Manager implementation
 */

namespace OpenSimRT {

class IMU_API NGIMUInputDriver : public InputDriver<NGIMUData> {
 public:
    typedef std::pair<double, std::vector<NGIMUData>> IMUDataFrame;
    NGIMUInputDriver() = default;
    NGIMUInputDriver(const std::vector<std::string>& names,
                     const std::vector<std::string>&, const std::vector<int>&);

    // setup listening sockets
    virtual void setupInput(const std::vector<std::string>& imuLabels,
                            const std::vector<std::string>& ips,
                            const std::vector<int>& ports) override;

    // setup transmitting messages to IMU
    virtual void setupTransmitters(const std::vector<std::string>& remoteIPs,
                                   const std::vector<int>& remotePorts,
                                   const std::string& localIP,
                                   const std::vector<int>& localPorts) override;

    virtual void startListening() override;
    virtual void stopListening() override;
    virtual IMUDataFrame getFrame() override;
    virtual OpenSim::TimeSeriesTable initializeLogger() override;

    SimTK::Vector asVector(const IMUDataFrame& imuDataFrame);

 private:
    SocketReceiveMultiplexer mux;
    std::vector<UdpSocket*> udpSockets;
    double initFrameTime = 0;
};
} // namespace OpenSimRT
