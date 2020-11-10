#include "NGIMUInputDriver.h"

#include "Exception.h"
#include "InverseKinematics.h"
#include "NGIMUData.h"
#include "NGIMUListener.h"
#include "OpenSimUtils.h"
#include "ip/UdpSocket.h"
#include "osc/OscOutboundPacketStream.h"
#include "osc/OscReceivedElements.h"
#include "osc/OscTypes.h"

#include <Common/TimeSeriesTable.h>
#include <SimTKcommon/internal/BigMatrix.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <tuple>
#include <type_traits>
#include <utility>

using namespace std;
using namespace osc;
using namespace std::chrono;
using namespace OpenSimRT;

#define OUTPUT_BUFFER_SIZE 1024

#define TIME_NOW                                                               \
    duration_cast<nanoseconds>(                                                \
            high_resolution_clock::now().time_since_epoch())                   \
            .count()

/*  TODO this is shit!
 */
static TimeTag tv2ntp() {
    auto time = static_cast<double>(TIME_NOW) / PICOSECS_RESOLUTION_DEC;
    auto fracpart =
            static_cast<uint32_t>((time - int(time)) * PICOSECS_RESOLUTION_DEC);
    auto intpart =
            static_cast<uint64_t>(time) + RFC_PROTOCOL + GMT_LOCAL_TIMEZONE;
    return TimeTag((intpart << 32) + fracpart);
}

/**
 * @brief Type-agnostic variadic template function for sending messages to
 * remote IP.
 */
template <typename... Args>
void sendMessage(UdpTransmitSocket& socket, const std::string& command,
                 Args&&... args) {
    // create tuple from args to allow different types
    using ArgsTuple = std::tuple<Args...>;
    ArgsTuple argTuple = {std::forward<Args>(args)...};

    // initialize message stream
    char buffer[OUTPUT_BUFFER_SIZE];
    osc::OutboundPacketStream p(buffer, OUTPUT_BUFFER_SIZE);

    // create the message...
    p << osc::BeginMessage(command.c_str());
    std::apply([&p](const Args&... arg) { ((p << arg), ...); }, argTuple);
    p << osc::EndMessage;

    // ...and send it
    socket.Send(p.Data(), p.Size());
}

/*******************************************************************************/

NGIMUInputDriver::NGIMUInputDriver(const std::vector<std::string>& names,
                                   const std::vector<std::string>& ips,
                                   const std::vector<int>& ports)
        : NGIMUInputDriver() {
    setupInput(names, ips, ports);
}

NGIMUInputDriver::~NGIMUInputDriver() {
    // delete every new object and clear lists
    for (auto listener : listeners) delete listener;
    for (auto socket : udpSockets) delete socket;
    for (auto x : buffer) delete x.second;
    listeners.clear();
    udpSockets.clear();
    buffer.clear();
}

void NGIMUInputDriver::setupInput(const std::vector<std::string>& names,
                                  const std::vector<std::string>& ips,
                                  const std::vector<int>& ports) {
    for (int i = 0; i < ports.size(); ++i) {
        // create new listeners and sockets
        listeners.push_back(new NGIMUListener());
        udpSockets.push_back(new UdpSocket());

        // assign ports and manager to listeners
        listeners[i]->name = names[i];
        listeners[i]->port = ports[i];
        listeners[i]->driver = this;

        // initialize manager buffer
        buffer[ports[i]] =
                new CircularBuffer<CIRCULAR_BUFFER_SIZE, NGIMUData>();
    }
}

void NGIMUInputDriver::setupTransmitters(
        const std::vector<std::string>& remoteIPs,
        const std::vector<int>& remotePorts, const std::string& localIP,
        const std::vector<int>& localPorts) {
    for (int i = 0; i < remoteIPs.size(); ++i) {
        // create socket
        UdpTransmitSocket socket(
                IpEndpointName(remoteIPs[i].c_str(), remotePorts[i]));

        // send commands to imu
        sendMessage(socket, "/time", tv2ntp());
        sendMessage(socket, "/wifi/send/ip", localIP.c_str());
        sendMessage(socket, "/wifi/send/port", localPorts[i]);
        sendMessage(socket, "/wifi/client/lowpower", false);
        sendMessage(socket, "/rate/sensors", 60);
        sendMessage(socket, "/rate/quaternion", 60);
        sendMessage(socket, "/rate/linear", 60);
        sendMessage(socket, "/rate/altitude", 60);
        sendMessage(socket, "/ahrs/magnetometer", false);
        sendMessage(socket, "/wifi/synchronisation/enabled", false);
        sendMessage(socket, "/identify"); // bling!
    }
}

void NGIMUInputDriver::startListening() {
    for (int i = 0; i < listeners.size(); ++i) {
        // get IP and port info from listener
        const auto& ip = listeners[i]->ip;
        const auto& port = listeners[i]->port;

        // bind socket, attach to mux, and run
        udpSockets[i]->Bind(IpEndpointName(ip.c_str(), port));
        mux.AttachSocketListener(udpSockets[i],
                                 dynamic_cast<PacketListener*>(listeners[i]));
        std::cout << "Start Listening on port: " << port << std::endl;
    }
    // start listening..
    mux.RunUntilSigInt();
}

// required to terminate simulation
void NGIMUInputDriver::stopListening() { mux.Break(); }

NGIMUInputDriver::IMUDataFrame NGIMUInputDriver::getFrame() {
    IMUDataFrame results;
    for (const auto& listener : listeners) {
        results.push_back(buffer[listener->port]->get(CIRCULAR_BUFFER_SIZE)[0]);
    }
    return results;
}

// transform all imu dataFrames into a single vector
SimTK::Vector NGIMUInputDriver::asVector(const IMUDataFrame& imuDataFrame) {
    int n = NGIMUData::size();
    SimTK::Vector vec(n * imuDataFrame.size());
    int i = 0;
    for (const auto& imuData : imuDataFrame) {
        vec(i, n) = imuData.asVector();
        i += n;
    }
    return vec;
}

std::vector<NGIMUData> NGIMUInputDriver::fromVector(const SimTK::Vector& v) {
    std::vector<NGIMUData> frame;
    for (int i = 0; i < v.size(); i += NGIMUData::size()) {
        NGIMUData data;
        data.fromVector(v(i, NGIMUData::size()));
        frame.push_back(data);
    }
    return frame;
}

std::vector<std::pair<double, SimTK::Vector>>
NGIMUInputDriver::asPairsOfVectors(const IMUDataFrame& imuDataFrame) {
    std::vector<std::pair<double, SimTK::Vector>> res;
    for (const auto& frame : imuDataFrame) {
        for (const auto& p : frame.getAsPairsOfVectors()) { res.push_back(p); }
    }
    return std::move(res);
}

OpenSim::TimeSeriesTable NGIMUInputDriver::initializeLogger() {
    vector<string> suffixes = {
            "_q1",       "_q2",       "_q3",      "_q4",        "_ax",
            "_ay",       "_az",       "_gx",      "_gy",        "_gz",
            "_mx",       "_my",       "_mz",      "_barometer", "_linAcc_x",
            "_linAcc_y", "_linAcc_z", "_altitude"};

    // create column names for each combination of imu names and measurement
    // suffixes
    vector<string> columnNames;
    for (const auto& listener : listeners) {
        for (const auto& suffix : suffixes) {
            columnNames.push_back(listener->name + suffix);
        }
    }

    // return table
    OpenSim::TimeSeriesTable q;
    q.setColumnLabels(columnNames);
    return q;
}
