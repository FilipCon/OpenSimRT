#include "Manager.h"

#include "CircularBuffer.h"
#include "IMUListener.h"
#include "InverseKinematics.h"
#include "osc/OscOutboundPacketStream.h"
#include "osc/OscReceivedElements.h"
#include "osc/OscTypes.h"

#include <algorithm>
#include <iterator>
#include <tuple>
#include <type_traits>
#include <utility>

using namespace std;
using namespace osc;
using namespace std::chrono;
using namespace OpenSimRT;

#define OUTPUT_BUFFER_SIZE 1024

/*******************************************************************************/

// operator overload
std::ostream& operator<<(std::ostream& os, const IMUData::Quaternion& q) {
    return os << q.q1 << " " << q.q2 << " " << q.q3 << " " << q.q4;
}

/**
 * @brief Type-agnostic variadic template function for sending messages to
 * remote IP.
 */
template <typename... Args>
void sendMessage(UdpTransmitSocket& socket, const std::string& command,
                 Args&&... args) {
    // create tuple from args to allow different types
    using ArgsTuple = std::tuple<std::decay_t<Args>...>;
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

NGIMUManager::NGIMUManager(const std::vector<std::string>& ips,
                           const std::vector<int>& ports)
        : NGIMUManager() {
    initFrameTime = 0;
    setupListeners(ips, ports);
}

void NGIMUManager::setupListeners(const std::vector<std::string>& ips,
                                  const std::vector<int>& ports) {
    for (int i = 0; i < ports.size(); ++i) {
        // create new listeners and sockets
        listeners.push_back(new NGIMUListener());
        udpSockets.push_back(new UdpSocket());

        // assign ports and manager to listeners
        listeners[i]->port = ports[i];
        listeners[i]->manager = this;

        // initialize manager buffer
        buffer[ports[i]] = new CircularBuffer<CIRCULAR_BUFFER_SIZE, IMUData>();
    }
}

void NGIMUManager::setupTransmitters(const std::vector<std::string>& remoteIPs,
                                     const std::vector<int>& remotePorts,
                                     const std::string& localIP,
                                     const std::vector<int>& localPorts) {
    for (int i = 0; i < remoteIPs.size(); ++i) {
        // create socket
        UdpTransmitSocket socket(
                IpEndpointName(remoteIPs[i].c_str(), remotePorts[i]));

        // send commands to imu
        sendMessage(socket, "/wifi/send/ip", localIP.c_str());
        sendMessage(socket, "/wifi/send/port", localPorts[i]);
        sendMessage(socket, "/rate/sensors", 60);
        sendMessage(socket, "/rate/quaternion", 60);
        sendMessage(socket, "/identify"); // bling!
    }
}

void NGIMUManager::startListeners() {
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

void NGIMUManager::stopListeners() { mux.Break(); }

pair<double, vector<IMUData>> NGIMUManager::getObservations() {
    vector<IMUData> results;
    double time;
    for (const auto& listener : listeners) {
        results.push_back(buffer[listener->port]->get(CIRCULAR_BUFFER_SIZE)[0]);

        const auto& timeStamp = results.back().quaternion.timeStamp;
        unsigned last32bitsValue = timeStamp & 0xFFFFFFFF;
        unsigned first32bitsValue = (timeStamp >> 32) & 0xFFFFFFFF;

        time = first32bitsValue + last32bitsValue * pow(2.0, -32) -
               initFrameTime;
        if (initFrameTime == 0) initFrameTime = time;
    }
    return make_pair(time, results);
}
