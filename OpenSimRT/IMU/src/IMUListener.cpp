#include "IMUListener.h"

#include "Manager.h"
#include "osc/OscOutboundPacketStream.h"
#include "osc/OscReceivedElements.h"
#include "osc/OscTypes.h"

#include <chrono>
#include <sys/types.h>

using namespace OpenSimRT;
using namespace osc;
using namespace std;

// void ListenerAdapter::pushDataToManagerBuffer(const int& port,
//                                               const IMUData& input) {
//     manager->buffer[port]->add(input);
// }

void NGIMUListener::ProcessBundle(const ReceivedBundle& b,
                                  const IpEndpointName& remoteEndpoint) {
    timeTag = b.TimeTag();
    for (ReceivedBundle::const_iterator i = b.ElementsBegin();
         i != b.ElementsEnd(); ++i) {
        if (i->IsBundle())
            ProcessBundle(ReceivedBundle(*i), remoteEndpoint);
        else
            ProcessMessage(ReceivedMessage(*i), remoteEndpoint);
    }
}

void NGIMUListener::ProcessMessage(const ReceivedMessage& m,
                                   const IpEndpointName& remoteEndpoint) {
    // every time this function runs, it processes only one message, i.e.
    // /quaternions or /sensors.
    try {
        // get /quaternions
        if (strcmp(m.AddressPattern(), "/quaternion") == 0) {
            ReceivedMessageArgumentStream args = m.ArgumentStream();
            float q1, q2, q3, q4;
            args >> q1 >> q2 >> q3 >> q4 >> osc::EndMessage;
            quaternion = new IMUData::Quaternion{q1, q2, q3, q4, timeTag};
        }

        // get /sensors
        if (strcmp(m.AddressPattern(), "/sensors") == 0) {
            ReceivedMessageArgumentStream args = m.ArgumentStream();
            float gX, gY, gZ; // gyroscope
            float aX, aY, aZ; // acceleration
            float mX, mY, mZ; // magnetometer
            float barometer;
            args >> gX >> gY >> gZ >> aX >> aY >> aZ >> mX >> mY >> mZ >>
                    barometer >> osc::EndMessage;
            sensors = new IMUData::Sensors();
            sensors->acceleration = IMUData::Sensors::Acceleration{aX, aY, aZ};
            sensors->gyroscope = IMUData::Sensors::Gyroscope{gX, gY, gZ};
            sensors->magnetometer = IMUData::Sensors::Magnetometer{mX, mY, mZ};
            sensors->barometer = IMUData::Sensors::Barometer{barometer};
            sensors->timeStamp = timeTag;
        }

        // TODO add more patterns if necessary
        // ...
        //

        // when all messages are processed, push IMU bundle to buffer
        if (quaternion && sensors) {
            auto data = new IMUData();
            data->quaternion = *quaternion;
            data->sensors = *sensors;

            // push data to buffer
            pushDataToManagerBuffer(port, *data);

            // reset pointers
            quaternion = nullptr;
            sensors = nullptr;
        };
    } catch (Exception& e) {
        cout << "error while parsing message: " << m.AddressPattern() << ": "
             << e.what() << "\n";
    }
}
