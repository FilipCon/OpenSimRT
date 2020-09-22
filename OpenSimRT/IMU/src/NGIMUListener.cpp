#include "NGIMUListener.h"

#include "Exception.h"

#include <SimTKcommon/SmallMatrix.h>
#include <SimTKcommon/internal/Quaternion.h>
#include <memory>

using namespace std;
using namespace osc;
using namespace OpenSimRT;

NGIMUListener::NGIMUListener() {
    quaternion = nullptr;
    sensors = nullptr;
    linear = nullptr;
    altitude = nullptr;
}

NGIMUListener::~NGIMUListener() {
    delete quaternion;
    delete sensors;
    delete linear;
    delete altitude;
}

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
            quaternion = new NGIMUData::Quaternion{
                    SimTK::Quaternion(q1, q2, q3, q4), timeTag};
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
            sensors = new NGIMUData::Sensors();
            sensors->acceleration =
                    NGIMUData::Sensors::Acceleration{SimTK::Vec3(aX, aY, aZ)};
            sensors->gyroscope =
                    NGIMUData::Sensors::Gyroscope{SimTK::Vec3(gX, gY, gZ)};
            sensors->magnetometer =
                    NGIMUData::Sensors::Magnetometer{SimTK::Vec3(mX, mY, mZ)};
            sensors->barometer = barometer;
            sensors->timeStamp = timeTag;
        }

        if (strcmp(m.AddressPattern(), "/linear") == 0) {
            ReceivedMessageArgumentStream args = m.ArgumentStream();
            float ax, ay, az; // linear acceleration
            args >> ax >> ay >> az >> osc::EndMessage;
            linear = new NGIMUData::LinearAcceleration{SimTK::Vec3(ax, ay, az),
                                                       timeTag};
        }

        if (strcmp(m.AddressPattern(), "/altitude") == 0) {
            ReceivedMessageArgumentStream args = m.ArgumentStream();
            float x;
            args >> x >> osc::EndMessage;
            altitude = new NGIMUData::Altitude{x, timeTag};
        }

        // TODO add more patterns if necessary
        // ...
        //

        // when all messages are processed, push IMU bundle to buffer
        if (quaternion && sensors && linear && altitude) {
            auto data = make_unique<NGIMUData>();
            data->quaternion = *quaternion;
            data->sensors = *sensors;
            data->linear = *linear;
            data->altitude = *altitude;

            // push data to buffer
            pushDataToManagerBuffer(port, *data);

            // clear heap and reset pointers
            delete quaternion;
            delete sensors;
            delete linear;
            delete altitude;
            quaternion = nullptr;
            sensors = nullptr;
            linear = nullptr;
            altitude = nullptr;
        };
    } catch (Exception& e) {
        cout << "error while parsing message: " << m.AddressPattern() << ": "
             << e.what() << "\n";
    }
}
