#include "NGIMUListener.h"

#include "Exception.h"

#include <SimTKcommon/SmallMatrix.h>
#include <SimTKcommon/internal/Quaternion.h>
#include <cstring>
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
        // the 32-first bits are the #seconds since Jan 1 1900 00:00 GMT. To
        // start from 1970 (Unix epoch), remove the 70 years in seconds. Add 3
        // hours since local timezone is GMT+03:00
        auto time = (timeTag >> 32) +
                    double(timeTag & 0xFFFFFFFF) / PICOSECS_RESOLUTION_BIN -
                    RFC_PROTOCOL - GMT_LOCAL_TIMEZONE;

        // get /quaternions
        if (strcmp(m.AddressPattern(), "/quaternion") == 0) {
            ReceivedMessageArgumentStream args = m.ArgumentStream();
            float q1, q2, q3, q4;
            args >> q1 >> q2 >> q3 >> q4 >> osc::EndMessage;
            quaternion = new NGIMUData::Quaternion{
                    SimTK::Quaternion(q1, q2, q3, q4), time};
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
            sensors->acceleration = SimTK::Vec3(aX, aY, aZ);
            sensors->gyroscope = SimTK::Vec3(gX, gY, gZ);
            sensors->magnetometer = SimTK::Vec3(mX, mY, mZ);
            sensors->barometer = SimTK::Vec1(barometer);
            sensors->timeStamp = time;
        }

        if (strcmp(m.AddressPattern(), "/linear") == 0) {
            ReceivedMessageArgumentStream args = m.ArgumentStream();
            float ax, ay, az; // linear acceleration
            args >> ax >> ay >> az >> osc::EndMessage;
            linear = new NGIMUData::LinearAcceleration{SimTK::Vec3(ax, ay, az),
                                                       time};
        }

        if (strcmp(m.AddressPattern(), "/altitude") == 0) {
            ReceivedMessageArgumentStream args = m.ArgumentStream();
            float x;
            args >> x >> osc::EndMessage;
            altitude = new NGIMUData::Altitude{SimTK::Vec1(x), time};
        }

        // TODO add more patterns if necessary
        // ...
        //

        if (strcmp(m.AddressPattern(), "/button") == 0) {
            driver->stopListening(); // TODO do something else with it
            THROW_EXCEPTION(
                    "Destruction Button is Pressed! Goodbye cruel word!");
        }

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
