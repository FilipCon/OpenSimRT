#pragma once

#include "internal/MoticonExports.h"

#include <SimTKcommon.h>

namespace OpenSimRT {
struct Moticon_API MoticonInsoleSizes {
    struct Size {
        double length;
        double width;
    };

    // TODO add more sizes
    static Size size4() { return Size{0.2486, 0.089}; }
};

struct Moticon_API MoticonReceivedBundle {
    double timestamp;
    struct Measurement {
        SimTK::Vec3 acceleration;
        SimTK::Vec3 angularRate;
        SimTK::Vec2 cop;
        SimTK::Vec<16> pressure; // 16 sensors
        double totalForce;
    } left, right;
    static constexpr int size() { return 50; };
    SimTK::Vector asVector() const;
    void fromVector(const SimTK::Vector&);
};
} // namespace OpenSimRT
