/**
 * @file MarkerReconstruction.h
 * @author Filip Konstantinos (filip.k@ece.upatras.gr)
 * @brief Missing Marker Reconstruction module
 * @version 0.1
 * @date 2019-12-16
 *
 * @copyright Copyright (c) 2019
 *
 */
#pragma once
#include "InverseKinematics.h"
#include "internal/RealTimeExports.h"

#include <Common/TimeSeriesTable.h>
#include <SimTKcommon/SmallMatrix.h>
#include <iostream>
#include <utility>
namespace OpenSimRT {
class RealTime_API MarkerReconstruction {
 public:

    MarkerReconstruction(
            const OpenSim::Model& model,
            const std::vector<InverseKinematics::MarkerTask>& markerTasks);
    /**
     * Determine if an input frame is valid.
     */
    bool isValidFrame(const InverseKinematics::Input& frame);

    void solve(SimTK::Array_<SimTK::Vec3>& currentObservations);

    OpenSim::TimeSeriesTable_<SimTK::Vec3> initializeLogger();

    template <typename F> void init(F&& dataAcquisitionFunction) {
        // ignore invalid first frames.
        decltype(dataAcquisitionFunction()) acquisitionData;
        do {
            acquisitionData = std::forward<F>(dataAcquisitionFunction)();
        } while (!isValidFrame(acquisitionData.IkFrame));

        previousObservations = acquisitionData.IkFrame.markerObservations;
    }

 private:
    typedef std::map<std::string, std::map<std::string, SimTK::Vec3>>
            DistanceTable;
    struct Circle {
        SimTK::Vec3 origin;
        SimTK::Vec3 normal;
        double radius;
    };

    struct Sphere {
        SimTK::Vec3 origin;
        double radius;
    };

    OpenSim::Model model;
    DistanceTable markerDistanceTable;
    std::multimap<std::string, std::string> markersPerBodyMap;
    std::vector<std::string> observationOrder;
    SimTK::Array_<SimTK::Vec3> previousObservations;

    Circle* sphereSphereIntersection(const Sphere& c1, const Sphere& c2);
    SimTK::Vec3 closestPointToCircle(const SimTK::Vec3& vec, const Circle* c);
    std::vector<int>
    findClosestMarkers(const std::string& mMarkerName,
                       const SimTK::Array_<SimTK::Vec3>& currentObservations,
                       int numMarkers);
    // std::vector<int> findTwoClosestMarkers(const std::string& mMarkerName,
    //         const SimTK::Array_<SimTK::Vec3>& currentObservations);
    // std::vector<SimTK::Vec3> circleCircleIntersection(const Circle& c1, const
    // Circle& c2);
};
} // namespace OpenSimRT
