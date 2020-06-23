/**
 * @file OpenSimUtils.h
 *
 * \brief Common OpenSim operations and functions.
 *
 * @author Dimitar Stanev <jimstanev@gmail.com>
 */
#ifndef OPENSIM_UTILS_H
#define OPENSIM_UTILS_H

#include "internal/CommonExports.h"

#include <Common/TimeSeriesTable.h>
#include <OpenSim/Simulation/Model/Model.h>

#ifdef PROFILING
#    define PROFILE_SCOPE(name) InstrumentationTimer timer##__LINE__(name)
#else
#    define PROFILE_SCOPE(name)
#endif
#define PROFILE_FUNCTION() PROFILE_SCOPE(__PRETTY_FUNCTION__)

namespace OpenSimRT {

struct ProfileResult {
    std::string Name;
    long long Start, End;
    uint32_t ThreadID;
};

struct InstrumentationSession {
    std::string Name;
};

class Instrumentor {
 private:
    InstrumentationSession* m_CurrentSession;
    std::ofstream m_OutputStream;
    int m_ProfileCount;
    std::mutex m_mu;

 public:
    Instrumentor() : m_CurrentSession(nullptr), m_ProfileCount(0) {}

    void BeginSession(const std::string& name, const std::string& filepath) {
        m_OutputStream.open(filepath);
        WriteHeader();
        m_CurrentSession = new InstrumentationSession{name};
    }

    void EndSession() {
        WriteFooter();
        m_OutputStream.close();
        delete m_CurrentSession;
        m_CurrentSession = nullptr;
        m_ProfileCount = 0;
    }

    void WriteProfile(const ProfileResult& result) {
        std::lock_guard<std::mutex> locker(m_mu);
        if (m_ProfileCount++ > 0) m_OutputStream << ",";

        std::string name = result.Name;
        std::replace(name.begin(), name.end(), '"', '\'');

        m_OutputStream << "{";
        m_OutputStream << "\"cat\":\"function\",";
        m_OutputStream << "\"dur\":" << (result.End - result.Start) << ',';
        m_OutputStream << "\"name\":\"" << name << "\",";
        m_OutputStream << "\"ph\":\"X\",";
        m_OutputStream << "\"pid\":0,";
        m_OutputStream << "\"tid\":" << result.ThreadID << ",";
        m_OutputStream << "\"ts\":" << result.Start;
        m_OutputStream << "}";

        m_OutputStream.flush();
    }

    void WriteHeader() {
        m_OutputStream << "{\"otherData\": {},\"traceEvents\":[";
        m_OutputStream.flush();
    }

    void WriteFooter() {
        m_OutputStream << "]}";
        m_OutputStream.flush();
    }

    static Instrumentor& Get() {
        static Instrumentor instance;
        return instance;
    }
};

class InstrumentationTimer {
 public:
    InstrumentationTimer(const char* name) : m_Name(name), m_Stopped(false) {
        m_StartTimepoint = std::chrono::high_resolution_clock::now();
    }

    ~InstrumentationTimer() {
        if (!m_Stopped) Stop();
    }

    void Stop() {
        auto endTimepoint = std::chrono::high_resolution_clock::now();

        long long start =
                std::chrono::time_point_cast<std::chrono::microseconds>(
                        m_StartTimepoint)
                        .time_since_epoch()
                        .count();
        long long end = std::chrono::time_point_cast<std::chrono::microseconds>(
                                endTimepoint)
                                .time_since_epoch()
                                .count();

        uint32_t threadID =
                std::hash<std::thread::id>{}(std::this_thread::get_id());
        Instrumentor::Get().WriteProfile({m_Name, start, end, threadID});

        m_Stopped = true;
    }

 private:
    const char* m_Name;
    std::chrono::time_point<std::chrono::high_resolution_clock>
            m_StartTimepoint;
    bool m_Stopped;
};

// Type definitions from a moment arm function that accepts a vector and returns
// a matrix.
typedef SimTK::Matrix (*MomentArmFunctionT)(const SimTK::Vector& q);

struct Common_API OpenSimUtils {
    // Generates a unique identifier
    static int generateUID();
    // Extract model's coordinate names in multibody tree order.
    static std::vector<std::string>
    getCoordinateNamesInMultibodyTreeOrder(const OpenSim::Model& model);
    // Extract model's coordinate names in normal order.
    static std::vector<std::string>
    getCoordinateNames(const OpenSim::Model& model);
    // Extract model's muscle names.
    static std::vector<std::string> getMuscleNames(const OpenSim::Model& model);
    // Extract model's actuator names.
    static std::vector<std::string>
    getActuatorNames(const OpenSim::Model& model);
    // Disable actuators.
    static void disableActuators(OpenSim::Model& model);
    // Remove actuators.
    static void removeActuators(OpenSim::Model& model);
    // Get ordered generalized coordinates from storage.
    static OpenSim::TimeSeriesTable
    getMultibodyTreeOrderedCoordinatesFromStorage(const OpenSim::Model& model,
                                                  const std::string stoFilePath,
                                                  double samplingInterval);
    // Load moment arm from a dynamic library
    static MomentArmFunctionT
    getMomentArmFromDynamicLibrary(const OpenSim::Model& model,
                                   std::string libraryPath);
};

} // namespace OpenSimRT

#endif
