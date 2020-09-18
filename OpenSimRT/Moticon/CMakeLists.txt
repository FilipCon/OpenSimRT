# files
file(GLOB includes include/*.h)
file(GLOB sources src/*.cpp)
file(GLOB tests tests/*.cpp)

# dependencies
include_directories(include/)
include_directories(../Common/include/)
set(DEPENDENCY_LIBRARIES ${OpenSim_LIBRARIES} Common)

# dynamic library
set(target Moticon)
addLibrary(
    KIT ${target}
    SOURCES ${sources}
    INCLUDES ${includes}
    LINKLIBS ${DEPENDENCY_LIBRARIES}
)

# tests
addTests(
    TESTPROGRAMS ${tests}
    LINKLIBS ${target} ${DEPENDENCY_LIBRARIES}
)

# # applications
# addApplications(
#     SOURCES ${applications}
#     LINKLIBS ${target} ${DEPENDENCY_LIBRARIES}
# )