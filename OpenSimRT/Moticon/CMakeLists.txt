# configure oscpack
set(OSCPACK_DIR $ENV{OSCPACK_DIR} CACHE PATH
    "Top-level directory of oscpack_1_1_0")
set(OSCPACK_INCLUDE_DIRS ${OSCPACK_DIR}/include)
find_library(oscpack
  NAMES oscpack
  PATHS ${OSCPACK_DIR}/lib
  PATH_SUFFIXES lib)
set(OSCPACK_LIBRARIES ${oscpack} Common)

if(WIN32)
  set(DEPENDENCY_LIBRARIES ${DEPENDENCY_LIBRARIES} Ws2_32 winmm)
endif()

# files
file(GLOB includes include/*.h)
file(GLOB sources src/*.cpp)
# file(GLOB tests tests/*.cpp)

message(${includes})
message(${sources})

# dependencies
include_directories(include/)
include_directories(../Common/include/)
include_directories(${OSCPACK_INCLUDE_DIRS})
set(DEPENDENCY_LIBRARIES ${OpenSim_LIBRARIES} ${OSCPACK_LIBRARIES})

# dynamic library
set(target Moticon)

addLibrary(
    KIT ${target}
    SOURCES ${sources}
    INCLUDES ${includes}
    LINKLIBS ${DEPENDENCY_LIBRARIES}
)

# # tests
# addTests(
#     TESTPROGRAMS ${tests}
#     LINKLIBS ${target} ${DEPENDENCY_LIBRARIES}
# )

# # applications
# addApplications(
#     SOURCES ${applications}
#     LINKLIBS ${target} ${DEPENDENCY_LIBRARIES}
# )
