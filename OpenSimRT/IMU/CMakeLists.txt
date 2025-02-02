# configure oscpack
set(OSCPACK_DIR $ENV{OSCPACK_DIR} CACHE PATH
    "Top-level directory of oscpack_1_1_0")
set(OSCPACK_INCLUDE_DIRS ${OSCPACK_DIR}/include)
include_directories(${OSCPACK_INCLUDE_DIRS})
find_library(oscpack
  NAMES oscpack
  PATHS ${OSCPACK_DIR}/lib
  PATH_SUFFIXES lib)
set(OSCPACK_LIBRARIES ${oscpack})
set(DEPENDENCY_LIBRARIES ${OpenSim_LIBRARIES} ${OSCPACK_LIBRARIES} RealTime Common)

if(WIN32)
  set(DEPENDENCY_LIBRARIES ${DEPENDENCY_LIBRARIES} Ws2_32 winmm)
endif()

# global files
file(GLOB includes include/*.h)
file(GLOB sources src/*.cpp)
# file(GLOB applications applications/*.cpp)
file(
  GLOB applications
  applications/OnlineLowerLimbIMUIK.cpp
  applications/OnlineUpperLimbIMUIK.cpp
)
# file(GLOB tests tests/*.cpp)
file(GLOB tests
  tests/TestLowerLimbIMUIKFromFile.cpp
  tests/TestUpperLimbIMUIKFromFile.cpp
)

# dependencies
include_directories(include/)
include_directories(../RealTime/include/)
include_directories(../Common/include/)

# dynamic library
set(target IMU)

addlibrary(
  KIT ${target}
  LINKLIBS ${DEPENDENCY_LIBRARIES}
  INCLUDES ${includes}
  SOURCES ${sources})

# tests
addtests(
  TESTPROGRAMS ${tests}
  LINKLIBS ${target} ${DEPENDENCY_LIBRARIES})

# applications
addapplications(
  SOURCES ${applications}
  LINKLIBS ${target} ${DEPENDENCY_LIBRARIES}
)
