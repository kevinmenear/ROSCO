cmake_minimum_required(VERSION 3.6)
project(ROSCO VERSION 2.9.0 LANGUAGES C CXX)

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Choose the build type: Debug RelWithDebInfo Release" FORCE)
endif()

# VIT: -ffp-contract=off for bit-identical verification during translation phase.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ffp-contract=off")

set(SOURCES
    src/activewakecontrol.cpp
    src/aerodyntorque.cpp
    src/cablecontrol.cpp
    src/checkinputs.cpp
    src/colemantransform.cpp
    src/colemantransforminverse.cpp
    src/computevariablessetpoints.cpp
    src/debug.cpp
    src/discon.cpp
    src/extcontroller.cpp
    src/flapcontrol.cpp
    src/floatingfeedback.cpp
    src/foreaftdamping.cpp
    src/hpfilter.cpp
    src/identity.cpp
    src/interp1d.cpp
    src/interp2d.cpp
    src/ipc.cpp
    src/lpfilter.cpp
    src/notchfilter.cpp
    src/notchfilterslopes.cpp
    src/picontroller.cpp
    src/pidcontroller.cpp
    src/piicontroller.cpp
    src/pitchcontrol.cpp
    src/pitchsaturation.cpp
    src/powercontrolsetpoints.cpp
    src/prefiltermeasuredsignals.cpp
    src/ratelimit.cpp
    src/readavrswap.cpp
    src/readcontrolparameterfilesub.cpp
    src/readcpfile.cpp
    src/readrestartfile.cpp
    src/refspeedexclusion.cpp
    src/rescontroller.cpp
    src/saturate.cpp
    src/seclpfilter.cpp
    src/seclpfilter_vel.cpp
    src/setparameters.cpp
    src/setpointsmoother.cpp
    src/shutdown.cpp
    src/sigma.cpp
    src/startup.cpp
    src/statemachine.cpp
    src/structuralcontrol.cpp
    src/unwrap.cpp
    src/updatezeromq.cpp
    src/variablespeedcontrol.cpp
    src/windspeedestimator.cpp
    src/wrap_180.cpp
    src/wrap_360.cpp
    src/writerestartfile.cpp
    src/yawratecontrol.cpp
)

# ZMQ Library
find_package(PkgConfig)
pkg_check_modules(PC_ZeroMQ libzmq)
if(PC_ZeroMQ_FOUND)
    find_path(ZeroMQ_INCLUDE_DIR NAMES zmq.h PATHS ${PC_ZeroMQ_INCLUDE_DIRS})
    find_library(ZeroMQ_LIBRARY NAMES zmq PATHS ${PC_ZeroMQ_LIBRARY_DIRS})
    include_directories(${ZeroMQ_INCLUDE_DIR})
    add_compile_options(-I${ZeroMQ_INCLUDE_DIR} -l${ZeroMQ_LIBRARY} -fPIC)
    add_library(zmq_client OBJECT src/zmq_client.c)
    add_definitions(-DZMQ_CLIENT="TRUE")
    set(SOURCES ${SOURCES} $<TARGET_OBJECTS:zmq_client>)
endif()

add_library(discon SHARED ${SOURCES})
target_include_directories(discon PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../translations)
target_link_libraries(discon PRIVATE stdc++ dl)

if(PC_ZeroMQ_FOUND)
    target_include_directories(discon PUBLIC ${ZeroMQ_INCLUDE_DIR})
    target_link_libraries(discon PUBLIC ${ZeroMQ_LIBRARY})
endif()

install(TARGETS discon
  EXPORT "${CMAKE_PROJECT_NAME}Libraries"
  RUNTIME DESTINATION lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

set(linuxDefault ${CMAKE_INSTALL_PREFIX} STREQUAL "/usr/local")
set(windowsDefault ${CMAKE_INSTALL_PREFIX} STREQUAL "C:\\Program Files (x86)")
if(${linuxDefault} OR ${windowsDefault})
  set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/../")
endif()
