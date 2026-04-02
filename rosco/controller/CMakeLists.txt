cmake_minimum_required(VERSION 3.6)
project(ROSCO VERSION 2.9.0 LANGUAGES Fortran C)
enable_language(CXX)

set(CMAKE_Fortran_MODULE_DIRECTORY "${CMAKE_BINARY_DIR}/ftnmods")

if (NOT CMAKE_BUILD_TYPE)
  # Sets the optimization level to -O2 and includes -g for debugging
  set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Choose the build type: Debug RelWithDebInfo Release" FORCE)
endif()

message(STATUS "CMAKE_Fortran_COMPILER_ID = ${CMAKE_Fortran_COMPILER_ID}")
if(APPLE OR UNIX)
# Enable .dll export
if (CMAKE_Fortran_COMPILER_ID MATCHES "^Intel")
set(CMAKE_Fortran_FLAGS "${CMAKE_Fortran_FLAGS} -DIMPLICIT_DLLEXPORT -r8 -double-size 64 -cpp -no-wrap-margin")
elseif (CMAKE_Fortran_COMPILER_ID STREQUAL "Flang")
add_definitions(-DFLANG_COMPILER)
set(CMAKE_Fortran_FLAGS "${CMAKE_Fortran_FLAGS} -DIMPLICIT_DLLEXPORT -fno-backslash -cpp -fdefault-real-8")  
else()
set(CMAKE_Fortran_FLAGS "${CMAKE_Fortran_FLAGS} -DIMPLICIT_DLLEXPORT -ffree-line-length-0 -fdefault-real-8 -fdefault-double-8 -cpp -ffp-contract=off")
# VIT: -ffp-contract=off disables FMA contraction so that gfortran and g++
# produce bit-identical results for equivalent expressions during translation verification.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ffp-contract=off")
endif()
elseif (WIN32)
if (CMAKE_Fortran_COMPILER_ID STREQUAL "GNU")
# Ensure static linking to avoid requiring Fortran runtime dependencies
set(CMAKE_Fortran_FLAGS "${CMAKE_Fortran_FLAGS} -ffree-line-length-0 -static-libgcc -static-libgfortran -static -fdefault-real-8 -fdefault-double-8 -cpp")
elseif (CMAKE_Fortran_COMPILER_ID STREQUAL "Intel")
set(CMAKE_Fortran_FLAGS "${CMAKE_Fortran_FLAGS} -libs:static -free -static -fpp -real-size:64 -double-size:64")
#     set(CMAKE_Fortran_FLAGS "${CMAKE_Fortran_FLAGS} /ffree-line-length-0 /static-libgcc /static-libgfortran /static /fdefault-real-8 /fdefault-double-8 /cpp")
endif()
endif()

set(SOURCES
    src/Constants.f90
    src/ControllerBlocks.f90
    src/Controllers.f90
    src/DISCON.F90
    src/ROSCO_Types.f90
    src/Filters.f90
    src/Functions.f90
    src/ReadSetParameters.f90
    src/ROSCO_Helpers.f90
    src/ROSCO_IO.f90
    src/ZeroMQInterface.f90
    src/ExtControl.f90
    src/saturate.cpp
    src/wrap_180.cpp
    src/wrap_360.cpp
    src/ratelimit.cpp
    src/colemantransform.cpp
    src/colemantransforminverse.cpp
    src/identity.cpp
    src/sigma.cpp
    src/interp1d.cpp
    src/lpfilter.cpp
    src/hpfilter.cpp
    src/seclpfilter.cpp
    src/seclpfilter_vel.cpp
    src/notchfilter.cpp
    src/notchfilterslopes.cpp
    src/picontroller.cpp
    src/piicontroller.cpp
    src/rescontroller.cpp
    src/vit_controlparameters_view.f90
    src/foreaftdamping.cpp
    src/interp2d.cpp
    src/vit_performancedata_view.f90
    src/aerodyntorque.cpp
    src/floatingfeedback.cpp
    src/structuralcontrol.cpp
    src/cablecontrol.cpp
    src/flapcontrol.cpp
    src/yawratecontrol.cpp
    src/variablespeedcontrol.cpp
    src/ipc.cpp
    src/activewakecontrol.cpp
    src/pitchsaturation.cpp
    src/statemachine.cpp
    src/setpointsmoother.cpp
    src/powercontrolsetpoints.cpp
    src/refspeedexclusion.cpp
    src/computevariablessetpoints.cpp
    src/shutdown.cpp
    src/startup.cpp
    src/windspeedestimator.cpp
    src/pitchcontrol.cpp
    src/readavrswap.cpp
    src/pidcontroller.cpp
    src/checkinputs.cpp
)

if (${CMAKE_Fortran_COMPILER_ID} STREQUAL "GNU")
  if (WIN32)
    set(NWTC_SYS_FILE src/SysFiles/SysGnuWin.f90)
  elseif (APPLE OR UNIX OR CYGWIN)
    set(NWTC_SYS_FILE src/SysFiles/SysGnuLinux.f90)
  endif ()
elseif (${CMAKE_Fortran_COMPILER_ID} MATCHES "Flang")
  if (APPLE OR UNIX)
    set(NWTC_SYS_FILE src/SysFiles/SysFlangLinux.f90)
  endif (APPLE OR UNIX)
elseif (${CMAKE_Fortran_COMPILER_ID} MATCHES "^Intel")
  if (APPLE OR UNIX)
    set(NWTC_SYS_FILE src/SysFiles/SysIFL.f90)
  elseif (WIN32)
    set(NWTC_SYS_FILE src/SysFiles/SysIVF.f90)
  endif (APPLE OR UNIX)
endif ()

if (NWTC_SYS_FILE)
  message("-- Setting system file as: ${NWTC_SYS_FILE}")
  list(APPEND SOURCES ${NWTC_SYS_FILE})
else (NWTC_SYS_FILE)
  message(FATAL_ERROR "Cannot determine system file used with NWTC_Library")
endif (NWTC_SYS_FILE)

# ZMQ Library
find_package(PkgConfig)
pkg_check_modules(PC_ZeroMQ libzmq)
if(PC_ZeroMQ_FOUND)
	# Find ZeroMQ installation
	find_path(ZeroMQ_INCLUDE_DIR
			NAMES zmq.h
			PATHS ${PC_ZeroMQ_INCLUDE_DIRS}
			)
	find_library(ZeroMQ_LIBRARY
			NAMES zmq
			PATHS ${PC_ZeroMQ_LIBRARY_DIRS}
			)
	include_directories(${ZeroMQ_INCLUDE_DIR})

	set(CMAKE_Fortran_FLAGS "${CMAKE_Fortran_FLAGS} -L${PC_ZeroMQ_LIBRARY_DIRS} -lzmq")

	# Compile C-based ZeroMQ client as object library
	add_compile_options(-I${ZeroMQ_INCLUDE_DIR} -l${ZeroMQ_LIBRARY} -fPIC)
	add_library(zmq_client OBJECT src/zmq_client.c)

	# Add definition
	add_definitions(-DZMQ_CLIENT="TRUE")
    	set(SOURCES ${SOURCES}
	$<TARGET_OBJECTS:zmq_client>)
endif()

add_library(discon SHARED ${SOURCES})
target_include_directories(discon PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../translations)
target_link_libraries(discon PRIVATE stdc++)

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
  message("TRUE")
  set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/../")
endif()
