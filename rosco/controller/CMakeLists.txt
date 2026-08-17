cmake_minimum_required(VERSION 3.6)
project(ROSCO VERSION 2.9.0 LANGUAGES Fortran C)
enable_language(CXX)

set(CMAKE_Fortran_MODULE_DIRECTORY "${CMAKE_BINARY_DIR}/ftnmods")

if (NOT CMAKE_BUILD_TYPE)
  # Sets the optimization level to -O2 and includes -g for debugging
  set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Choose the build type: Debug RelWithDebInfo Release" FORCE)
endif()

message(STATUS "CMAKE_Fortran_COMPILER_ID = ${CMAKE_Fortran_COMPILER_ID}")

# E1.2 -- translation-phase flag, on EVERY compiler. Not a preference.
#
# Measured on this tree 2026-08-10, not assumed: at Release (-O3) gfortran
# contracts ColemanTransform's `cos(a)*r1 + cos(b)*r2 + cos(c)*r3` into 4
# `vfmadd` instructions, confirmed with objdump on the shipped
# __functions_MOD_colemantransform. A C++ translation of the same expression
# does not, so the two stop being bit-comparable: 105 of 200 random inputs
# differed by ~1 ULP. Adding this flag took that to 0 of 200. The difference
# was never in the translation.
#
# The compiler-specific spellings are Intel's `-fp-model precise` (its
# `-ffp-contract` spelling differs) and Flang/GNU's `-ffp-contract=off`. Only
# the GNU/Linux branch has been EXERCISED here; the others are written to the
# same requirement but are unverified on this tree.
set(FP_CONTRACT_OFF_GNU "-ffp-contract=off")
set(FP_CONTRACT_OFF_INTEL "-fp-model precise")

# The C++ side, which only exists once a translation is integrated. `vit
# integrate` adds `enable_language(CXX)` and a .cpp to SOURCES; it does not set
# CXX flags, so the translation would compile at Release's bare -O3 -DNDEBUG
# and could contract exactly the multiply-add the Fortran no longer does --
# reintroducing the difference from the other side, after the Fortran fix had
# been verified. "Every compiler" is the criterion, not "the one that showed
# the problem".
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${FP_CONTRACT_OFF_GNU}")

if(APPLE OR UNIX)
# Enable .dll export
if (CMAKE_Fortran_COMPILER_ID MATCHES "^Intel")
set(CMAKE_Fortran_FLAGS "${CMAKE_Fortran_FLAGS} -DIMPLICIT_DLLEXPORT -r8 -double-size 64 -cpp -no-wrap-margin ${FP_CONTRACT_OFF_INTEL}")
elseif (CMAKE_Fortran_COMPILER_ID STREQUAL "Flang")
add_definitions(-DFLANG_COMPILER)
set(CMAKE_Fortran_FLAGS "${CMAKE_Fortran_FLAGS} -DIMPLICIT_DLLEXPORT -fno-backslash -cpp -fdefault-real-8 ${FP_CONTRACT_OFF_GNU}")  
else()
set(CMAKE_Fortran_FLAGS "${CMAKE_Fortran_FLAGS} -DIMPLICIT_DLLEXPORT -ffree-line-length-0 -fdefault-real-8 -fdefault-double-8 -cpp ${FP_CONTRACT_OFF_GNU}")  
endif()
elseif (WIN32)
if (CMAKE_Fortran_COMPILER_ID STREQUAL "GNU")
# Ensure static linking to avoid requiring Fortran runtime dependencies
set(CMAKE_Fortran_FLAGS "${CMAKE_Fortran_FLAGS} -ffree-line-length-0 -static-libgcc -static-libgfortran -static -fdefault-real-8 -fdefault-double-8 -cpp ${FP_CONTRACT_OFF_GNU}")
elseif (CMAKE_Fortran_COMPILER_ID STREQUAL "Intel")
set(CMAKE_Fortran_FLAGS "${CMAKE_Fortran_FLAGS} -libs:static -free -static -fpp -real-size:64 -double-size:64 ${FP_CONTRACT_OFF_INTEL}")
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
    # Production bridge for ExtController's SAVE TYPE(ExtDLL_Type) and its
    # LoadDynamicLib call (plan.json declares that bridge). It is a NEW file,
    # not an integration wrapper, so it belongs in the build whether or not
    # ExtController is integrated -- the differential harness links it too.
    src/vit_extcontroller_dll.f90
    # Production bridge for Debug's CurDate/CurTime calls (plan.json declares
    # both PERMANENT BRIDGES for that unit). Same rule as the line above: a NEW
    # file rather than an integration wrapper, so it is built whether or not
    # Debug is integrated.
    src/vit_debug_bridges.f90
    src/colemantransform.cpp
    src/addtolist.cpp
    src/colemantransforminverse.cpp
    src/conv2uc.cpp
    src/vit_errorvariables_view.f90
    src/vit_extcontroltype_view.f90
    src/vit_localvariables_view.f90
    src/vit_controlparameters_view.f90
    src/extcontroller.cpp
    src/getpath.cpp
    src/getroot.cpp
    src/getwords.cpp
    src/hpfilter.cpp
    src/int2lstr.cpp
    src/lpfilter.cpp
    src/nondecreasing.cpp
    src/notchfilter.cpp
    src/notchfilterslopes.cpp
    src/pathisrelative.cpp
    src/readavrswap.cpp
    src/read_ol_input.cpp
    src/seclpfilter.cpp
    src/seclpfilter_vel.cpp
    src/statemachine.cpp
    src/updatezeromq.cpp
    src/identity.cpp
    src/interp1d.cpp
    src/saturate.cpp
    src/sigma.cpp
    src/unwrap.cpp
    src/wrap_180.cpp
    src/wrap_360.cpp
    src/checkinputs.cpp
    src/chkparsedata.cpp
    src/debug.cpp
    src/findline.cpp
    src/picontroller.cpp
    src/pidcontroller.cpp
    src/piicontroller.cpp
    src/pitchsaturation.cpp
    src/powercontrolsetpoints.cpp
    src/prefiltermeasuredsignals.cpp
    src/rescontroller.cpp
    src/setpointsmoother.cpp
    src/shutdown.cpp
    src/startup.cpp
    src/structuralcontrol.cpp
    src/yawratecontrol.cpp
    src/interp2d.cpp
    src/ratelimit.cpp
    src/activewakecontrol.cpp
    src/vit_performancedata_view.f90
    src/aerodyntorque.cpp
    src/cablecontrol.cpp
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

if(PC_ZeroMQ_FOUND)
	target_include_directories(discon PUBLIC ${ZeroMQ_INCLUDE_DIR})
	target_link_libraries(discon PUBLIC ${ZeroMQ_LIBRARY} stdc++)
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
