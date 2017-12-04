cmake_minimum_required (VERSION 2.8)
project (hatschet)

FIND_LIBRARY(SCALP_LIB
        NAMES libScaLP libScaLP.so libScaLP.dylib
	HINTS "${SCALP_PREFIX_PATH}/lib"
	DOC "Directory of the ScaLP library")

FIND_PATH(SCALP_H
          ScaLP/Solver.h
	  HINTS "${SCALP_PREFIX_PATH}/include"
	  DOC "Path to main ScaLP header file Solver.h"
)

IF (SCALP_H AND SCALP_LIB)
    MESSAGE(STATUS "Found ScaLP: ${SCALP_H},${SCALP_LIB}")
ELSE (SCALP_H AND SCALP_LIB)
    MESSAGE(FATAL_ERROR "Could not find ScaLP (${SCALP_H},${SCALP_LIB}). Please specify path by setting SCALP_PREFIX_PATH (-DSCALP_PREFIX_PATH=<path to scalp install dir>)")
ENDIF (SCALP_H AND SCALP_LIB)

add_library(hatschet 

src/HatScheT/Graph.cpp 
src/HatScheT/Exception.cpp
src/HatScheT/ASAPScheduler.cpp 
src/HatScheT/SchedulerBase.cpp 
src/HatScheT/ModuloSchedulerBase.cpp
)


add_executable(hatschet-bin src/hatschet.cpp)

set_target_properties(hatschet-bin PROPERTIES OUTPUT_NAME hatschet)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(${SCALP_H} src/)

target_link_libraries(hatschet-bin hatschet)
target_link_libraries(hatschet-bin ${SCALP_LIB})

