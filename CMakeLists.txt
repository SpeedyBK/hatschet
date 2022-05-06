# This file is part of the HatScheT project, developed at University of Kassel and TU Darmstadt, Germany
# Author: Martin Kumm, Patrick Sittel, Julian Oppermann ({kumm, sittel}@uni-kassel.de, oppermann@esa.tu-darmstadt.de)
#
# Copyright (C) 2018
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


cmake_minimum_required (VERSION 2.8)
project (hatschet)

####################
#Set options for used libs
####################
option(USE_XERCESC "Use XercesC for parsing" ON)
#option(USE_CADICAL "Use CaDiCaL SAT solving" ON)
option(OLD_CXX_ABI "Build with an old C++-abi (activate if you get undefined references with '__cxx11')" OFF)


# Deactivate C++11-abi?
if(OLD_CXX_ABI) 
  add_definitions(-D_GLIBCXX_USE_CXX11_ABI=0) 
endif()

####################
#Find ScaLP
####################
FIND_LIBRARY(SCALP_LIB
    NAMES libScaLP libScaLP.so libScaLP.dylib
    HINTS "${SCALP_PREFIX_PATH}/lib"
    DOC "Directory of the ScaLP library"
)

FIND_PATH(SCALP_H
    ScaLP/Solver.h
    HINTS "${SCALP_PREFIX_PATH}/include"
    DOC "Path to main ScaLP header file Solver.h"
)

IF (SCALP_H AND SCALP_LIB)
    add_definitions(-DUSE_SCALP)
    MESSAGE(STATUS "Found ScaLP: ${SCALP_H},${SCALP_LIB}")
ELSE (SCALP_H AND SCALP_LIB)
    MESSAGE(STATUS "Could not find ScaLP (${SCALP_H},${SCALP_LIB}). Please specify path by setting SCALP_PREFIX_PATH (-DSCALP_PREFIX_PATH=<path to scalp install dir>)")
ENDIF (SCALP_H AND SCALP_LIB)

####################
#Find XerxesC lib
####################
FIND_LIBRARY(XERCESC_LIB
        NAMES xerces-c
        HINTS "${XERCESC_LIBRARY_PATH}"
        DOC "Directory of the XercesC library")

IF (XERCESC_LIB)
    MESSAGE(STATUS "Found XercesC Lib: ${XERCESC_LIB}")
ELSE (XERCESC_LIB)
    MESSAGE(STATUS "Could not find XercesC Lib (${XERCESC_LIB}). Please specify path by setting XERCESC_LIBRARY_PATH (-DXERCESC_LIBRARY_PATH=<path to xerces lib dir>)")
ENDIF (XERCESC_LIB)

####################
#Find XerxesC headers
####################
FIND_PATH(XERCESC_H
          xercesc/parsers/SAXParser.hpp
          HINTS "${XERCESC_INCLUDE_PATH}"
          DOC "Path to main XercesC header files"
)

IF (XERCESC_H)
    MESSAGE(STATUS "Found XercesC Headers: ${XERCESC_H}")
ELSE (XERCESC_H)
    MESSAGE(STATUS "Could not find XercesC Headers (${XERCESC_H}). Please specify path by setting XERCESC_INCLUDE_PATH (-DXERCESC_INCLUDE_PATH=<path to xerces include dir>)")
ENDIF (XERCESC_H)

####################
#set define for XercesC
####################
IF (USE_XERCESC)
    IF (XERCESC_H AND XERCESC_LIB)
        add_definitions(-DUSE_XERCESC)
        MESSAGE(STATUS "Found XercesC: GraphML parser will be available")
    ELSE (XERCESC_H AND XERCESC_LIB)
        MESSAGE(STATUS "Did not found XercesC: GraphML parser will not be available")
    ENDIF (XERCESC_H AND XERCESC_LIB)
ELSE(USE_XERCESC)
    MESSAGE(STATUS "XercesC was turned off manually (-DUSE_XERCESC=OFF): GraphML parser will not be available")
ENDIF (USE_XERCESC)

####################
#Find CaDiCaL
####################
FIND_LIBRARY(CADICAL_LIB
             NAMES libcadical.a
             HINTS "${CADICAL_LIBRARY_PATH}"
             DOC "Directory of the CaDiCaL library")

FIND_PATH(CADICAL_H
        cadical.hpp
        HINTS "${CADICAL_INCLUDE_PATH}"
        DOC "Path to main CaDiCal header file"
        )

IF(CADICAL_H AND CADICAL_LIB)
    add_definitions(-DUSE_CADICAL)
    MESSAGE(STATUS "Found CaDiCaL Lib: ${CADICAL_LIB} and Headers: ${CADICAL_H}")
ELSE(CADICAL_H AND CADICAL_LIB)
    MESSAGE(STATUS "Could not find CaDiCaL Lib (${CADICAL_LIB}) or CaDiCaL Header (${CADICAL_H}). Please specify paths by setting CADICAL_LIBRARY_PATH (-DCADICAL_LIBRARY_PATH=<path to CaDiCaL lib dir>) and CADICAL_INCLUDE_PATH (-DCADICAL_INCLUDE_PATH=<path to CaDiCaL include dir>)")
ENDIF(CADICAL_H AND CADICAL_LIB)


####################
#Find Z3-SMT
####################
#Has to be changed, when Z3 is installed completely
set(Z3_DIR "/home/bkessler/Repositories/z3_install/usr/local/lib64/cmake/z3/")

find_package(Z3
        CONFIG
        # `NO_DEFAULT_PATH` is set so that -DZ3_DIR has to be passed to find Z3.
        # This should prevent us from accidentally picking up an installed
        # copy of Z3. This is here to benefit Z3's build system when building
        # this project. When making your own project you probably shouldn't
        # use this option.
        NO_DEFAULT_PATH
        )

IF(${Z3_FOUND})
    #message(STATUS "Z3_FOUND: ${Z3_FOUND}")
    message(STATUS "Z3_FOUND!")
    message(STATUS "Found Z3 ${Z3_VERSION_STRING}")
    message(STATUS "Z3_DIR: ${Z3_DIR}")
    add_definitions(-DUSE_Z3)
ELSE(${Z3_FOUND})
    message(STATUS "Could not find Z3 (...). Please specify path by setting Z3_PREFIX_PATH (-DZ3_PREFIX_PATH=<path to Z3 install dir>)")
ENDIF(${Z3_FOUND})

####################
#SRC and Headerfiles
####################
set(HatScheT_SOURCES
src/HatScheT/Vertex.cpp
src/HatScheT/Edge.cpp
src/HatScheT/Graph.cpp
src/HatScheT/TargetModel.cpp
src/HatScheT/utility/Exception.cpp
src/HatScheT/base/SchedulerBase.cpp
src/HatScheT/layers/RationalIISchedulerLayer.cpp
src/HatScheT/ResourceModel.cpp
src/HatScheT/utility/reader/Reader.cpp
src/HatScheT/utility/reader/GraphMLGraphReader.cpp
src/HatScheT/utility/reader/XMLResourceReader.cpp
src/HatScheT/utility/reader/ScheduleAndBindingReader.cpp
src/HatScheT/utility/subgraphs/KosarajuSCC.cpp
src/HatScheT/utility/subgraphs/Occurrence.cpp
src/HatScheT/utility/subgraphs/OccurrenceSet.cpp
src/HatScheT/utility/subgraphs/OccurrenceSetCombination.cpp
src/HatScheT/utility/writer/Writer.cpp
src/HatScheT/utility/writer/GraphMLGraphWriter.cpp
src/HatScheT/utility/writer/DotWriter.cpp
src/HatScheT/utility/writer/ScheduleAndBindingWriter.cpp
src/HatScheT/utility/writer/DGLWriter.cpp
src/HatScheT/utility/Utility.cpp
src/HatScheT/utility/Binding.cpp
src/HatScheT/utility/BindingBase.cpp
src/HatScheT/utility/TreeBind.cpp
src/HatScheT/scheduler/ASAPScheduler.cpp
src/HatScheT/scheduler/ALAPScheduler.cpp
src/HatScheT/scheduler/ULScheduler.cpp
src/HatScheT/utility/Verifier.cpp
src/HatScheT/utility/reader/XMLTargetReader.cpp
src/HatScheT/utility/writer/XMLResourceWriter.cpp
src/HatScheT/base/IterativeSchedulerBase.cpp
src/HatScheT/utility/subgraphs/SCC.cpp
src/HatScheT/utility/FibonacciHeap.h
src/HatScheT/utility/SDCSolver.cpp 
src/HatScheT/utility/SDCSolver.h
)

set(HatScheT_HEADERS
src/HatScheT/base/ILPSchedulerBase.h
src/HatScheT/base/ModuloSchedulerBase.h
src/HatScheT/base/SchedulerBase.h
src/HatScheT/base/IterativeSchedulerBase.h
src/HatScheT/layers/RationalIISchedulerLayer.h
src/HatScheT/TargetModel.h
src/HatScheT/Edge.h
src/HatScheT/Graph.h
src/HatScheT/ResourceModel.h
src/HatScheT/scheduler/ALAPScheduler.h
src/HatScheT/scheduler/ASAPScheduler.h
src/HatScheT/scheduler/graphBased/SGMScheduler.h
src/HatScheT/scheduler/ilpbased/ASAPILPScheduler.h
src/HatScheT/scheduler/ilpbased/ModuloSDCScheduler.h
src/HatScheT/scheduler/ilpbased/MoovacMinRegScheduler.h
src/HatScheT/scheduler/ilpbased/MoovacResAwScheduler.h
src/HatScheT/scheduler/ilpbased/MoovacScheduler.h
src/HatScheT/scheduler/ilpbased/EichenbergerDavidson97Scheduler.h
src/HatScheT/scheduler/ilpbased/SuchaHanzalek11Scheduler.h
src/HatScheT/scheduler/ilpbased/SuchaHanzalek11ResAwScheduler.h
src/HatScheT/scheduler/ilpbased/RationalIIScheduler.h
src/HatScheT/scheduler/ilpbased/RationalIISchedulerFimmel.h
src/HatScheT/scheduler/dev/IntegerIINonRectScheduler.h
src/HatScheT/scheduler/dev/MinRegMultiScheduler.h
src/HatScheT/scheduler/graphBased/PBScheduler.h
src/HatScheT/scheduler/dev/ModuloQScheduler.h
src/HatScheT/scheduler/dev/UniformRationalIIScheduler.h
src/HatScheT/scheduler/dev/UniformRationalIISchedulerNew.h
src/HatScheT/scheduler/dev/RationalIIModuloSDCScheduler.h
src/HatScheT/scheduler/dev/NonUniformRationalIIScheduler.h
src/HatScheT/scheduler/dev/UnrollRationalIIScheduler.h
src/HatScheT/scheduler/dev/SCCQScheduler.h
src/HatScheT/scheduler/dev/CombinedRationalIIScheduler.h
src/HatScheT/scheduler/ULScheduler.h
src/HatScheT/scheduler/dev/ModSDC.h
src/HatScheT/utility/Exception.h
src/HatScheT/utility/reader/GraphMLGraphReader.h
src/HatScheT/utility/reader/XMLResourceReader.h
src/HatScheT/utility/reader/XMLTargetReader.h
src/HatScheT/utility/reader/ScheduleAndBindingReader.h
src/HatScheT/utility/reader/Reader.h
src/HatScheT/utility/subgraphs/KosarajuSCC.h
src/HatScheT/utility/subgraphs/Occurrence.h
src/HatScheT/utility/subgraphs/OccurrenceSet.h
src/HatScheT/utility/subgraphs/OccurrenceSetCombination.h
src/HatScheT/utility/Tests.h
src/HatScheT/utility/Utility.h
src/HatScheT/utility/Binding.h
src/HatScheT/utility/BindingBase.h
src/HatScheT/utility/TreeBind.h
src/HatScheT/utility/OptimalIntegerIIBinding.h
src/HatScheT/utility/OptimalIntegerIIBindingCong.h
src/HatScheT/utility/OptimalIntegerIIGeneralizedBinding.h
src/HatScheT/utility/Verifier.h
src/HatScheT/utility/writer/DotWriter.h
src/HatScheT/utility/writer/ScheduleAndBindingWriter.h
src/HatScheT/utility/writer/DGLWriter.h
src/HatScheT/utility/writer/Writer.h
src/HatScheT/utility/writer/GraphMLGraphWriter.h
src/HatScheT/utility/writer/XMLResourceWriter.h
src/HatScheT/utility/subgraphs/SCC.h
src/HatScheT/Vertex.h
src/HatScheT/scheduler/dev/DaiZhang19Scheduler.h
src/HatScheT/scheduler/dev/SDSScheduler.h
src/HatScheT/scheduler/dev/SATScheduler.h
src/HatScheT/scheduler/dev/SATMinRegScheduler.h
)

IF (SCALP_H AND SCALP_LIB)
list(APPEND HatScheT_SOURCES
src/HatScheT/base/ILPSchedulerBase.cpp
src/HatScheT/base/ModuloSchedulerBase.cpp
src/HatScheT/scheduler/ilpbased/ASAPILPScheduler.cpp
src/HatScheT/scheduler/ilpbased/MoovacScheduler.cpp
src/HatScheT/scheduler/ilpbased/MoovacMinRegScheduler.cpp
src/HatScheT/scheduler/ilpbased/EichenbergerDavidson97Scheduler.cpp
src/HatScheT/scheduler/ilpbased/SuchaHanzalek11Scheduler.cpp
src/HatScheT/scheduler/ilpbased/SuchaHanzalek11ResAwScheduler.cpp
src/HatScheT/scheduler/ilpbased/ModuloSDCScheduler.cpp
src/HatScheT/scheduler/ilpbased/RationalIIScheduler.cpp
src/HatScheT/scheduler/ilpbased/ModuloSDCScheduler.cpp
src/HatScheT/scheduler/ilpbased/RationalIIScheduler.cpp
src/HatScheT/scheduler/ilpbased/RationalIISchedulerFimmel.cpp
src/HatScheT/scheduler/ilpbased/MoovacResAwScheduler.cpp
src/HatScheT/scheduler/dev/IntegerIINonRectScheduler.cpp
src/HatScheT/scheduler/dev/MinRegMultiScheduler.cpp
src/HatScheT/scheduler/graphBased/PBScheduler.cpp
src/HatScheT/scheduler/dev/UnrollRationalIIScheduler.cpp
src/HatScheT/scheduler/dev/ModSDC.cpp
src/HatScheT/scheduler/dev/ModuloQScheduler.cpp
src/HatScheT/scheduler/dev/SCCQScheduler.cpp
src/HatScheT/scheduler/dev/UniformRationalIIScheduler.cpp
src/HatScheT/scheduler/dev/UniformRationalIISchedulerNew.cpp
src/HatScheT/scheduler/dev/RationalIIModuloSDCScheduler.cpp
src/HatScheT/scheduler/dev/CombinedRationalIIScheduler.cpp
src/HatScheT/scheduler/dev/NonUniformRationalIIScheduler.cpp
src/HatScheT/scheduler/dev/DaiZhang19Scheduler.cpp
src/HatScheT/scheduler/graphBased/SGMScheduler.cpp
src/HatScheT/utility/Tests.cpp
src/HatScheT/utility/OptimalIntegerIIBinding.cpp
src/HatScheT/utility/OptimalIntegerIIBindingCong.cpp
src/HatScheT/utility/OptimalIntegerIIGeneralizedBinding.cpp
)
ENDIF (SCALP_H AND SCALP_LIB)

IF (CADICAL_H AND CADICAL_LIB)
list(APPEND HatScheT_SOURCES
src/HatScheT/scheduler/dev/SDSScheduler.cpp
src/HatScheT/scheduler/dev/SATScheduler.cpp
src/HatScheT/scheduler/dev/SATMinRegScheduler.cpp
)
ENDIF (CADICAL_H AND CADICAL_LIB)

add_library(hatschet ${HatScheT_SOURCES})
add_executable(hatschet-bin src/hatschet.cpp)
set_target_properties(hatschet-bin PROPERTIES OUTPUT_NAME hatschet)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
target_link_libraries(hatschet-bin hatschet)

IF (SCALP_H AND SCALP_LIB)
    target_link_libraries(hatschet-bin ${SCALP_LIB})
ENDIF (SCALP_H AND SCALP_LIB)
IF (XERCESC_H AND XERCESC_LIB)
    target_link_libraries(hatschet-bin ${XERCESC_LIB})
ENDIF (XERCESC_H AND XERCESC_LIB)
IF (CADICAL_H AND CADICAL_LIB)
    target_link_libraries(hatschet-bin ${CADICAL_LIB})
ENDIF (CADICAL_H AND CADICAL_LIB)

include_directories(src/)

IF (SCALP_H AND SCALP_LIB)
include_directories(${SCALP_H})
ENDIF (SCALP_H AND SCALP_LIB)

IF (XERCESC_H AND XERCESC_LIB)
include_directories(${XERCESC_H})
ENDIF (XERCESC_H AND XERCESC_LIB)

IF (CADICAL_H AND CADICAL_LIB)
include_directories(${CADICAL_H})
ENDIF (CADICAL_H AND CADICAL_LIB)

####################
#Z3 related
####################
IF(${Z3_FOUND})
    target_link_libraries(hatschet-bin ${Z3_LIBRARIES})
ENDIF(${Z3_FOUND})

IF(${Z3_FOUND})
    include_directories(${Z3_CXX_INCLUDE_DIRS})
    target_compile_options(hatschet-bin PRIVATE ${Z3_COMPONENT_CXX_FLAGS})
ENDIF(${Z3_FOUND})

if (CMAKE_SYSTEM_NAME MATCHES "[Ww]indows")
    # On Windows we need to copy the Z3 libraries
    # into the same directory as the executable
    # so that they can be found.
    foreach (z3_lib ${Z3_LIBRARIES})
        message(STATUS "Adding copy rule for ${z3_lib}")
        add_custom_command(TARGET hatschet-bin
                POST_BUILD
                COMMAND
                ${CMAKE_COMMAND} -E copy_if_different
                $<TARGET_FILE:${z3_lib}>
                $<TARGET_FILE_DIR:cpp_example>
                )
    endforeach()
endif()

####################
# HaTScheTConfig.cmake
####################

configure_file("${PROJECT_SOURCE_DIR}/HaTScheTConfig.cmake.in" "${PROJECT_SOURCE_DIR}/HaTScheTConfig.cmake" @ONLY)

####################
# testing
####################

enable_testing()

IF (SCALP_H AND SCALP_LIB AND XERCESC_H AND XERCESC_LIB)
add_test("Read" "./hatschet" "--test=READ")
add_test("Moovac" "./hatschet" "--test=MOOVAC")
add_test("ReadWriteReadSchedule" "./hatschet" "--test=RWRS")
add_test("ModuloSDC" "./hatschet" "--test=MODULOSDC")
add_test("API" "./hatschet" "--test=API")
add_test("ASAPHC" "./hatschet" "--test=ASAPHC")
add_test("ALAPHC" "./hatschet" "--test=ALAPHC")
add_test("ULScheduler" "./hatschet" "--test=ULScheduler")
add_test("criticalpath" "./hatschet" "--test=CRITPATH")
add_test("KosarajuSCC" "./hatschet" "--test=KOSARAJU")
add_test("CompareMSAlgorithms" "./hatschet" "--test=COMPAREMSALGORITHMS")
add_test("rationalIISchedulerTest" "./hatschet" "--test=RATIONALIISCHEDULER")
add_test("rationalIISchedulerFimmelTest" "./hatschet" "--test=RATIONALIISCHEDULERFIMMEL")
add_test("ratIIVerifierWrongMRTDetected" "./hatschet" "--test=ratIIVerifierWrongMRTDetected")
add_test("ratIIVerifierWrongCausalityDetected" "./hatschet" "--test=ratIIVerifierWrongCausalityDetected")
add_test("ratIIVerifierWrongCausalityDetected" "./hatschet" "--test=ratIIVerifierWrongCausalityDetected")
add_test("uniformRatII" "./hatschet" "--test=UNIFORMRATIONALIISCHEDULER")
add_test("uniformRatIINew" "./hatschet" "--test=UNIFORMRATIONALIISCHEDULERNEW")
add_test("nonUniformRatII" "./hatschet" "--test=NONUNIFORMRATIONALIISCHEDULER")
add_test("ratIIModuloQ" "./hatschet" "--test=RATIONALIIMODULOQ")
add_test("ratIISCCQ" "./hatschet" "--test=RATIONALIISCCQ")
add_test("ratIIUnroll" "./hatschet" "--test=UNROLLSCHEDULER")
add_test("integerIINonRect" "./hatschet" "--test=INTEGERIINONRECT")
add_test("optimalBinding" "./hatschet" "--test=ILPBASEDINTIIBINDING")
add_test("treeBind" "./hatschet" "--test=TREEBIND")
add_test("treeBindCommutative" "./hatschet" "--test=TREEBINDCOMMUTATIVE")
add_test("congBind" "./hatschet" "--test=ILPBASEDINTIIBINDINGCONG")
ENDIF(SCALP_H AND SCALP_LIB AND XERCESC_H AND XERCESC_LIB)

IF(CADICAL_H AND CADICAL_LIB)
    add_test("CaDiCaL" "./hatschet" "--test=CADICAL")
ENDIF(CADICAL_H AND CADICAL_LIB)

IF(${Z3_FOUND})
    add_test("Z3" "./hatschet" "--test=Z3")
ENDIF(${Z3_FOUND})

####################
# example project
####################
IF (SCALP_H AND SCALP_LIB)
add_executable(example-bin example/example.cpp)
target_link_libraries(example-bin hatschet)
target_link_libraries(example-bin ${SCALP_LIB})
ENDIF (SCALP_H AND SCALP_LIB)

####################
# Installation
####################

install (TARGETS hatschet-bin DESTINATION bin)
install (TARGETS hatschet DESTINATION lib)
install (DIRECTORY src/HatScheT/ DESTINATION include/HatScheT FILES_MATCHING PATTERN "*.h")
