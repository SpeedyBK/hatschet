/*
    This file is part of the HatScheT project, developed at University of Kassel and TU Darmstadt, Germany
    Author: Patrick Sittel (sittel@uni-kassel.de)
    Author: Julian Oppermann (oppermann@esa.tu-darmstadt.de)

    Copyright (C) 2018

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#pragma once

#include <string>

namespace HatScheT {
	class Tests {
	public:
		/*!
		 * \brief readTest test whether a reference graph an resource model are read correctly
		 * \return
		 */
		static bool readTest();

		/*!
		 * test for the calculation of the critical path of a graph
		 * @return
		 */
		static bool cpTest();

		/*!
		 * read a graph, write it, read it again, then schedule both and compare
		 * @return
		 */
		static bool readWriteReadScheduleTest();

		/*!
		 * \brief moovacTest test whether moovac scheduling is running as expected
		 * \return
		 */
		static bool moovacTest();

		/*!
		 * \brief moduloSDCTest test whether modulo SDC scheduling is running as expected
		 * \return
		 */
		static bool moduloSDCTest();

		/*!
		 * \brief moduloSDCTestFiege test whether modulo SDC scheduling by nfiege is running as expected
		 * \return
		 */
		static bool moduloSDCTestFiege();

		/*!
		 * \brief test whether integer II non rect scheduling is running as expected
		 * @return
		 */
		static bool integerIINonRectTest();

		/*!
		 * \brief test whether integer II non rect scheduling is running as expected
		 * @return
		 */
		static bool integerIIPBTest();

		/*!
		 * \brief apiTest tests API functionality
		 * \return
		 */
		static bool apiTest();

		/*!
		 * \brief asapHCTest  test asap hc scheduler
		 * \return
		 */
		static bool asapHCTest();

		/*!
		 * \brief alapHCTest test alap hc scheduler
		 * \return
		 */
		static bool alapHCTest();

		/*!
		 * \brief ulSchedulerTest
		 * \return
		 */
		static bool ulSchedulerTest();

		/*!
		 * the rational min II test
		 * @return
		 */
		static bool rationalMinIITest();

		/*!
		* \brief Test if the Kosaraju works properly.
		* \return
		*/
		static bool KosarajuTest();

		/*!
		* \brief To test the DaiZhang scheduler
		* \return
		*/
		static bool DaiZhangTest();

		/*!
		 * @brief a test for functionallity of modulo schedulers
		 * this test also compares results achieved by different modulo scheduling algorithms
		 * @return
		 */
		static bool compareModuloSchedulerTest();

		/*!
		 * @brief a test for functionallity of the rational II scheduler
		 * by Sittel et. al.
		 * this test tries to solve the example problem from their paper
		 * @return
		 */
		static bool rationalIISchedulerTest();

		/*!
		 * @brief a test for functionallity of the uniform rational II scheduler
		 * by Sittel et. al.
		 * this test tries to solve the example problem from their paper
		 * @return
		 */
		static bool uniformRationalIISchedulerTest();

		/*!
		 * @brief a test for functionallity of the improved uniform rational II scheduler
		 * by Fiege
		 * this test tries to solve the example problem from their paper
		 * @return
		 */
		static bool uniformRationalIISchedulerNewTest();

		/*!
		 * @brief a test for functionallity of the non uniform rational II scheduler
		 * by Sittel et. al.
		 * this test tries to solve the example problem from their paper
		 * @return
		 */
		static bool nonUniformRationalIISchedulerTest();

		/*!
		 * @brief a test for functionallity of the rational II scheduler
		 * by Fimmel & Müller
		 * this test tries to solve the example problem from their paper (vanDongen)
		 * @return
		 */
		static bool rationalIISchedulerFimmelTest();

		/*!
		 * @brief test functionality of (rational II) ModuloQScheduler
		 */
		static bool rationalIIModuloQTest();

		/*!
		 * @brief test functionality of (rational II) SCCQScheduler
		 */
		static bool rationalIISCCQTest();

		/*!
		 * @brief test functionality of (rational II) ModuloSDC scheduler
		 */
		static bool rationalIIModuloSDCTest();

		/*!
		 * @brief test functionality of the combined rational II scheduler
		 */
		static bool rationalIICombinedSchedulerTest();

		/*!
		 * @brief test functionality of (rational II) SCCQScheduler for the TCAD example
		 */
		static bool tcadExampleTest();

		/*!
		 * @return test functionality to generate various ratII implementations for fir_SAM filter from Origami benchmark
		 */
		static bool firSAMRatIIImplementationsTest();

		/*!
		 * @return test functionality to generate various ratII implementations for fir_SHI filter from Origami benchmark
		 */
		static bool firSHIRatIIImplementationsTest();

		/*!
		 * @brief schedule a graph with asap, ed97 and rational II scheduler for direct comparison
		 */
		static bool maFiegeTest();

		/*!
		 * @brief schedule a graph with an II between 0 and 1
		 */
		static bool iiSmallerOneTest();

		/*!
		 * @brief try scheduling a graph thats impossible to schedule with current SCCQ scheduler implementation
		 */
		static bool sccqFailTest();

		/*!
		 * @brief try scheduling a graph thats impossible to schedule for the minimum II with any scheduler
		 */
		static bool minIntIIFailTest();

		/*!
		 * \brief A simpler test for the DaiZhang19Scheduler, with examples from the Paper "Improving Scalability of Exact
		 * Modulo Scheduling with Specialized Conflict-Driven Learning"
		 */
		static bool DaiZhangTestTwo();

		/*!
		 * \brief Test for the CaDiCaL-(SAT)-Solver. Reads a simple SAT-Problem from a file, solves it and compares the results
		 * with the expected results.
		 * @return
		 */
		static bool cadicalTest();

		/*!
		 * @brief test functionality of the SDSScheduler (joined SDC and SAT)
		 */
		static bool sdsSchedulerTest();

		/*!
		 * @brief this tests checks whether the ratII verifier is able to detect a MRT that
		 * exceeds the resource limits
		 * @return
		 */
		static bool ratIIVerifierWrongMRTDetected();

		/*!
		 * @brief his tests checks whether the ratII verifier is able to detect wrong causality
		 * @return
		 */
		static bool ratIIVerifierWrongCausalityDetected();

		/*!
		 * @brief tests iteration function in Utility
		 * @return
		 */
		static bool ratIIOptimalIterationTest();

		/*!
		 * @brief tests rational II modulo scheduler based on unrolling and integer II modulo scheduling
		 * @return
		 */
		static bool ratIIUnrollSchedulerTest();

		/*!
		 * @brief tests the basic functions of the fibonacci heap. In this test the heap is used to sort an array of numbers.
		 * An the result is compared to a simple bubble sort algorithm used on the same array of numbers.
		 * @return
		 */
		static bool fibonacciTest();

		static bool sdcSolverTest();

		/*!
		 * test getILPBasedIntIIBindingCong function
		 * @return
		 */
		static bool ilpBasedIntIIBindingTestCong();

		/*!
		 * test getILPBasedIntIIBinding function
		 * @return
		 */
		static bool ilpBasedIntIIBindingTest();

		/*!
		 * test OptimalIntegerIIGeneralizedBinding class
		 * @return
		 */
		static bool optimalIntegerIIGeneralizedBindingTest();

		/*!
		 * test getILPMinMuxBinding function
		 * @return
		 */
		static bool ilpBasedIntIIMinMuxBindingTest();

		/*!
		 * test treeBindTest function
		 * @return if test was passed
		 */
		static bool treeBindTest();

		/*!
		 * test treeBindTest function with commutative resource types
		 * @return if test was passed
		 */
		static bool treeBindCommutativeTest();

		/*!
		 * use tree bind algorithm for motivating example for FCCM paper about optimal binding
		 * @return if test was passed
		 */
		static bool fccmPaperTest();

		/*!
		 * test functionality of multi min reg scheduler
		 * @return if test was passed
		 */
		static bool multiMinRegSchedulerTest();

		/*!
		 * test functionality of multi min reg scheduler
		 * @return if test was passed
		 */
		static bool satSchedulerTest();

		/*!
		 * test functionality of Z3 Theorem Prover
		 * @return if test was passed
		 */
		static bool z3Test();
		/*!
		 * test functionality of Z3 based scheduler
		 * @return if test was passed
		 */
		static bool smtBinary();
        /*!
         * test functionality of utility latency estimation
         * @return if test was passed
         */
		static bool utilityLatencyEstimation();

        /*!
         * test functionality of SAT based binding algorithm
         * @return if test was passed
         */
		static bool satBinding();

        /*!
         * test functionality of Z3 based heuristic scheduler with scc-based preprocessing
         * @return if test was passed
         */
		static bool smtScc();

		static bool smtOptimizer();

	};
}
