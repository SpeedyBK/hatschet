/*
    This file is part of the HatScheT project, developed at University of Kassel and TU Darmstadt, Germany
    Author: Martin Kumm, Patrick Sittel ({kumm, sittel}@uni-kassel.de)

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

#include <vector>
#include "HatScheT/base/SchedulerBase.h"
#include "HatScheT/base/ModuloSchedulerBase.h"
#include "HatScheT/base/IterativeSchedulerBase.h"
#include "HatScheT/utility/Binding.h"

namespace HatScheT {
/*!
 * \brief The RationalIISchedulerLayer class providing information that is needed implement rational II modulo schedules
 */
	class RationalIISchedulerLayer : public SchedulerBase, public ModuloSchedulerBase, public IterativeSchedulerBase {
	public:
		RationalIISchedulerLayer(Graph &g, ResourceModel &resourceModel);

		/*!
		 * schedule function - since the core of this function is the same for all rational II schedulers,
		 * this is only implemented once! All schedulers must only overload scheduleIteration,
		 * where the loop body is implemented
		 */
		void schedule() final;

		/*!
		 * @brief
		 * @return
		 */
		int getScheduleLength() final;

		/*!
		 * \brief getInitIntervalls specific timeslots for initiation of samples into the schedule
		 * \return
		 */
		std::vector<int> getLatencySequence() { return this->latencySequence; }

		/*!
		 * \brief getModulo the modulo number determines the ratio in which the initiation of samples is repeated
		 * \return
		 */
		int getModulo() const { return this->modulo; }

		/*!
		 * \brief getSamples get the number of samples that can inserted every 'this->modulo' clock cycles
		 * @return
		 */
		int getSamples() const { return this->samples; }

		/*!
		 * \brief setModulo manually set modulo
		 * \return
		 */
		void setModulo(int m) { this->modulo = m; }

		/*!
		 * \brief setSamples manually set samples
		 * @return
		 */
		void setSamples(int s) { this->samples = s; }

		/*!
		 * dont use this function for rational II modulo schedules
		 * this function will throw an exception
		 * use getRatIILifeTimes()
		 * @return
		 */
		std::map<Edge *, int> getLifeTimes() override {throw HatScheT::Exception("RationalIISchedulerLayer::getLifeTimes: Rational II Lifetimes are more complicated! Don't use this function! Use getRatIILifeTimes() instead!");}

		/*!
		 * dont use this function fo rational II modulo schedules
		 * @return
		 */
		std::map<const Vertex *, int> getBindings() override {throw Exception("RationalIISchedulerLayer::getBindings: Dont use this function for rational II schedules! Use getRationalIIBinding!");}

		/*!
		* \brief getLifeTimes using the determined rational II
		* lifetimes in rational II schedules are determined using the start times vector
		* this is crucial because samples are inserted and calculated "arbitrarily"
		* remark: overloaded function from the scheduler base class
		* \return
		*/
		virtual std::map<Edge *, vector<int> > getRatIILifeTimes();

		/*!
		 * base function for rational II schedule bindings
		 * @return
		 */
		virtual vector<std::map<const Vertex *, int> > &getRationalIIBindings();

		/*!
		 * print the uneven spaced initiation times of data samples
		 * those repeat every m cycles
		 * @return
		 */
		vector<std::map<Vertex *, int> > &getStartTimeVector() { return this->startTimesVector; }

		/*!
		 * @brief iteration start of s
		 * @return
		 */
		int getS_Start() { return this->s_start; }

		/*!
		 * @brief iteration start of m
		 * @return
		 */
		int getM_Start() { return this->m_start; }

		/*!
		 * @brief found value for s (-1 if no schedule was found)
		 * @return
		 */
		int getS_Found() { return this->s_found; }

		/*!
		 * @brief found value for m (-1 if no schedule was found)
		 * @return
		 */
		int getM_Found() { return this->m_found; }

		/*!
		 * this algorithm creates a sorted queue with M/S pairs in the interval [minII_Q, minII_N)
		 * list is sorted by the values of M/S
		 * first list element is always mMinII/sMinII which corresponds to II = minII_Q
		 * the list contains only non-reducable fractions! So if e.g. M/S = 3/2 is in the list, 6/4, 9/6, ... will NOT be!
		 * @param sMinII samples for minII_Q
		 * @param mMinII modulo for minII_Q
		 * @param integerII minimum integer II (minII_N)
		 * @param sMax maximum number of samples => all M/S pairs have S leq sMax -> -1: sMax = sMinII
		 * @param maxListSize only return the best maxListSize M/S pairs -> -1: return all found M/S pairs
		 * @return first pair element: M, second pair element: S
		 */
		static std::list<pair<int, int>>
		getRationalIIQueue(int sMinII, int mMinII, int integerII, int sMax, int maxListSize = -1);

		/*!
		 * calculates initiation interval sequence, which is as uniformly distributed as possible
		 * @param samples
		 * @param modulo
		 * @param quiet
		 * @return
		 */
		static std::vector<int> getOptimalInitiationIntervalSequence(int samples, int modulo, bool quiet);

		/*!
		 * compute intervals between samples for the initiation intervals sequence
		 * @param initIntervals
		 * @param M modulo
		 * @return
		 */
		static std::vector<int> getLatencySequenceFromInitiationIntervals(std::vector<int> &initIntervals, int M);

		/*!
		 *
		 * @return if the found schedule is valid
		 */
		bool getScheduleValid() const { return this->scheduleValid; }

		/*!
		 * @brief the s_max value is used to obtain smaller ILP problem formulations
		 * the smaller s_max, the easier the problems should be solveable
		 * default value -1 (unlimited)
		 * use this value only when you have knowledge regarding the rat II iteration process
		 * @param s
		 */
		void setSMax(int s) {
			this->s_max = s;
		}

		int getSMax() {
			return this->s_max;
		}

	protected:
		/*!
		 * each scheduler should overload this function for one schedule iteration
		 * M/S are already set automatically by this class
		 *
		 */
		virtual void scheduleIteration() {
			throw HatScheT::Exception("RationalIISchedulerLayer::scheduleIteration should never be called!");
		}

		/*!
		 * this function sets the s and m values in a way that not needed values are skipped
		 * and the rational II becomes as small as possible
		 */
		void autoSetMAndS();

		/*!
		 * the bindings of rational II schedule
		 */
		vector<std::map<const Vertex *, int> > ratIIbindings;
		/*!
		 * \brief initIntervals
		 */
		std::vector<int> latencySequence;
		/*!
		 * \brief modulo
		 */
		int modulo;
		/*!
		 * \brief samples
		 */
		int samples;
		/*!
		 * @brief the s value for the iteration start
		 */
		int s_start;
		/*!
		 * @brief the m value for the iteration start
		 */
		int m_start;
		/*!
		 * @brief the identified s value
		 */
		int s_found;
		/*!
		 * @brief the identified s value
		 */
		int m_found;
		/*!
		 * @brief used for iteration
		 */
		int s_max;
		/*!
		 * the minimum interger II that is possible
		 */
		int integerMinII;
		/*!
		 * the final rational II schedule
		 */
		vector<std::map<Vertex *, int> > startTimesVector;
		/*!
		 * flag if the minimum rational II was found
		 */
		bool minRatIIFound;

	private:
		/*!
		 * verify the found schedule (stored in startTimesVector)
		 * 	1) based on rational II verifier
		 * 	2) based on integer II verifier of unrolled graph
		 * throw error if the verifiers lead to different results!
		 * @return if the schedule is valid
		 */
		bool verifySchedule();

		/*!
		 * flag if the found schedule is valid
		 */
		bool scheduleValid;
	};

}
