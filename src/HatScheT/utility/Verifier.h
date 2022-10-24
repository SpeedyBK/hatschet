/*
    This file is part of the HatScheT project, developed at University of Kassel and TU Darmstadt, Germany
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

#include "HatScheT/utility/Binding.h"
#include "HatScheT/Graph.h"
#include "HatScheT/ResourceModel.h"
#include <vector>
#include <map>

/*!
 * \namespace HatScheT
 * \brief The main namespace for all HatScheT components
 */
namespace HatScheT {

/*!
 * \brief Verifies the given resource-constrained schedule
 *
 * Verifies whether all precedence (induced by the graph) and resource
 * constraints (induced by the resource model) are obeyed in the given
 * resource-constrained schedule.
 *
 * TODO: also check cycle-time constraints (-> chaining)
 *
 * @param g the problem instance's dependence graph
 * @param rm the problem instance's resource model
 * @param schedule the computed schedule
 * @param SL the determined schedule length
 * @return true iff the schedule is valid
 */
bool verifyResourceConstrainedSchedule(Graph &g, ResourceModel &rm, std::map<Vertex *, int> &schedule, int SL, bool quiet = false);

/*!
 * \brief Verifies the given modulo schedule
 *
 * Verifies whether all precedence (induced by the graph) and resource
 * constraints (induced by the resource model and the II) are obeyed
 * in the given modulo schedule.
 *
 * TODO: also check cycle-time constraints (-> chaining)
 *
 * @param g the problem instance's dependence graph
 * @param rm the problem instance's resource model
 * @param schedule schedule the computed schedule
 * @param II the determined initiation interval
 * @return true iff the schedule is valid
 */
bool verifyModuloSchedule(Graph &g, ResourceModel &rm, std::map<Vertex *, int> &schedule, int II, bool quiet = false);
/*!
 * Verifies the given rational II modolo schedule
 *
 * Verifies all precedence (induced by the graph) calculating the effective II of every sample and start time
 * The resource constraints of all samples (with the possibility of a non uniform schedule for each sample)
 * are verified.
 *
 * @param g the problem instance's dependence graph
 * @param rm the problem instance's resource model
 * @param schedule the determined schedule for each sample
 * @param latencySequence the determined initiation intervals as vector of differences of II
 * @param scheduleLength the max Latency of the determined rational II modulo schedule
 * @return true iff the schedule is valid
 */
bool verifyRationalIIModuloSchedule2(Graph &g, ResourceModel &rm, vector <std::map<Vertex *, int>> &schedule,
																		 vector<int> latencySequence, int scheduleLength,
																		 bool quiet = false);
bool verifyRationalIIModuloSchedule(Graph &g, ResourceModel &rm, vector <std::map<Vertex *, int>> &schedule,
																		int samples, int modulo, bool quiet = false);

/*!
 * \brief Verifies the given modulo schedule
 *
 * Verifies whether all precedence (induced by the graph) and resource
 * constraints (induced by the resource model and the II) are obeyed
 * in the given modulo schedule. This function accepts a Rational II and timings.
 *
 * TODO: also check cycle-time constraints (-> chaining)
 *
 * @param g the problem instance's dependence graph
 * @param rm the problem instance's resource model
 * @param schedule schedule the computed schedule
 * @param II the determined initiation interval
 * @return true iff the schedule is valid
 */
bool verifyModuleScheduleRational(Graph &g, ResourceModel &rm, std::map<Vertex *, double> &schedule, double II, bool quiet = false);

/*!
 * verify a binding
 * @param g graph
 * @param rm resource model
 * @param sched schedule time container
 * @param II initiation interval
 * @param commutativeOps a set with commutative operations that do not care about their port assignments
 * @return if the binding is ok
 */
	bool verifyIntIIBinding(Graph *g, ResourceModel *rm, map<Vertex*, int> sched, int II,
                         Binding::RegChainBindingContainer bind,
                         std::set<const Resource*> commutativeOps = {}, bool quiet=true);
	/*!
	 * do the same but with a different binding container type
	 * @param g
	 * @param rm
	 * @param sched
	 * @param II
	 * @param bind
	 * @param portAssignments
	 * @param commutativeOps
	 * @return
	 */
	bool verifyIntIIBinding(Graph *g, ResourceModel *rm, map<Vertex*, int> sched, int II,
                         Binding::BindingContainer bind, std::set<const Resource*> commutativeOps = {}, bool quiet=true);

/*!
 * verify a binding
 * @param g graph
 * @param rm resource model
 * @param sched schedule time container
 * @param samples number of samples included in the scheudle (II = modulo / samples)
 * @param modulo cycle length of the schedule (II = modulo / samples)
 * @param portAssignments this container specifies which edge is connected with which input port of the sink vertex
 * @param commutativeOps a set with commutative operations that do not care about their port assignments
 * @return if the binding is ok
 */
	bool verifyRatIIBinding(Graph *g, ResourceModel *rm, std::vector<map<Vertex*, int>> sched,
                         int samples, int modulo, Binding::RatIIRegChainBindingContainer bind,
                         std::map<Edge*,int> portAssignments = {}, std::set<const Resource*> commutativeOps = {});
}
