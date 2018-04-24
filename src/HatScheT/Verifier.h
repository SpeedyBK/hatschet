#pragma once

#include "Graph.h"
#include "ResourceModel.h"

#include <map>

namespace HatScheT {

/*!
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
bool verifyResourceConstrainedSchedule(Graph &g, ResourceModel &rm, std::map<Vertex *, int> &schedule, int SL);

/*!
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
bool verifyModuloSchedule(Graph &g, ResourceModel &rm, std::map<Vertex *, int> &schedule, int II);

}
