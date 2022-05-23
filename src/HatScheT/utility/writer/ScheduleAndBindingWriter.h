//
// Created by nfiege on 03/12/19.
//

#ifndef HATSCHET_SCHEDULEANDBINDINGWRITER_H
#define HATSCHET_SCHEDULEANDBINDINGWRITER_H

#include <HatScheT/utility/writer/Writer.h>
#include <HatScheT/Vertex.h>
#include <vector>
#include <map>
#include <string>

namespace HatScheT {

	class ScheduleAndBindingWriter : public Writer {

	public:
		struct fuConnection {
			/*!
			 * source resource name
			 */
			std::string resourceSrc;
			/*!
			 * source fu number
			 */
			int fuSrc;
			/*!
			 * destination resource name
			 */
			std::string resourceDst;
			/*!
			 * destination fu number
			 */
			int fuDst;
			/*!
			 * destination input port number
			 */
			int port;
			/*!
			 * number of lifetime regs of that connection
			 */
			int lifetimeRegs;
		};
		/*!
		 * rational II constructor
		 * @param path
		 * @param schedule
		 * @param binding
		 * @param samples
		 * @param modulo
		 */
		ScheduleAndBindingWriter(std::string path, std::vector<std::map<Vertex *, int>> &schedule,
														 std::vector<std::map<const Vertex *, int>> &binding, int samples, int modulo,
														 std::vector<fuConnection> fuConnections = {});
		/*!
		 * integer II constructor
		 * @param path
		 * @param schedule
		 * @param binding
		 * @param II
		 */
		ScheduleAndBindingWriter(std::string path, std::map<Vertex *, int> &schedule,
			                       std::map<const Vertex *, int> &binding, int II,
														 std::vector<fuConnection> fuConnections = {});
		/*!
		 * base method we override
		 */
		void write() override;
		/*!
		 * @param g
		 */
		void setGraphPath(std::string g) { this->graphPath = std::move(g); }
		/*!
		 * @param r
		 */
		void setRMPath(std::string r) { this->rmPath = std::move(r); }
		/*!
		 * sets the solving time
		 * @param solvingTime new value for the solving time
		 */
		void setSolvingTime(const double & newSolvingTime) { this->solvingTime = newSolvingTime; }
		/*!
		 * sets the schedule length
		 * @param newScheduleLength new value for the schedule length
		 */
		void setScheduleLength(const int & newScheduleLength) { this->scheduleLength = newScheduleLength; }
		/*!
		 * sets the minimum number of registers to implement the schedule
		 * @param newMinNumRegs new value for the minimum number of registers
		 */
		void setMinNumRegs(const int & newMinNumRegs) { this->minNumRegs = newMinNumRegs; }
		/*!
		 * sets the minimum number of registers to implement the schedule
		 * @param newMinNumRegs new value for the minimum number of registers
		 */
		void setMinNumRegsChain(const int & newMinNumRegsChain) { this->minNumRegsChain = newMinNumRegsChain; }
		/*!
		 * sets the minimum II
		 * @param newMinNumRegs new value for the minimum II
		 */
		void setMinII(const double & newMinII) { this->minII = newMinII; }
		/*!
		 * sets this->objectivesOptimal
		 * @param o new value for this->objectivesOptimal
		 */
		void setObjectivesOptimal(const std::pair<bool, bool> o) { this->objectivesOptimal = o; }

	private:
		/*!
		 * tracks whether the first and second optimization objectives (usually II and schedule length) are proven optimal
		 */
		std::pair<bool, bool> objectivesOptimal;
		/*!
		 * minimum II for this graph and resource model
		 */
		double minII;
		/*!
		 * minimum number of registers needed to implement this schedule
		 * -> maximum number of concurrently alive variables in any time step mod cycle length (= II/modulo for intII/ratII)
		 */
		int minNumRegs;
		/*!
		 * minimum number of registers needed to implement this schedule
		 * when implementing registers as register chains after FUs
		 * -> maximum lifetime of all variables produced by that FU
		 */
		int minNumRegsChain;
		/*!
		 * schedule length = max (t_i + D_i of all vertices i in the schedule)
		 */
		int scheduleLength;
		/*!
		 * elapsed time of the scheduling/binding algorithm to obtain the given solution
		 */
		double solvingTime;
		/*!
		 * connections between FUs (do not have to be specified if multiplexer port assignment should not be included)
		 */
		std::vector<fuConnection> fuConnections;
		/*!
		 * path to the graph associated to the schedule/binding
		 */
		std::string graphPath;
		/*!
		 * path to the resource model associated to the schedule/binding
		 */
		std::string rmPath;
		/*!
		 * rational II schedule container
		 * => integer II schedule has vector size=1
		 */
		std::vector<std::map<Vertex *, int>> schedule;
		/*!
		 * rational II binding container
		 * => integer II binding has vector size=1
		 */
		std::vector<std::map<const Vertex *, int>> binding;
		/*!
		 * number of samples
		 * => integer II modulo schedule has samples=1
		 */
		int samples;
		/*!
		 * cycle length
		 * => integer II modulo schedule has modulo=II
		 */
		int modulo;
		/*!
		 * struct where a line of the csv file is stored
		 */
		struct fileLine {
			/*!
			 * vertex
			 */
			const Vertex* v = nullptr;
			/*!
			 * name of the vertex
			 */
			std::string name;
			/*!
			 * the sample number this line belongs to (there are 'this->samples' number of lines per vertex)
			 */
			int sample = -1;
			/*!
			 * cycle time when this vertex is scheduled
			 */
			int cycle = -1;
			/*!
			 * functional unit this vertex is bind to
			 */
			int fu = -1;
		};
	};
}

#endif //HATSCHET_SCHEDULEANDBINDINGWRITER_H
