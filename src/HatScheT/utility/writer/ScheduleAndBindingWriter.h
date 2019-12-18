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
		/*!
		 * rational II constructor
		 * @param path
		 * @param schedule
		 * @param binding
		 * @param samples
		 * @param modulo
		 */
		ScheduleAndBindingWriter(std::string path, std::vector<std::map<Vertex *, int>> &schedule,
														 std::vector<std::map<const Vertex *, int>> &binding, int samples, int modulo);
		/*!
		 * integer II constructor
		 * @param path
		 * @param schedule
		 * @param binding
		 * @param II
		 */
		ScheduleAndBindingWriter(std::string path, std::map<Vertex *, int> &schedule, std::map<const Vertex *, int> &binding, int II);
		/*!
		 * base method we override
		 */
		void write() override;
		/*!
		 * @param g
		 */
		void setGraphPath(std::string g) { this->graphPath = g; }
		/*!
		 * @param r
		 */
		void setRMPath(std::string r) { this->rmPath = r; }

	private:
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
			std::string name = "";
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
