//
// Created by nfiege on 6/7/21.
//

#ifndef HATSCHET_SCHEDULEANDBINDINGREADER_H
#define HATSCHET_SCHEDULEANDBINDINGREADER_H

#include <vector>
#include <map>
#include <string>
#include "HatScheT/Graph.h"
#include "HatScheT/ResourceModel.h"

namespace HatScheT {
	class ScheduleAndBindingReader {
	public:
		/*!
		 * constructor
		 * @param g graph belonging to the file that we want to read
		 * @param rm resource model belonging to the file that we want to read
		 */
		ScheduleAndBindingReader(Graph* g, ResourceModel* rm);
		/*!
		 * read file and store contents
		 * @param filepath
		 */
		void read(const std::string &filepath);
		/*!
		 * @return true if it's a rational-II schedule and false if it's an integer-II schedule
		 */
		bool isRatII() const;
		/*!
		 * get schedule when we just read an integer-II schedule file
		 * @return
		 */
		std::map<Vertex*,int> getIntegerIISchedule() const;
		/*!
		 * get schedule when we just read a rational-II schedule file
		 * @return
		 */
		std::vector<std::map<Vertex*,int>> getRationalIISchedule() const;
		/*!
		 * get binding when we just read an integer-II schedule file
		 * @return
		 */
		std::map<Vertex*,int> getIntegerIIBinding() const;
		/*!
		 * get schedule when we just read a rational-II schedule file
		 * @return
		 */
		std::vector<std::map<Vertex*,int>> getRationalIIBinding() const;
		/*!
		 * get number of samples in rational-II schedule
		 * @return
		 */
		int getSamples() const;
		/*!
		 * get cycle length in rational-II schedule
		 * @return
		 */
		int getModulo() const;
		/*!
		 * get initiation interval
		 * @return
		 */
		double getII() const;
		/*!
		 * get schedule length
		 * @return
		 */
		int getScheduleLength() const;
		/*!
		 * get minimum number of registers
		 * @return
		 */
		int getMinNumRegs() const ;

	private:
		/*!
		 * read a line of the file header
		 * @param line
		 */
		void readHeader(std::string &line);
		/*!
		 * graph
		 */
		Graph* g;
		/*!
		 * resource model
		 */
		ResourceModel* rm;
		/*!
		 * schedule times container
		 */
		std::vector<std::map<Vertex*,int>> schedule;
		/*!
		 * binding info container
		 */
		std::vector<std::map<Vertex*,int>> binding;
		/*!
		 * number of samples in rat-II schedule
		 */
		int samples;
		/*!
		 * cycle length in rat-II schedule
		 */
		int modulo;
		/*!
		 * initiation interval
		 */
		double II;
		/*!
		 * length of the schedule that we just read
		 */
		int scheduleLength;
		/*!
		 * minimum number of registers needed to implement this schedule
		 */
		int minNumRegs;
	};
}

#endif //HATSCHET_SCHEDULEANDBINDINGREADER_H
