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
		void read(std::string filepath);
		/*!
		 * @return true if it's a rational-II schedule and false if it's an integer-II schedule
		 */
		bool isRatII();
		/*!
		 * get schedule when we just read an integer-II schedule file
		 * @return
		 */
		std::map<Vertex*,int> getIntegerIISchedule();
		/*!
		 * get schedule when we just read a rational-II schedule file
		 * @return
		 */
		std::vector<std::map<Vertex*,int>> getRationalIISchedule();
		/*!
		 * get binding when we just read an integer-II schedule file
		 * @return
		 */
		std::map<Vertex*,int> getIntegerIIBinding();
		/*!
		 * get schedule when we just read a rational-II schedule file
		 * @return
		 */
		std::vector<std::map<Vertex*,int>> getRationalIIBinding();
		/*!
		 * get number of samples in rational-II schedule
		 * @return
		 */
		int getSamples();
		/*!
		 * get cycle length in rational-II schedule
		 * @return
		 */
		int getModulo();
		/*!
		 * get initiation interval
		 * @return
		 */
		double getII();

	private:
		/*!
		 * read a line of the file header
		 * @param line
		 */
		void readHeader(std::string line);
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
	};
}

#endif //HATSCHET_SCHEDULEANDBINDINGREADER_H
