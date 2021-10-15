//
// Created by nfiege on 7/5/21.
//

#ifndef HATSCHET_DGLWRITER_H
#define HATSCHET_DGLWRITER_H

#include <HatScheT/utility/writer/Writer.h>
#include <HatScheT/Graph.h>
#include <HatScheT/ResourceModel.h>
#include <map>
#include <string>

namespace HatScheT {

	class DGLWriter : public Writer {
	public:
		/*!
		 * constructor
		 * @param path file path
		 * filename 'example_graph' would produce two following files
		 * 	  - example_graph_nodes.csv with node data
		 * 	  - example_graph_edges.csv with edge data
		 * @param g graph
		 * @param rm resource model
		 * @param II initiation interval
		 * @param schedule (optimal) schedule
		 */
		DGLWriter(std::string path, Graph* g, ResourceModel* rm, int II, std::map<Vertex *, int> &schedule);
		/*!
		 * base method we override
		 */
		void write() override;

	private:
		/*!
		 * graph to write
		 */
		Graph* g;
		/*!
		 * associated resource model
		 */
		ResourceModel* rm;
		/*!
		 * initiation interval
		 */
		int II;
		/*!
		 * (optimal) schedule for the graph/rm
		 */
		std::map<Vertex *, int> schedule;
	};
}


#endif //HATSCHET_DGLWRITER_H
