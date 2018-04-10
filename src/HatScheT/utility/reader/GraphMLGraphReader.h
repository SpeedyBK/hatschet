/*
  This file is part of the HatScheT project, developed at University of Kassel, TU Darmstadt
  Author: Patrick Sittel (sittel@uni-kassel.de)
  All rights reserved.
*/
#pragma once
#ifdef USE_XERCESC
#include "Reader.h"

using namespace xercesc;
using namespace std;

namespace HatScheT
{
/*!
 * \brief The graphMLReader class use this class to parse graph objects from graphML representations
 */
class GraphMLGraphReader : public Reader, DefaultHandler
{
public:
  /*!
     * \brief graphMLReader
     */
  GraphMLGraphReader(ResourceModel* rm, Graph *g);
  ~GraphMLGraphReader();
  /*!
     * \brief readGraph the main function of the graphML parser
     * \param path
     * \param readGraph
     * \return
     */
  virtual Graph& readGraph(const char* path) final;
  /*!
   * \brief readResourceModel dont use the function in this class
   * \param path
   * \return
   */
  virtual ResourceModel& readResourceModel(const char* path){
    string p(path);
    throw Exception("GraphMLGraphReader.readResourcemodel: Dont use this class to read resource: " + p);}
private:
  ResourceModel* rm;
  /*!
     * \brief g used to add elemets to the graph during parsing process
     */
  Graph* g;
  /*!
     * \brief startElement
     * \param uri
     * \param localname
     * \param qname
     * \param atts
     */
  void startElement(
      const   XMLCh* const    uri,
      const   XMLCh* const    localname,
      const   XMLCh* const    qname,
      const   Attributes    &atts
      );
  /*!
     * \brief endElement
     * \param uri
     * \param localname
     * \param qname
     */
  void endElement( const XMLCh* const uri,
                   const XMLCh* const localname,
                   const XMLCh* const qname );
  /*!
   * \brief characters
   * \param chars
   * \param length
   */
  void characters    (   const XMLCh *const      chars,
      const XMLSize_t     length);
   /*!
   * \brief nodeTagFound
   */
  bool nodeTagFound;
  /*!
   * \brief edgeTagFound
   */
  bool edgeTagFound;
  /*!
   * \brief nameTagFound
   */
  bool nameTagFound;
  /*!
   * \brief latencyTagFound
   */
  bool latencyTagFound;
  /*!
   * \brief dataTagFound
   */
  bool dataTagFound;
  /*!
   * \brief resourceTagFound
   */
  bool resourceTagFound;
  /*!
   * \brief currVertexId
   */
  int currVertexId;
  int dstId;
  int srcId;
  int edgeLatency;
  string currVertexResName;
  };
}

#endif //USE_XERCESC
