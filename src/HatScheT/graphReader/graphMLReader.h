
#pragma once
#ifdef USE_XERCESC
#include "graphReader.h"
#include "xercesc/sax2/DefaultHandler.hpp"

using namespace xercesc;
using namespace std;

namespace HatScheT
{
/*!
 * \brief The graphMLReader class use this class to parse graph objects from graphML representations
 */
class GraphMLReader : public GraphReader, DefaultHandler
{
public:
  /*!
     * \brief graphMLReader
     */
  GraphMLReader();
  ~GraphMLReader();
  /*!
     * \brief readGraph the main function of the graphML parser
     * \param path
     * \param readGraph
     * \return
     */
  virtual Graph& readGraph(const char* path) final;
private:

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
   * \brief currVertex
   */
  Vertex* currVertex;
  /*!
   * \brief currEdge
   */
  Edge* currEdge;
  };
}

#endif //USE_XERCESC
