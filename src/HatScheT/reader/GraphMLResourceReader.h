
#pragma once
#ifdef USE_XERCESC
#include "Reader.h"
#include "xercesc/sax2/DefaultHandler.hpp"
#include "../Exception.h"

using namespace xercesc;
using namespace std;

namespace HatScheT
{
/*!
 * \brief The graphMLReader class use this class to parse resource objects from graphML representations
 */
class GraphMLResourceReader : public Reader, DefaultHandler
{
public:
  /*!
     * \brief graphMLReader
     */
  GraphMLResourceReader();
  ~GraphMLResourceReader();
  /*!
     * \brief readGraph the main function of the graphML parser
     * \param path
     * \param readGraph
     * \return
     */
  virtual Graph& readGraph(const char* path){
    string p(path);
    throw Exception("GraphMLResourceReader.readGraph: Dont use this class to read graph: " + p);}
  /*!
   * \brief readResourceModel dont use the function in this class
   * \param path
   * \return
   */
  virtual ResourceModel& readResourceModel(const char* path, Graph& g);
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
   * \brief dataTagFound
   */
  bool dataTagFound;
  /*!
   * \brief resourceTagFound
   */
  bool resourceTagFound;
  };
}

#endif //USE_XERCESC
