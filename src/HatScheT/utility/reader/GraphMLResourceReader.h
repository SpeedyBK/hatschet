
#pragma once
#ifdef USE_XERCESC
#include "Reader.h"

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
     * \brief readGraph Dont use this class to read graphs!
     * \param path
     * \param readGraph
     * \return
     */
  virtual Graph& readGraph(const char* path){
    string p(path);
    throw Exception("GraphMLResourceReader.readGraph: Dont use this class to read graph: " + p);}
  /*!
   * \brief readResourceModel use this class to read resource models from a graphml representation
   * see the attached chstone benchmark for info about how you resource should be build in this case
   * this function will generate a new resourcemodel object instance
   * \param path
   * \return
   */
  virtual ResourceModel& readResourceModel(const char* path);
private:
  /*!
   * \brief rm
   */
  ResourceModel* rm;
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
   * \brief reservationTableTagFound
   */
  bool reservationTableTagFound;
  /*!
   * \brief blogTagFound
   */
  bool blogTagFound;
  /*!
   * \brief currRt
   */
  ReservationTable* currRt;
  };
}

#endif //USE_XERCESC
