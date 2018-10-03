/*
    This file is part of the HatScheT project, developed at University of Kassel and TU Darmstadt, Germany
    Author: Patrick Sittel (sittel@uni-kassel.de)

    Copyright (C) 2018

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

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
  GraphMLResourceReader(ResourceModel* rm);
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
   * dont use the function in this class
   * @param path
   * @return
   */
    virtual Target& readHardwareTarget(const char* path){
      string p(path);
      throw Exception("GraphMLResourceReader.readHardwareTarget: Dont use this class to read hardware targets: " + p);}
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
   * \brief resourceTagFound
   */
   bool resourceTagFound;
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
  /*!
   * store the name of the current resource to add costs later
   */
  string currentResourceParsing;
  };
}

#endif //USE_XERCESC
