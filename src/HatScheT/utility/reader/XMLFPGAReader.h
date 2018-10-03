/*
    This file is part of the HatScheT project, developed at University of Kassel and TU Darmstadt, Germany
    Patrick Sittel (sittel@uni-kassel.de)

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
 * The XMLFPGAReader class use this class to parse FPGA objects from XML representations
 */
class XMLFPGAReader : public Reader, DefaultHandler {
public:
    XMLFPGAReader(Target* hw);
    ~XMLFPGAReader(){};
    /*!
     * main method to read and return FPGA class from a xml file
     * @param path
     * @return
     */
    virtual Target& readHardwareTarget(const char* path) final;
    /*!
   * \brief readResourceModel dont use the function in this class
   * \param path
   * \return
   */
  virtual ResourceModel& readResourceModel(const char* path){
    string p(path);
    throw Exception("GraphMLGraphReader.readResourcemodel: Dont use this class to read resource: " + p);}
  /*!
   * \brief readGraph Dont use this class to read graphs!
   * \param path
   * \param readGraph
   * \return
   */
  virtual Graph& readGraph(const char* path){
    string p(path);
    throw Exception("GraphMLResourceReader.readGraph: Dont use this class to read graph: " + p);}
private:
    Target* hardwareTarget;
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
};

}

#endif //USE_XERCESC