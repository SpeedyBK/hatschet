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

#ifdef USE_XERCESC
#include "../../ResourceModel.h"
#include "XMLResourceReader.h"
#include "xercesc/sax2/SAX2XMLReader.hpp"
#include "xercesc/sax2/XMLReaderFactory.hpp"
#include "xercesc/util/XMLString.hpp"
#include "xercesc/sax2/Attributes.hpp"
#include "string"
#include "stack"
#include "iostream"
#include "fstream"

namespace HatScheT {

XMLResourceReader::XMLResourceReader(ResourceModel* rm)
{
  this->rm = rm;
  this->reservationTableTagFound = false;
  this->blogTagFound = false;
  this->resourceTagFound = false;
  this->currentResourceParsing ="";
}

XMLResourceReader::~XMLResourceReader()
{

}

void XMLResourceReader::endElement(const XMLCh * const uri, const XMLCh * const localname, const XMLCh * const qname)
{
  string name = XMLString::transcode(localname);

  if(name == "rt") {
    this->reservationTableTagFound = false;
  }

  if(name == "resource") {
    this->resourceTagFound = false;
  }

}

void XMLResourceReader::characters(const XMLCh * const chars, const XMLSize_t length)
{

}

void XMLResourceReader::startElement(const XMLCh * const uri, const XMLCh * const localname, const XMLCh * const qname, const Attributes &attrs)
{
  string tag = XMLString::transcode(localname);

  if(tag == "Resource") {
    this->resourceTagFound = true;

    string namestring = XMLString::transcode(attrs.getValue(XMLString::transcode("name")));
    string limitstring = XMLString::transcode(attrs.getValue(XMLString::transcode("limit")));
    string latencystring = XMLString::transcode(attrs.getValue(XMLString::transcode("latency")));
    string blockingtimestring = XMLString::transcode(attrs.getValue(XMLString::transcode("blockingTime")));
    string physicaldelaystring = XMLString::transcode(attrs.getValue(XMLString::transcode("physicalDelay")));

    int limit = -1;
    if(limitstring != "inf") limit = stoi(limitstring);

    auto r = &this->rm->makeResource(namestring, limit, stoi(latencystring), stoi(blockingtimestring));
    r->setPhysicalDelay(stod(physicaldelaystring));
    this->currentResourceParsing = namestring;
  }

  if(tag == "rt") {
    this->reservationTableTagFound = true;

    this->currRt = &(this->rm->makeReservationTable(XMLString::transcode(attrs.getValue(XMLString::transcode("name")))));
  }

  if(tag == "block") {
    string resourcenamestring = XMLString::transcode(attrs.getValue(XMLString::transcode("resourceName")));
    int startTime = stoi(XMLString::transcode(attrs.getValue(XMLString::transcode("startTime"))));
    Resource* r = this->rm->getResource(resourcenamestring);

    this->currRt->makeReservationBlock(r, startTime);
  }

  if(tag == "Cost" && this->resourceTagFound == true) {
    string namestring = XMLString::transcode(attrs.getValue(XMLString::transcode("name")));
    string demandstring = XMLString::transcode(attrs.getValue(XMLString::transcode("demand")));

    auto r = this->rm->getResource(this->currentResourceParsing);
    r->addHardwareCost(namestring,stof(demandstring));
  }
}

    void XMLResourceReader::readResourceModel(const char *path)
{
  try {
    XMLPlatformUtils::Initialize();
  }
  catch (const XMLException& toCatch) {
    char* message = XMLString::transcode(toCatch.getMessage());
    cout << "graphMLReader.parseGraph: " << message << endl;
  }

  const char* xmlFile = path;
  SAX2XMLReader* parser = XMLReaderFactory::createXMLReader();
  parser->setFeature(XMLUni::fgSAX2CoreValidation, true);
  parser->setFeature(XMLUni::fgSAX2CoreNameSpaces, true);

  parser->setContentHandler(this);
  parser->setErrorHandler(this);

  try {
    //check for valid path of xmlfile
    //------------------------------------
    std::ifstream infile(xmlFile);
    if (infile.good())
    {
      // start parsing
      //---------------------------------
      parser->parse(xmlFile);
    }

    else
    {
      cout << "XMLResourceReader.readResourceModel:  File not found! (" << xmlFile << ")" << endl;
    }
  }
  catch (const XMLException& toCatch) {

    char* message = XMLString::transcode(toCatch.getMessage());
    cout << "XMLResourceReader.readResourceModel:  " << message << endl;


  }
  catch (const SAXParseException& toCatch) {
    char* message = XMLString::transcode(toCatch.getMessage());
    cout << "XMLResourceReader.readResourceModel:  " << message << endl;

  }
  catch (...) {
    cout << "XMLResourceReader.readResourceModel: Unexpected Error" << endl;
  }

  delete parser;
  XMLPlatformUtils::Terminate();
}

}

#endif //USE_XERCESC
