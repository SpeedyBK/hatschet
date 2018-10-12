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
#ifdef USE_XERCESC
#include "XMLTargetReader.h"
#include "xercesc/sax2/SAX2XMLReader.hpp"
#include "xercesc/sax2/XMLReaderFactory.hpp"
#include "xercesc/util/XMLString.hpp"
#include "xercesc/sax2/Attributes.hpp"
#include "string"
#include "stack"
#include "iostream"
#include "fstream"

namespace HatScheT {

XMLTargetReader::XMLTargetReader(Target* hw)
{
  this->hardwareTarget = hw;
}

void XMLTargetReader::characters(const XMLCh * const chars, const XMLSize_t length)
{

}

void XMLTargetReader::startElement(const XMLCh * const uri, const XMLCh * const localname, const XMLCh * const qname, const Attributes &attrs)
{
  string tag = XMLString::transcode(localname);

  if(tag=="FPGA"){
    string family = XMLString::transcode(attrs.getValue(XMLString::transcode("family")));
    string name = XMLString::transcode(attrs.getValue(XMLString::transcode("name")));
    string vendor = XMLString::transcode(attrs.getValue(XMLString::transcode("vendor")));

    this->hardwareTarget->setName(name);
    this->hardwareTarget->setFamily(family);
    this->hardwareTarget->setVendor(vendor);
  }

  if(tag=="Element"){
    string name = XMLString::transcode(attrs.getValue(XMLString::transcode("name")));
    string avail = XMLString::transcode(attrs.getValue(XMLString::transcode("avail")));

    this->hardwareTarget->addElement(name, stod(avail));
  }
}

void XMLTargetReader::endElement(const XMLCh * const uri, const XMLCh * const localname, const XMLCh * const qname)
{

}

void XMLTargetReader::readHardwareTarget(const char *path)
{
  try {
    XMLPlatformUtils::Initialize();
  }
  catch (const XMLException& toCatch) {
    char* message = XMLString::transcode(toCatch.getMessage());
    cout << "XMLTargetReader.readFPGA: " << message << endl;
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
      cout << "XMLTargetReader.readFPGA:  File not found! (" << xmlFile << ")" << endl;
    }
  }
  catch (const XMLException& toCatch) {

    char* message = XMLString::transcode(toCatch.getMessage());
    cout << "XMLTargetReader.readFPGA:  " << message << endl;


  }
  catch (const SAXParseException& toCatch) {
    char* message = XMLString::transcode(toCatch.getMessage());
    cout << "XMLTargetReader.readFPGA:  " << message << endl;

  }
  catch (...) {
    cout << "XMLTargetReader.readFPGA: Unexpected Error" << endl;
  }

  delete parser;
  XMLPlatformUtils::Terminate();
}

}

#endif //USE_XERCESC