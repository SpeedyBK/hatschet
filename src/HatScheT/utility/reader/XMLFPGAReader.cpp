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
#include "XMLFPGAReader.h"
#include "xercesc/sax2/SAX2XMLReader.hpp"
#include "xercesc/sax2/XMLReaderFactory.hpp"
#include "xercesc/util/XMLString.hpp"
#include "xercesc/sax2/Attributes.hpp"
#include "string"
#include "stack"
#include "iostream"
#include "fstream"

namespace HatScheT {

XMLFPGAReader::XMLFPGAReader(FPGALayer* fpga)
{
  if(fpga->getVendor()==FPGAVendor::XILINX){
    this->xilinxfpga = (XilinxFPGA*)fpga;
  }
  else throw HatScheT::Exception("XMLFPGAReader::XMLFPGAReader: Only xilinx FPGA supported for now! Plz provide a 'xilinx' FPGA!");
}

void XMLFPGAReader::characters(const XMLCh * const chars, const XMLSize_t length)
{

}

void XMLFPGAReader::startElement(const XMLCh * const uri, const XMLCh * const localname, const XMLCh * const qname, const Attributes &attrs)
{
  string tag = XMLString::transcode(localname);

  if(tag=="FPGA"){
    string family = XMLString::transcode(attrs.getValue(XMLString::transcode("family")));
    string name = XMLString::transcode(attrs.getValue(XMLString::transcode("name")));
    string LUTs = XMLString::transcode(attrs.getValue(XMLString::transcode("LUTs")));
    string Slices = XMLString::transcode(attrs.getValue(XMLString::transcode("Slices")));
    string DSPs = XMLString::transcode(attrs.getValue(XMLString::transcode("DSPs")));
    string BRAMs = XMLString::transcode(attrs.getValue(XMLString::transcode("BRAMs")));

    this->xilinxfpga->setName(name);
    this->xilinxfpga->setFamily(family);

    this->xilinxfpga->setTotalLUTs(stoi(LUTs));
    this->xilinxfpga->setTotalSlices(stoi(Slices));
    this->xilinxfpga->setTotalDSPs(stoi(DSPs));
    this->xilinxfpga->setTotalBRAMs(stoi(BRAMs));
  }
}

void XMLFPGAReader::endElement(const XMLCh * const uri, const XMLCh * const localname, const XMLCh * const qname)
{

}

XilinxFPGA& XMLFPGAReader::readFPGA(const char *path)
{
  try {
    XMLPlatformUtils::Initialize();
  }
  catch (const XMLException& toCatch) {
    char* message = XMLString::transcode(toCatch.getMessage());
    cout << "XMLFPGAReader.readFPGA: " << message << endl;
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
      cout << "XMLFPGAReader.readFPGA:  File not found! (" << xmlFile << ")" << endl;
    }
  }
  catch (const XMLException& toCatch) {

    char* message = XMLString::transcode(toCatch.getMessage());
    cout << "XMLFPGAReader.readFPGA:  " << message << endl;


  }
  catch (const SAXParseException& toCatch) {
    char* message = XMLString::transcode(toCatch.getMessage());
    cout << "XMLFPGAReader.readFPGA:  " << message << endl;

  }
  catch (...) {
    cout << "XMLFPGAReader.readFPGA: Unexpected Error" << endl;
  }

  delete parser;
  XMLPlatformUtils::Terminate();

  return *(this->xilinxfpga);
}

}

#endif //USE_XERCESC