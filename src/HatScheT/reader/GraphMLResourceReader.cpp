#ifdef USE_XERCESC
#include "../ResourceModel.h"
#include "GraphMLResourceReader.h"
#include "xercesc/sax2/SAX2XMLReader.hpp"
#include "xercesc/sax2/XMLReaderFactory.hpp"
#include "xercesc/util/XMLString.hpp"
#include "xercesc/sax2/Attributes.hpp"
#include "string"
#include "stack"
#include "iostream"
#include "fstream"

namespace HatScheT {

GraphMLResourceReader::GraphMLResourceReader()
{
  this->rm = new ResourceModel();

  this->resourceTagFound = false;
  this->dataTagFound = false;
  this->minIITagFound = false;
  this->maxIITagFound = false;
  this->minII = -1;
  this->maxII = -1;
}

GraphMLResourceReader::~GraphMLResourceReader()
{

}

void GraphMLResourceReader::endElement(const XMLCh * const uri, const XMLCh * const localname, const XMLCh * const qname)
{


}

void GraphMLResourceReader::characters(const XMLCh * const chars, const XMLSize_t length)
{
  if(this->minIITagFound == true)
  {
    this->minII = atoi(XMLString::transcode(chars));
  }

  if(this->maxIITagFound == true)
  {
    this->maxII = atoi(XMLString::transcode(chars));
  }
}

void GraphMLResourceReader::startElement(const XMLCh * const uri, const XMLCh * const localname, const XMLCh * const qname, const Attributes &attrs)
{
  string name = XMLString::transcode(localname);

  if(name == "data")
  {
    string keystring = XMLString::transcode(attrs.getValue(XMLString::transcode("key")));

    if(keystring == "minII")
    {
      this->minIITagFound = true;
    }

    if(keystring == "maxII")
    {
      this->maxIITagFound = true;
    }
  }

  if(name == "resource")
  {
    string idstring = XMLString::transcode(attrs.getValue(XMLString::transcode("id")));
    string limitstring = XMLString::transcode(attrs.getValue(XMLString::transcode("limit")));
    string isunlimiedtstring = XMLString::transcode(attrs.getValue(XMLString::transcode("isUnlimited")));
    string latency = XMLString::transcode(attrs.getValue(XMLString::transcode("latency")));

    if(isunlimiedtstring == "false")
    {
      auto res = this->rm->makeResource(idstring, stoi(limitstring));
      this->rm->makeSimpleReservationTable(stoi(latency), res);
    }
  }
}

ResourceModel& GraphMLResourceReader::readResourceModel(const char *path)
{
  cout << "GraphMLResourceReader.readResourceModel: Start parsing from path: " << path << endl;

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
      cout << "GraphMLResourceReader.readResourceModel:  File not fount! (" << xmlFile << ")" << endl;
    }
  }
  catch (const XMLException& toCatch) {

    char* message = XMLString::transcode(toCatch.getMessage());
    cout << "GraphMLResourceReader.readResourceModel:  " << message << endl;


  }
  catch (const SAXParseException& toCatch) {
    char* message = XMLString::transcode(toCatch.getMessage());
    cout << "GraphMLResourceReader.readResourceModel:  " << message << endl;

  }
  catch (...) {
    cout << "GraphMLResourceReader.readResourceModel: Unexpected Error" << endl;
  }

  delete parser;

  cout << "GraphMLResourceReader.readResourceModel:  Finished parsing from path: " << path << endl;

  return *(this->rm);
}

}

#endif //USE_XERCESC
