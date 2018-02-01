#ifdef USE_XERCESC
#include "../../ResourceModel.h"
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
  this->reservationTableTagFound = false;
  this->blogTagFound = false;
  this->currRtName = "";
}

GraphMLResourceReader::~GraphMLResourceReader()
{

}

void GraphMLResourceReader::endElement(const XMLCh * const uri, const XMLCh * const localname, const XMLCh * const qname)
{
  string name = XMLString::transcode(localname);

  if(name == "rt")
  {
    this->rm->makeReservationTable(this->currRtName);

    this->reservationTableTagFound = false;
    this->blocks.clear();
  }
}

void GraphMLResourceReader::characters(const XMLCh * const chars, const XMLSize_t length)
{

}

void GraphMLResourceReader::startElement(const XMLCh * const uri, const XMLCh * const localname, const XMLCh * const qname, const Attributes &attrs)
{
  string tag = XMLString::transcode(localname);

  if(tag == "resource")
  {
    string namestring = XMLString::transcode(attrs.getValue(XMLString::transcode("name")));
    string limitstring = XMLString::transcode(attrs.getValue(XMLString::transcode("limit")));
    string latencystring = XMLString::transcode(attrs.getValue(XMLString::transcode("latency")));
    string blockingtimestring = XMLString::transcode(attrs.getValue(XMLString::transcode("blockingTime")));

    int limit = -1;
    if(limitstring != "inf") limit = stoi(limitstring);

    this->rm->makeResource(namestring, limit, stoi(latencystring), stoi(blockingtimestring));
  }

  if(tag == "rt")
  {
    this->reservationTableTagFound = true;

    this->currRtName = XMLString::transcode(attrs.getValue(XMLString::transcode("name")));
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
