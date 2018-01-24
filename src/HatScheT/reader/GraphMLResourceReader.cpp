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
  this->nodeTagFound = false;
  this->resourceTagFound = false;
  this->dataTagFound = false;
}

GraphMLResourceReader::~GraphMLResourceReader()
{

}

void GraphMLResourceReader::endElement(const XMLCh * const uri, const XMLCh * const localname, const XMLCh * const qname)
{


}

void GraphMLResourceReader::characters(const XMLCh * const chars, const XMLSize_t length)
{

}

void GraphMLResourceReader::startElement(const XMLCh * const uri, const XMLCh * const localname, const XMLCh * const qname, const Attributes &attrs)
{

}

ResourceModel& GraphMLResourceReader::readResourceModel(const char *path,Graph& g)
{
  cout << "GraphMLResourceReader.readResourceModel: Start parsing from path: " << path << endl;
  ResourceModel rm = ResourceModel(g);

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

  return rm;
}

}

#endif //USE_XERCESC
