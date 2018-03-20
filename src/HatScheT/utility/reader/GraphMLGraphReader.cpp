#ifdef USE_XERCESC
#include "GraphMLGraphReader.h"
#include "xercesc/sax2/SAX2XMLReader.hpp"
#include "xercesc/sax2/XMLReaderFactory.hpp"
#include "xercesc/util/XMLString.hpp"
#include "xercesc/sax2/Attributes.hpp"
#include "string"
#include "stack"
#include "iostream"
#include "fstream"
#include <algorithm>

namespace HatScheT {

GraphMLGraphReader::GraphMLGraphReader(ResourceModel *rm, Graph* g)
{
  this->g = g;
  this->rm = rm;

  this->nodeTagFound = false;
  this->edgeTagFound = false;
  this->nameTagFound = false;
  this->latencyTagFound = false;
  this->dataTagFound = false;
  this->resourceTagFound = false;

  this->currVertexId = -1;
  this->dstId = -1;
  this->srcId = -1;
  this->edgeLatency = -1;
}

GraphMLGraphReader::~GraphMLGraphReader()
{

}

void GraphMLGraphReader::endElement(const XMLCh * const uri, const XMLCh * const localname, const XMLCh * const qname)
{
  string name = XMLString::transcode(localname);

  if(name == "node" && this->nodeTagFound == true)
  {
    Vertex* v = &this->g->createVertex(this->currVertexId);
    Resource* r = this->rm->getResource(this->currVertexResName);

    this->rm->registerVertex(v, r);

    this->nodeTagFound = false;
  }

  if(name == "edge")
  {
    Vertex& source = this->g->getVertexById(this->srcId);
    Vertex& target = this->g->getVertexById(this->dstId);

    this->g->createEdge(source, target, this->edgeLatency);

    this->edgeTagFound = false;
  }

  if(name == "data")
  {
    this->dataTagFound = false;

    if(this->nameTagFound == true) this->nameTagFound = false;
    if(this->resourceTagFound == true) this->resourceTagFound = false;
  }
}

void GraphMLGraphReader::characters(const XMLCh * const chars, const XMLSize_t length)
{
  if(this->dataTagFound == true)
  {
    if(this->nodeTagFound == true)
    {
      if(this->nameTagFound)
      {

      }

      if(this->resourceTagFound)
      {
        std::string name = XMLString::transcode(chars);
        //remove whitespaces
        std::string::iterator end_pos = std::remove(name.begin(), name.end(), ' ');
        name.erase(end_pos, name.end());

        this->currVertexResName = name;
      }
    }

    if(this->edgeTagFound == true)
    {
      if(this->latencyTagFound)
      {
         this->edgeLatency = atoi(XMLString::transcode(chars));
      }
    }
  }
}

void GraphMLGraphReader::startElement(const XMLCh * const uri, const XMLCh * const localname, const XMLCh * const qname, const Attributes &attrs)
{
  string name = XMLString::transcode(localname);

  if(name == "node")
  {
    string idstring = XMLString::transcode(attrs.getValue(XMLString::transcode("id")));

    if(idstring != "modsched_info")
    {
      this->nodeTagFound = true;
      this->currVertexId = stoi(idstring);
    }
  }

  if(name == "edge")
  {
    this->edgeTagFound = true;

    string sourcestring = XMLString::transcode(attrs.getValue(XMLString::transcode("source")));
    string targetstring = XMLString::transcode(attrs.getValue(XMLString::transcode("target")));

    this->srcId = stoi(sourcestring);
    this->dstId = stoi(targetstring);
  }

  if(name == "data")
  {
    this->dataTagFound = true;
    string key = XMLString::transcode(attrs.getValue(XMLString::transcode("key")));

    if(this->nodeTagFound)
    {
      if(key == "name")
      {
          this->nameTagFound = true;
      }

      if (key == "uses_resource")
      {
          this->resourceTagFound = true;
      }
    }

    if(this->edgeTagFound)
    {
      if(key == "latency")
      {
        this->latencyTagFound = true;
      }
    }
  }

}

Graph& GraphMLGraphReader::readGraph(const char *path)
{
  cout << "graphMLReader.parseGraph: Start parsing from path: " << path << endl;

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
      cout << "graphMLReader.parseGraph: File not fount! (" << xmlFile << ")" << endl;
    }
  }
  catch (const XMLException& toCatch) {

    char* message = XMLString::transcode(toCatch.getMessage());
    cout << "graphMLReader.parseGraph: " << message << endl;


  }
  catch (const SAXParseException& toCatch) {
    char* message = XMLString::transcode(toCatch.getMessage());
    cout << "graphMLReader.parseGraph: " << message << endl;

  }
  catch (...) {
    cout << "graphMLReader.parseGraph: Unexpected Error" << endl;
  }

  delete parser;

  cout << "graphMLReader.parseGraph: Finished parsing from path: " << path << endl;

  return *(this->g);
}

}

#endif //USE_XERCESC
