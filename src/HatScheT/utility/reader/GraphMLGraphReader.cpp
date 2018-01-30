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

GraphMLGraphReader::GraphMLGraphReader(ResourceModel *rm)
{
  this->g = HatScheT::Graph();
  this->rm = rm;

  this->nodeTagFound = false;
  this->edgeTagFound = false;
  this->nameTagFound = false;
  this->latencyTagFound = false;
  this->dataTagFound = false;
  this->resourceTagFound = false;
}

GraphMLGraphReader::~GraphMLGraphReader()
{

}

void GraphMLGraphReader::endElement(const XMLCh * const uri, const XMLCh * const localname, const XMLCh * const qname)
{
  string name = XMLString::transcode(localname);

  if(name == "node" && this->nodeTagFound == true)
  {
    this->g.addVertex(*(this->currVertex), this->currVertex->getId());

    this->nodeTagFound = false;
  }

  if(name == "edge")
  {
    this->g.addEdge(*(this->currEdge));

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
        this->currVertex->setName(XMLString::transcode(chars));
      }

      if(this->resourceTagFound)
      {
        std::string name = XMLString::transcode(chars);
        //remove whitespaces
        std::string::iterator end_pos = std::remove(name.begin(), name.end(), ' ');
        name.erase(end_pos, name.end());

        if(this->rm->resourceExists(name) == true)
        {
          auto rt = this->rm->getRelatedRtByName(name);
          this->rm->registerVertex(this->currVertex, rt);
        }
      }
    }

    if(this->edgeTagFound == true)
    {
      if(this->latencyTagFound)
      {
         this->currEdge->setDelay(atoi(XMLString::transcode(chars)));
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
      Vertex* v = new Vertex(stoi(idstring));

      this->currVertex = v;
    }
  }

  if(name == "edge")
  {
    this->edgeTagFound = true;

    string sourcestring = XMLString::transcode(attrs.getValue(XMLString::transcode("source")));
    string targetstring = XMLString::transcode(attrs.getValue(XMLString::transcode("target")));

    Vertex& source = this->g.getVertex(stoi(sourcestring));
    Vertex& target = this->g.getVertex(stoi(targetstring));

    Edge* e = new Edge(source, target);
    this->currEdge = e;
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

      if (key == "uses_resource_kind")
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

  return (this->g);
}

}

#endif //USE_XERCESC
