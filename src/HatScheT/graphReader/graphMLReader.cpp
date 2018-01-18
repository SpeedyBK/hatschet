#ifdef USE_XERCESC
#include "graphMLReader.h"
#include "xercesc/sax2/SAX2XMLReader.hpp"
#include "xercesc/sax2/XMLReaderFactory.hpp"
#include "xercesc/util/XMLString.hpp"
#include "xercesc/sax2/Attributes.hpp"
#include "string"
#include "stack"
#include "iostream"
#include "fstream"

namespace HatScheT {

GraphMLReader::GraphMLReader()
{
  this->nodeTagFound = false;
  this->edgeTagFound = false;
  this->nameTagFound = false;
  this->latencyTagFound = false;
  this->dataTagFound = false;
}

GraphMLReader::~GraphMLReader()
{

}

void GraphMLReader::endElement(const XMLCh * const uri, const XMLCh * const localname, const XMLCh * const qname)
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

    if(this->nameTagFound) this->nameTagFound = false;
    else if(this->latencyTagFound) this->latencyTagFound = false;
    //else if(this->backwardTagFound) this->backwardTagFound = false;
  }

}

void GraphMLReader::characters(const XMLCh * const chars, const XMLSize_t length)
{
  if(this->dataTagFound == true)
  {
    if(this->nodeTagFound == true)
    {
      if(this->nameTagFound)
      {
        this->currVertex->setName(XMLString::transcode(chars));
      }

      if(this->latencyTagFound)
      {
         this->currVertex->setLatency(atoi(XMLString::transcode(chars)));
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

void GraphMLReader::startElement(const XMLCh * const uri, const XMLCh * const localname, const XMLCh * const qname, const Attributes &attrs)
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

      if (key == "latency")
      {
          this->latencyTagFound = true;
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

Graph& GraphMLReader::readGraph(const char *path)
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
