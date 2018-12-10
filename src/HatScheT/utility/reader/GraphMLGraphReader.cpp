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
  this->edgeDelayTagFound = false;
  this->dataTagFound = false;
  this->resourceTagFound = false;
  this->edgeDistanceTagFound = false;
  this->deptypeTagFound = false;

  this->currVertexId = -1;
  this->dstId = -1;
  this->srcId = -1;
  this->edgeDelay = 0;
  this->edgeDistance = 0;
  this->deptype = "";
}

GraphMLGraphReader::~GraphMLGraphReader()
{
}

void GraphMLGraphReader::endElement(const XMLCh * const uri, const XMLCh * const localname, const XMLCh * const qname)
{
  string name = XMLString::transcode(localname);

  if(name == "node" && this->nodeTagFound == true) {
    Vertex* v = &this->g->createVertex(this->currVertexId);
    v->setName(this->currVertexName);
    Resource* r = this->rm->getResource(this->currVertexResName);

    this->rm->registerVertex(v, r);

    this->nodeTagFound = false;
  }

  if(name == "edge" && this->edgeTagFound == true) {
    Vertex& source = this->g->getVertexById(this->srcId);
    Vertex& target = this->g->getVertexById(this->dstId);

    auto &edge = this->g->createEdge(source, target, this->edgeDistance);
    edge.setDelay(this->edgeDelay); // for now, expect it to be present
    if(this->deptype=="Precedence"){
      edge.setDependencyType(Edge::DependencyType::Precedence);
    }

    //reset id container
    this->srcId = -1;
    this->dstId = -1;
    this->edgeDelay = -1;

    this->edgeTagFound = false;
  }

  if(name == "data") {
    this->dataTagFound = false;

    if(this->nameTagFound == true) this->nameTagFound = false;
    if(this->resourceTagFound == true) this->resourceTagFound = false;
    if(this->edgeDelayTagFound == true) this->edgeDelayTagFound = false;
    if(this->edgeDistanceTagFound == true) this->edgeDistanceTagFound = false;
    if(this->deptypeTagFound == true) this->deptypeTagFound = false;
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
        std::string name = XMLString::transcode(chars);
        //remove whitespaces
        std::string::iterator end_pos = std::remove(name.begin(), name.end(), ' ');
        name.erase(end_pos, name.end());

        this->currVertexName = name;
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
      if(this->edgeDelayTagFound == true)
      {
         this->edgeDelay = atoi(XMLString::transcode(chars));
      }
      if(this->edgeDistanceTagFound == true)
      {
        this->edgeDistance = atoi(XMLString::transcode(chars));
      }
      if(this->deptypeTagFound == true)
      {
        this->deptype = XMLString::transcode(chars);
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

    cout << "read ids " << sourcestring << " -> " << targetstring << endl;

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
      if(key == "delay" || key == "latency")
      {
        this->edgeDelayTagFound = true;
      }
      if(key == "distance")
      {
        this->edgeDistanceTagFound = true;
      }
      if(key == "deptype")
      {
        this->deptypeTagFound = true;
      }
    }
  }

}

void GraphMLGraphReader::readGraph(const char *path)
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
      cout << "graphMLReader.parseGraph: File not found! (" << xmlFile << ")" << endl;
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
  catch (const HatScheT::Exception& e){
    cout << e.what() << endl;
    throw HatScheT::Exception(e.what());
  }
  catch (...) {
    cout << "graphMLReader.parseGraph: Unexpected Error" << endl;
    throw HatScheT::Exception("graphMLReader.parseGraph: Unexpected Error");
  }

  delete parser;
  XMLPlatformUtils::Terminate();
}

}

#endif //USE_XERCESC
