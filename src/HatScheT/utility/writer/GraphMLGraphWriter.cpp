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

#include "GraphMLGraphWriter.h"
#include <xercesc/util/XMLString.hpp>

#ifdef USE_XERCESC

namespace HatScheT {

GraphMLGraphWriter::GraphMLGraphWriter(std::string path, Graph *g, ResourceModel* rm) : Writer(path) {
  this->g = g;
  this->rm = rm;
}

GraphMLGraphWriter::~GraphMLGraphWriter() {

}

void GraphMLGraphWriter::write() {
  // Initilize Xerces.
  XMLPlatformUtils::Initialize();

  // Pointer to our DOMImplementation.
  DOMImplementation * pDOMImplementation = NULL;

  // Get the DOM Implementation (used for creating DOMDocuments).
  pDOMImplementation =
          DOMImplementationRegistry::getDOMImplementation(XMLString::transcode("core"));

  // Pointer to a DOMDocument.
  DOMDocument* pDOMDocument = NULL;

  // Root Element graphml
  pDOMDocument = pDOMImplementation->createDocument(0, XMLString::transcode("graphml"), 0);

  DOMElement * pRootElement = NULL;
  pRootElement = pDOMDocument->getDocumentElement();

  // Create an Element graph, then fill the with edges and nodes
  // and then append this to the root element.
  DOMElement * graphElement = NULL;
  graphElement = pDOMDocument->createElement(XMLString::transcode("graph"));
  graphElement->setAttribute(XMLString::transcode("id"), XMLString::transcode(this->g->getName().c_str()));
  graphElement->setAttribute(XMLString::transcode("edgedefault"), XMLString::transcode("directed"));

  // Nodes of the graph
  for(auto it = this->g->verticesBegin(); it != this->g->verticesEnd(); ++it){
    Vertex* v = *it;
    const Resource* r = this->rm->getResource(v);

    // Create an Element node, then fill in attributes,
    // and then append this to the root element.
    DOMElement * pDataElement = NULL;
    pDataElement = pDOMDocument->createElement(XMLString::transcode("node"));
    pDataElement->setAttribute(XMLString::transcode("id"), XMLString::transcode(to_string(v->getId()).c_str()));

    // Create an Element date, then fill in name attributes,
    // and then append this to the node element.
    DOMElement * nameElement = NULL;
    nameElement = pDOMDocument->createElement(XMLString::transcode("data"));
    nameElement->setAttribute(XMLString::transcode("key"), XMLString::transcode("name"));
    nameElement->setTextContent(XMLString::transcode(v->getName().c_str()));
    pDataElement->appendChild(nameElement);

    // Create an Element date, then fill in name attributes,
    // and then append this to the node element.
    DOMElement * resElement = NULL;
    resElement = pDOMDocument->createElement(XMLString::transcode("data"));
    resElement->setAttribute(XMLString::transcode("key"), XMLString::transcode("uses_resource"));
    resElement->setTextContent(XMLString::transcode(r->getName().c_str()));
    pDataElement->appendChild(resElement);

    graphElement->appendChild(pDataElement);
  }

  //edges
  for(auto it = this->g->edgesBegin(); it != this->g->edgesEnd(); ++it){
    Edge* e = *it;
    Vertex* src = &e->getVertexSrc();
    Vertex* dst = &e->getVertexDst();

    string edgeId = to_string(src->getId()) + "_" + to_string(dst->getId());

    // Create an Element edge, then fill in attributes,
    // and then append this to the root element.
    DOMElement * pDataElement = NULL;
    pDataElement = pDOMDocument->createElement(XMLString::transcode("edge"));
    pDataElement->setAttribute(XMLString::transcode("id"), XMLString::transcode(edgeId.c_str()));
    pDataElement->setAttribute(XMLString::transcode("source"), XMLString::transcode(to_string(src->getId()).c_str()));
    pDataElement->setAttribute(XMLString::transcode("target"), XMLString::transcode(to_string(dst->getId()).c_str()));

    // Create an Element date, then fill in distance attributes,
    // and then append this to the node element.
    DOMElement * dstElement = NULL;
    dstElement = pDOMDocument->createElement(XMLString::transcode("data"));
    dstElement->setAttribute(XMLString::transcode("key"), XMLString::transcode("distance"));
    dstElement->setTextContent(XMLString::transcode(to_string(e->getDistance()).c_str()));
    pDataElement->appendChild(dstElement);

    // Create an Element date, then fill in delay attributes,
    // and then append this to the node element.
    DOMElement * delayElement = NULL;
    delayElement = pDOMDocument->createElement(XMLString::transcode("data"));
    delayElement->setAttribute(XMLString::transcode("key"), XMLString::transcode("delay"));
    delayElement->setTextContent(XMLString::transcode(to_string(e->getDelay()).c_str()));
    pDataElement->appendChild(delayElement);

    //omit the backward attribute of edges, it not needed (patrick)

    graphElement->appendChild(pDataElement);
  }

  pRootElement->appendChild(graphElement);

  //path for xml writing
  DoOutput2File(pDOMDocument, XMLString::transcode(this->path.c_str()));
}

void GraphMLGraphWriter::DoOutput2File(DOMDocument *pmyDOMDocument, XMLCh *FullFilePath) {
  DOMImplementation *pImplement = NULL;
  DOMLSSerializer *pSerializer = NULL;
  XMLFormatTarget *pTarget = NULL;

  /*
  Return the first registered implementation that
  has the desired features. In this case, we are after
  a DOM implementation that has the LS feature... or Load/Save.
  */
  pImplement = DOMImplementationRegistry::getDOMImplementation(XMLString::transcode("LS"));

  /*
  From the DOMImplementation, create a DOMWriter.
  DOMWriters are used to serialize a DOM tree [back] into an XML document.
  */
  pSerializer = ((DOMImplementationLS *) pImplement)->createLSSerializer();


  /*
  This line is optional. It just sets a feature
  of the Serializer to make the output
  more human-readable by inserting line-feeds,
  without actually inserting any new elements/nodes
  into the DOM tree. (There are many different features to set.)
  Comment it out and see the difference.
  */
  DOMConfiguration *pDomConfiguration = pSerializer->getDomConfig();
  pDomConfiguration->setParameter(XMLUni::fgDOMWRTFormatPrettyPrint, true);


  /*
  Choose a location for the serialized output. The 3 options are:
      1) StdOutFormatTarget     (std output stream -  good for debugging)
      2) MemBufFormatTarget     (to Memory)
      3) LocalFileFormatTarget  (save to file)
      (Note: You'll need a different header file for each one)
      Don't forget to escape file-paths with a backslash character, or
      just specify a file to be saved in the exe directory.
  */
  pTarget = new LocalFileFormatTarget(FullFilePath);
  // Write the serialized output to the target.
  DOMLSOutput *pDomLsOutput = ((DOMImplementationLS *) pImplement)->createLSOutput();
  pDomLsOutput->setByteStream(pTarget);

  pSerializer->write(pmyDOMDocument, pDomLsOutput);

  //Cleanup
  pSerializer->release();
  XMLString::release(&FullFilePath);
  delete pTarget;
  pDomLsOutput->release();
}
}
#endif