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

#include "XMLResourceWriter.h"
#include <xercesc/util/XMLString.hpp>
#include <cmath>
#include <cfenv>

namespace HatScheT {

XMLResourceWriter::XMLResourceWriter(std::string path, ResourceModel* rm) : Writer(path) {
  this->rm = rm;
}

XMLResourceWriter::~XMLResourceWriter() {
}

void XMLResourceWriter::write() {
  // Initilize Xerces.
  XMLPlatformUtils::Initialize();

  // Pointer to our DOMImplementation.
  DOMImplementation *pDOMImplementation = NULL;

  // Get the DOM Implementation (used for creating DOMDocuments).
  pDOMImplementation =
          DOMImplementationRegistry::getDOMImplementation(XMLString::transcode("core"));

  // Pointer to a DOMDocument.
  DOMDocument *pDOMDocument = NULL;

  // Root Element graphml
  pDOMDocument = pDOMImplementation->createDocument(0, XMLString::transcode("ModelInformation"), 0);

  DOMElement *pRootElement = NULL;
  pRootElement = pDOMDocument->getDocumentElement();

  // Create an Element graph, then fill the with edges and nodes
  // and then append this to the root element.
  DOMElement *resourcesElement = NULL;
  resourcesElement = pDOMDocument->createElement(XMLString::transcode("Resources"));

  for(auto it = this->rm->resourcesBegin(); it != this->rm->resourcesEnd(); ++it){
    Resource* r = *it;

    // Create an Element resource, then fill in attributes,
    // and then append this to the root element.
    DOMElement * pDataElement = NULL;
    pDataElement = pDOMDocument->createElement(XMLString::transcode("Resource"));
    pDataElement->setAttribute(XMLString::transcode("name"), XMLString::transcode(r->getName().c_str()));
    pDataElement->setAttribute(XMLString::transcode("limit"), XMLString::transcode(to_string(r->getLimit()).c_str()));
    pDataElement->setAttribute(XMLString::transcode("latency"), XMLString::transcode(to_string(r->getLatency()).c_str()));
    pDataElement->setAttribute(XMLString::transcode("blockingTime"), XMLString::transcode(to_string(r->getBlockingTime()).c_str()));
    pDataElement->setAttribute(XMLString::transcode("physicalDelay"), XMLString::transcode(to_string(r->getPhysicalDelay()).c_str()));

    //get resources cost
    map<string,double>& costs = r->getHardwareCosts();

    for(auto it = costs.begin(); it != costs.end(); ++it){
      string name = it->first;
      double amount = it->second;

      //resource cost element
      DOMElement * costElement = NULL;
      costElement = pDOMDocument->createElement(XMLString::transcode("Cost"));
      costElement->setAttribute(XMLString::transcode("name"), XMLString::transcode(name.c_str()));

      //check if demand is a real flost
      if(std::fmod(amount, 1.0) == 0.0){
        costElement->setAttribute(XMLString::transcode("demand"), XMLString::transcode(to_string((int)amount).c_str()));
      } else
      costElement->setAttribute(XMLString::transcode("demand"), XMLString::transcode(to_string(amount).c_str()));

      pDataElement->appendChild(costElement);
    }


    resourcesElement->appendChild(pDataElement);
  }

  pRootElement->appendChild(resourcesElement);

  //path for xml writing
  DoOutput2File(pDOMDocument, XMLString::transcode(this->path.c_str()));
}

void XMLResourceWriter::DoOutput2File(DOMDocument *pmyDOMDocument, XMLCh *FullFilePath) {
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