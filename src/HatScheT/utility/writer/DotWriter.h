#pragma once
#include "Writer.h"
#include "../../ResourceModel.h"
#include "../../Graph.h"

namespace HatScheT
{
/*!
 * \brief The DotWriter class use this class to write your graphs to the .dot format
 */
class DotWriter : public Writer
{
public:
  /*!
   * \brief DotWriter
   * \param path provide a filename (witout .dot!)
   * \param g
   * \param rm
   */
  DotWriter(std::string path, Graph* g, ResourceModel* rm);
  ~DotWriter();
  /*!
   * \brief setDisplayNames default is false
   * \param b
   */
  void setDisplayNames(bool b){this->displayNames = b;}
  /*!
   * \brief write writes to .dot
   * supported 7 different resources/colours
   */
  virtual void write();

protected:
  bool displayNames;
  /*!
   * \brief g save pointer to graph to write
   */
  Graph* g;
  /*!
   * \brief rm save pointer to used resource model
   */
  ResourceModel* rm;
};

}

