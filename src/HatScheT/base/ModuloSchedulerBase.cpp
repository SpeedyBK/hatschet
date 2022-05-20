/*
    This file is part of the HatScheT project, developed at University of Kassel and TU Darmstadt, Germany
    Author: Martin Kumm, Patrick Sittel ({kumm, sittel}@uni-kassel.de)
    Author: Julian Oppermann (oppermann@esa.tu-darmstadt.de)

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

#include <HatScheT/base/ModuloSchedulerBase.h>
#include <HatScheT/utility/Utility.h>

namespace HatScheT
{

ModuloSchedulerBase::ModuloSchedulerBase() :
	firstObjectiveOptimal(false), secondObjectiveOptimal(false) {

}

void ModuloSchedulerBase::computeMinII(Graph *g, ResourceModel *rm, Target* t) {
  this->resMinII = Utility::calcResMII(rm,t);
  this->recMinII = Utility::calcRecMII(g,rm);
  this->minII = Utility::calcMinII(this->resMinII,this->recMinII);
}

void ModuloSchedulerBase::computeMaxII(Graph *g, ResourceModel *rm) {
  this->maxII = Utility::calcMaxII(g, rm);
}

std::pair<bool, bool> ModuloSchedulerBase::getObjectivesOptimal() const {
	return std::make_pair(this->firstObjectiveOptimal, this->secondObjectiveOptimal);
}

}
