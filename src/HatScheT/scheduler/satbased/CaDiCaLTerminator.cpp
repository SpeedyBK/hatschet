//
// Created by nfiege on 5/20/22.
//

#include "CaDiCaLTerminator.h"

namespace HatScheT {
	CaDiCaLTerminator::CaDiCaLTerminator(double timeout)
		: maxTime(timeout), timerStart(std::chrono::steady_clock::now()) {}

	bool CaDiCaLTerminator::terminate() {
		return this->getElapsedTime() >= this->maxTime;
	}

	void CaDiCaLTerminator::reset(double newTimeout) {
		this->timerStart = std::chrono::steady_clock::now();
		this->maxTime = newTimeout;
	}

	double CaDiCaLTerminator::getElapsedTime() const {
		return static_cast<double>(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - this->timerStart).count()) / 1000.0;
	}
}