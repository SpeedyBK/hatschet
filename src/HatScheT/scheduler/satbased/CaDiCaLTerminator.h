//
// Created by nfiege on 5/20/22.
//

#ifndef HATSCHET_CADICALTERMINATOR_H
#define HATSCHET_CADICALTERMINATOR_H

#ifdef USE_CADICAL

#include <cadical.hpp>
#include <chrono>

namespace HatScheT {
	// Use this class to track time and add a timeout to CaDiCaL solver
	class CaDiCaLTerminator : public CaDiCaL::Terminator {
	public:
		explicit CaDiCaLTerminator(double timeout);
		bool terminate () override;
		void reset(double newTimeout);
		double getElapsedTime() const;
	private:
		double maxTime;
		std::chrono::steady_clock::time_point timerStart;
	};
}

#endif

#endif //HATSCHET_CADICALTERMINATOR_H
