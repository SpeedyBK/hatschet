//
// Created by nfiege on 2/15/23.
//

#include "ILPScheduleLengthSweeper.h"
#include <chrono>
#include <cmath>
#include <fstream>

namespace HatScheT {
	ILPScheduleLengthSweeper::ILPScheduleLengthSweeper(Graph &graph, ResourceModel &resourceModel,
		const std::list<std::string> &solverWishlist, const string &resultFilePath) :
		IterativeModuloSchedulerLayer(graph, resourceModel), sw(solverWishlist), resultFilePath(resultFilePath),
		estimator(&graph, &resourceModel, solverWishlist) {}

	void HatScheT::ILPScheduleLengthSweeper::scheduleIteration() {
		std::cerr << "ILPScheduleLengthSweeper::scheduleIteration: candidate II = " << this->II << std::endl;
		// this doesn't find a schedule and is only used for logging purposes :)
		this->scheduleFound = false;

		// do SL estimation
		this->estimator.setQuiet(this->quiet);
		auto timerStart = std::chrono::steady_clock::now();
		this->estimator.estimateMinSL((int)this->II, (int)this->solverTimeout);
		auto elapsedTimeSec = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - timerStart).count() / 1000.0;
		int minSLEstimation = -1;
		int sdcSLEstimation = -1;
		if (this->estimator.minSLEstimationFound()) {
			minSLEstimation = this->estimator.getMinSLEstimation();
			sdcSLEstimation = this->estimator.getSDCScheduleLength();
		}

		// append logfile with results
		std::fstream f;
		if (this->II == std::ceil(this->minII)) {
			// write file header before writing first result line
			f.open(this->resultFilePath, std::ios_base::out);
			f << "II;minSL;sdcSL;timeInSec" << std::endl;
			f.close();
		}
		// write result to file
		f.open(this->resultFilePath, std::ios_base::app);
		f << (int)this->II << ";" << minSLEstimation << ";" << sdcSLEstimation << ";" << elapsedTimeSec << std::endl;
		f.close();
	}
}