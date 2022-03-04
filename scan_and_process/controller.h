#pragma once
#include "myTypes.h"

#ifndef CONTROLLER_H
#define CONTROLLER_H

// Abstract base controller class
class Controller
{
protected:
	double _minE, _maxE, _minF, _maxF;

	void setAugerLimits(double minE, double maxE) { _minE = minE; _maxE = maxE; }
	void setFeedLimits(double minF, double maxF) { _minF = minF; _maxE = maxF; }

public:
	Controller() 
		: _minE(-10), _maxE(10), _minF(0), _maxF(5) {}
	Controller(double minE, double maxE, double minF, double maxF)
		: _minE(minE), _maxE(maxE), _minF(minF), _maxF(maxF) {}

	virtual void nextPath(Path& nextPath, Path prevPath, double errWd, double errCl) = 0;
};

// Auger controller class
class AugerController : public Controller
{
public:
	AugerController() {}
	AugerController(MaterialModel augerModel)
		: _augerModel(augerModel) {}
	AugerController(MaterialModel augerModel, double minE, double maxE)
		: _augerModel(augerModel) { Controller:setAugerLimits(minE, maxE);}

	void nextPath(Path& nextPath, Path prevPath, double errWd, double errCl)
	{
		double prevWidth = _augerModel.width(prevPath.e, prevPath.f);
		double nextE;
		if (!isnan(errWd)) {
			nextE = _augerModel.output(prevWidth + errWd, prevPath.f);
			// check if output is saturated
			if (nextE < _minE) { nextE = _minE; }
			if (nextE > _maxE) { nextE = _maxE; }
			nextPath.e = nextE;
		}
	}
private:
	MaterialModel _augerModel;

};

#endif // !CONTROLLER_H