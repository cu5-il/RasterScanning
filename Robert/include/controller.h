#pragma once
#include "MaterialModel.h"

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
	AugerController()
		:_kp(1) {}
	AugerController(MaterialModel augerModel)
		: _augerModel(augerModel), _kp(1) {}
	AugerController(MaterialModel augerModel, double kp)
		: _augerModel(augerModel), _kp(kp) {}
	AugerController(MaterialModel augerModel, double minE, double maxE)
		: _augerModel(augerModel), _kp(1) { Controller:setAugerLimits(minE, maxE);}
	AugerController(MaterialModel augerModel,double kp, double minE, double maxE)
		: _augerModel(augerModel), _kp(kp) {Controller:setAugerLimits(minE, maxE);}

	void nextPath(Path& nextPath, Path prevPath, double errWd, double errCl)
	{
		double prevWidth = _augerModel.width(prevPath.e, prevPath.f);
		double nextE;
		if (!isnan(errWd)) {
			nextE = _augerModel.output(prevWidth + _kp* errWd, prevPath.f);
			// check if output is saturated
			if (nextE < _minE) { nextE = _minE; }
			if (nextE > _maxE) { nextE = _maxE; }
			nextPath.e = nextE;
		}
	}

	void kp(double kp)
	{
		_kp = kp;
	}

private:
	MaterialModel _augerModel;
	double _kp;
};

#endif // !CONTROLLER_H