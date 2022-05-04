#pragma once
#include "MaterialModel.h"

#ifndef CONTROLLER_H
#define CONTROLLER_H

// Abstract base controller class
class Controller
{
protected:
	double _minE, _maxE, _minF, _maxF;

public:
	Controller() 
		: _minE(-10), _maxE(10), _minF(0), _maxF(5) {}
	Controller(double minE, double maxE, double minF, double maxF)
		: _minE(minE), _maxE(maxE), _minF(minF), _maxF(maxF) {}

	virtual void nextPath(Path& nextPath, Path prevPath, double errWd, double errCl) = 0;
	void setAugerLimits(double minE, double maxE) { _minE = minE; _maxE = maxE; }
	void setFeedLimits(double minF, double maxF) { _minF = minF; _maxE = maxF; }

};

// Auger controller class
class PController : public Controller
{
public:
	PController()
		:_kp(1) {}
	PController(MaterialModel model)
		: _model(model), _kp(1) {}
	PController(MaterialModel model, double kp)
		: _model(model), _kp(kp) {}

	void nextPath(Path& nextPath, Path prevPath, double errWd, double errCl)
	{
		double prevWidth = _model.width(prevPath.e, prevPath.f);
		double nextInput;
		if (!isnan(errWd)) {
			switch (_model.type())
			{
			case MaterialModel::AUGER:
				nextInput = _model.output(prevWidth + _kp * errWd, prevPath.f);
				// check if output is saturated
				if (nextInput < _minE) { nextInput = _minE; }
				if (nextInput > _maxE) { nextInput = _maxE; }
				nextPath.e = nextInput;
				break;
			case MaterialModel::VELOCITY:
				nextInput = _model.output(prevWidth + _kp * errWd, prevPath.e);
				// check if output is saturated
				if (nextInput < _minF) { nextInput = _minF; }
				if (nextInput > _maxF) { nextInput = _maxF; }
				nextPath.f = nextInput;
				break;
			}
		}
	}

	void kp(double kp)
	{
		_kp = kp;
	}

private:
	MaterialModel _model;
	double _kp;
};

#endif // !CONTROLLER_H