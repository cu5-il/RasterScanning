#pragma once
#include <iostream>
#include <vector>
#include <cmath>


#ifndef MATERIALMODEL_H
#define MATERIALMODEL_H

///////////////////////////////////////  MaterialModel  ///////////////////////////////////////
class MaterialModel
{
public:
	MaterialModel();
	MaterialModel(char type, std::vector<double> fixedParam, std::vector<double> a, std::vector<double> b, std::vector<double> c);

	double output(double width, double fixedParam);
	double width(double ctrl, double fixedParam);
	const bool& empty() const { return _a.empty(); }
	const char& type() const { return _type; }

	enum types : char
	{
		VELOCITY = 'f', // 
		FEED = 'f', // 
		AUGER = 'a' // 
	};

private:

	std::vector <double> _a, _b, _c, _fixedParam;
	char _type;
	double _interpolate(std::vector<double>& x, std::vector<double>& y, double xQ);

};

inline MaterialModel::MaterialModel()
	: _type(0){}

inline MaterialModel::MaterialModel(char type, std::vector<double> fixedParam, std::vector<double> a, std::vector<double> b, std::vector<double> c)
	: _type(type), _a(a), _b(b), _c(c), _fixedParam(fixedParam) {}

inline double MaterialModel::output(double width, double fixedParam)
{
	double a, b, c;
	a = _interpolate(_fixedParam, _a, fixedParam);
	b = _interpolate(_fixedParam, _b, fixedParam);
	c = _interpolate(_fixedParam, _c, fixedParam);
	return pow(((width - c) / a), 1 / b);
}

inline double MaterialModel::width(double ctrl, double fixedParam)
{
	double a, b, c;
	a = _interpolate(_fixedParam, _a, fixedParam);
	b = _interpolate(_fixedParam, _b, fixedParam);
	c = _interpolate(_fixedParam, _c, fixedParam);
	return a * pow(ctrl, b) + c;
}

inline double MaterialModel::_interpolate(std::vector<double>& xV, std::vector<double>& yV, double xQ)
{
	// make sure vectors are the same size
	if (xV.size() != yV.size()) {
		std::cout << "ERROR: interpolate x.size() != y.size()\n";
		return NAN;
	}

	// find the first value above xQ
	auto it = std::lower_bound(xV.begin(), xV.end(), xQ);

	// get the index of the iterator
	auto j = it - xV.begin();
	if (it == xV.end())
		--j;  // extrapolating above
	else if (*it == xQ)
		return yV[j];

	auto i = j ? j - 1 : 1; //nearest-below index, except when extrapolating downward

	return std::lerp(yV[i], yV[j], (xQ - xV[i]) / (xV[j] - xV[i]));

}

#endif // !MATERIALMODEL_H