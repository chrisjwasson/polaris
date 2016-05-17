#ifndef QUAT_H
#define QUAT_H

#include "Matrix.h"

class Quat {

	public:

		Quat();
		Quat(double q0, double q1, double q2, double q3);
		Quat(Matrix<double,3,1> vector, double thetaRad);
		Quat(const Quat& inputQuat);
		~Quat();
		Quat operator*(double factor) const;
		Quat operator/(double divisor) const;
		Quat operator+(double bias) const;
		Quat operator+(const Quat& inQuat) const;
		Quat operator-(double bias) const;
		double& operator()(int index);
		double operator()(int index) const;
		void printValues();
		double norm();
		Quat normalize();
		Quat inverse() const;
		Quat operator*(const Quat& rightQuat) const;
		Quat operator/(const Quat& quatDivisor) const;
		Quat getQuatDerivative(const Matrix<double,3,1> & rateRadSec);
        Matrix<double,3,1> rotateCsys(Matrix<double,3,1>);

	private:

		double m_data[4];

};

#endif
