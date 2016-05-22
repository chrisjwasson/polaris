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
		Quat operator-(const Quat& q) const;
		Quat operator-() const;
		double& operator()(int index);
		double operator()(int index) const;
		void printValues() const;
		double norm() const;
		Quat normalize() const;
		Quat inverse() const;
		double dot(const Quat& q) const;
		Quat operator*(const Quat& rightQuat) const;
		Quat operator/(const Quat& quatDivisor) const;
		Quat getQuatDerivative(const Matrix<double,3,1> & rateRadSec);
        Matrix<double,3,1> rotateCsys(Matrix<double,3,1>);

	private:

		double m_data[4];

};

inline Quat operator*(double f, const Quat& q2) { return q2 * f; }

Quat slerp(const Quat& q0, Quat q1, double t);

#endif
