#include <Quat.h>
#include <Matrix.h>
#include <stdio.h>
#include <math.h>

using namespace std; // for printValues() using cout

// default constructor      
Quat::Quat()
{
    // by default initialize to identity quaternion
    m_data[0] = 0.0;
    m_data[1] = 0.0;
    m_data[2] = 0.0;
    m_data[3] = 1.0;
}

// constructor with quaternion element values specified
Quat::Quat(double q0, double q1, double q2, double q3)
{
    // initialize quaternion to input values
    m_data[0] = q0;
    m_data[1] = q1;
    m_data[2] = q2;
    m_data[3] = q3;
}

// constructor with vector/angle input arguments
Quat::Quat(Matrix<double,3,1> vector, double thetaRad)
{
    // normalize vector
    vector = vector/(vector.norm());
    
    // get sin(theta/2) and cos(theta/2) multipliers
    double sinHalfTheta = sin(thetaRad/2);
    double cosHalfTheta = cos(thetaRad/2);
    
    // populate Quaternion elements
    m_data[0] = vector(0)*sinHalfTheta;
    m_data[1] = vector(1)*sinHalfTheta;
    m_data[2] = vector(2)*sinHalfTheta;
    m_data[3] = cosHalfTheta;
    
}

// copy constructor
Quat::Quat(const Quat& inputQuat)
{
    // initialize quaternion element values with input quaternion element values
    m_data[0] = inputQuat.m_data[0];
    m_data[1] = inputQuat.m_data[1];
    m_data[2] = inputQuat.m_data[2];
    m_data[3] = inputQuat.m_data[3];
}

// destructor (empty)
Quat::~Quat()
{
}

// multiplication by constant
Quat Quat::operator*(double factor) const
{
    return Quat(m_data[0]*factor, \
            m_data[1]*factor, \
            m_data[2]*factor, \
            m_data[3]*factor);
}

// division by constant
Quat Quat::operator/(double divisor) const
{
    return Quat(m_data[0]/divisor, \
            m_data[1]/divisor, \
            m_data[2]/divisor, \
            m_data[3]/divisor);
}

// addition of constant
Quat Quat::operator+(double bias) const
{
    return Quat(m_data[0]+bias, \
            m_data[1]+bias, \
            m_data[2]+bias, \
            m_data[3]+bias);
}

// addition of quaternions
Quat Quat::operator+(const Quat& inQuat) const
{
    return Quat(m_data[0] + inQuat(0), \
            m_data[1] + inQuat(1), \
            m_data[2] + inQuat(2), \
            m_data[3] + inQuat(3));
}

// subtraction of constant
Quat Quat::operator-(double bias) const
{
    return Quat(m_data[0]-bias, \
            m_data[1]-bias, \
            m_data[2]-bias, \
            m_data[3]-bias);
}

// quaternion element access by reference
double& Quat::operator()(int index)
{
    return m_data[index];
}

// quaternion element access by reference
double Quat::operator()(int index) const
{
    return m_data[index];
}

// print quaternion values
void Quat::printValues()
{
    printf("\n");
    for (int ii = 0; ii < 4; ii++)
    {
        printf("%0.16f",m_data[ii]);
    }
    printf("\n");
}

// get magnitude (2-norm) of quaternion
double Quat::norm()
{
    return sqrt(m_data[0]*m_data[0] + \
            m_data[1]*m_data[1] + \
            m_data[2]*m_data[2] + \
            m_data[3]*m_data[3]);
}

// return normalized version of quaternion
Quat Quat::normalize() const
{
    return (*this)/this->norm();
}

// return inverse quaternion (negate last element)
Quat Quat::inverse() const
{
    return Quat(m_data[0], \
            m_data[1], \
            m_data[2], \
            -m_data[3]);
}

double Quat::dot(const Quat& q) const {
    double z = 0;
    for (char i = 0; i < 4; i++) {
        z += m_data[i] * q.m_data[i]; 
    }
}

// quaternion multiplication on the right
Quat Quat::operator*(const Quat& rightQuat) const
{
    Quat quatOut;
    const Quat& leftQuat = *this;

    quatOut(0) =  rightQuat(3)*leftQuat(0) + rightQuat(2)*leftQuat(1) - rightQuat(1)*leftQuat(2) + rightQuat(0)*leftQuat(3);
    quatOut(1) = -rightQuat(2)*leftQuat(0) + rightQuat(3)*leftQuat(1) + rightQuat(0)*leftQuat(2) + rightQuat(1)*leftQuat(3);
    quatOut(2) =  rightQuat(1)*leftQuat(0) - rightQuat(0)*leftQuat(1) + rightQuat(3)*leftQuat(2) + rightQuat(2)*leftQuat(3);
    quatOut(3) = -rightQuat(0)*leftQuat(0) - rightQuat(1)*leftQuat(1) - rightQuat(2)*leftQuat(2) + rightQuat(3)*leftQuat(3);
    
    return quatOut;
}

// quaternion division on the right (right-multiply by inverse of factor)
Quat Quat::operator/(const Quat& quatDivisor) const
{
    return (*this)*(quatDivisor.inverse());
}

Quat Quat::getQuatDerivative(const Matrix<double,3,1> & rateRadSec) {
    
    // quaternion derivative is half the product of the "rate" quaternion and the quaternion being differentiated
    return (*this)*Quat(rateRadSec(0),rateRadSec(1),rateRadSec(2),0.0)*0.5;

}

Matrix<double,3,1> Quat::rotateCsys(Matrix<double,3,1> v)
{
    double qi = m_data[0];
    double qj = m_data[1];
    double qk = m_data[2];
    double qr = m_data[3];

    Matrix<double,3,3> dcm;
    dcm(0,0) = 1 - 2*qj*qj - 2*qk*qk;
    dcm(0,1) = 2*(qi*qj - qk*qr);
    dcm(0,2) = 2*(qi*qk + qj*qr);

    dcm(1,0) = 2*(qi*qj + qk*qr);
    dcm(1,1) = 1 - 2*qi*qi - 2*qk*qk;
    dcm(1,2) = 2*(qj*qk - qi*qr);

    dcm(2,0) = 2*(qi*qk - qj*qr);
    dcm(2,1) = 2*(qj*qk + qi*qr);
    dcm(2,2) = 1 - 2*qi*qi - 2*qj*qj;

    return (dcm * v);
}


Quat slerp(const Quat &q0, const Quat &q1, double t) const {
    // assumed unit quaternion.
    double dot = q0.dot(q1);
    if (dot < 0) { //angle is greater than 90. Avoid "long path" rotations.
        dot = -dot;
        q1 = -q1;
    }
    dot = std::max(0, std::min(1, dot)); //keep <dot> in the domain of acos.
    double omega = std::acos(dot);
    double theta = omega * t;
    Quat q2 = (q1 - q0 * dot).normalize();
    return std::cos(theta) * q0 + std::sin(theta) * q2;
}

