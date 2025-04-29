#include "Vector3.h"
#include <iostream>
#include <math.h>

namespace aut4
{

Vector3::Vector3()
{
    _v.resize(3);
    _v[0] = 0.0;
    _v[1] = 0.0;
    _v[2] = 0.0;
}

Vector3::Vector3(double x, double y, double z)
{
    _v.resize(3);
    _v[0] = x;
    _v[1] = y;
    _v[2] = z;
}

Vector3::~Vector3()
{
    
}

double &Vector3::operator[](unsigned int index)
{
    if(index>=_v.size())
    {
        std::cout << "Array index out of bound, exiting." << std::endl;
        exit(0);
    }
    return _v[index];
}

Vector3 &Vector3::operator*=(const double val)
{
    _v[0] *= val;
    _v[1] *= val;
    _v[2] *= val;
    return *this;
}

Vector3 &Vector3::operator+=(const Vector3 &v)
{
    _v[0] += v._v[0];
    _v[1] += v._v[1];
    _v[2] += v._v[2];
    return *this;
}

Vector3 &Vector3::operator-=(const Vector3 &v)
{
    _v[0] -= v._v[0];
    _v[1] -= v._v[1];
    _v[2] -= v._v[2];
    return *this;
}

Vector3 &Vector3::operator=(const Vector3 &v)
{
    _v[0] = v._v[0];
    _v[1] = v._v[1];
    _v[2] = v._v[2];
    return *this;
}

double Vector3::norm()
{
    return sqrt(_v[0]*_v[0] + _v[1]*_v[1] + _v[2]*_v[2]);
}

void Vector3::normalize()
{
    double n = norm();
    if(n>1e-9)
    {
        _v[0] /= n;
        _v[1] /= n;
        _v[2] /= n;
    }
}

double Vector3::dot(Vector3 &v2)
{
    return (_v[0]*v2[0] + _v[1]*v2[1] + _v[2]*v2[2]);
}

Vector3 Vector3::cross(Vector3 &v2)
{
    Vector3 vRes(_v[1]*v2[2] - _v[2]*v2[1],
                 _v[2]*v2[0] - _v[0]*v2[2],
                 _v[0]*v2[1] - _v[1]*v2[0]);
    return vRes;
}

void Vector3::axisAngleFromTwoVectors(Vector3 v1, Vector3 v2, Vector3 &axis, double &angle)
{
    double dotV = v1.dot(v2);
    if(dotV < -0.999999)
    {
        Vector3 xUnit(1.0, 0.0, 0.0);
        Vector3 yUnit(0.0, 1.0, 0.0);
        Vector3 crossV = xUnit.cross(v1);

        if(crossV.norm() < 1e-6)
        {
            crossV = yUnit.cross(v1);
        }

        angle = M_PI;
        axis[0] = crossV[0];
        axis[1] = crossV[1];
        axis[2] = crossV[2];
    }
    else if (dotV > 0.999999)
    {
        angle = 0.0;
        axis[0] = 0.0;
        axis[1] = 0.0;
        axis[2] = 0.0;
    }
    else
    {
        Vector3 crossV = v1.cross(v2);
        angle = acos(dotV);
        axis[0] = crossV[0];
        axis[1] = crossV[1];
        axis[2] = crossV[2];
    }
    axis.normalize();
    return;
}

void Vector3::print()
{
    std::cout << _v[0] << " " << _v[1] << " " << _v[2] << std::endl;
}

}