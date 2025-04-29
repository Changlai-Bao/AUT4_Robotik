#include "Quaternion.h"
#include <iostream>
#include <math.h>

namespace aut4
{

    Quaternion::Quaternion()
    {
        w = 1.0;
        x = 0.0;
        y = 0.0;
        z = 0.0;
    }

    Quaternion::Quaternion(double qw, double qx, double qy, double qz)
    {
        w = qw;
        x = qx;
        y = qy;
        z = qz;
    }

    Quaternion::~Quaternion()
    {
    }

    Quaternion Quaternion::conj()
    {
        Quaternion qNew;
        qNew.w = w;
        qNew.x = -x;
        qNew.y = -y;
        qNew.z = -z;
        return qNew;
    }

    Quaternion &Quaternion::operator*=(const Quaternion &q)
    {
        double wTemp = w*q.w - x*q.x - y*q.y - z*q.z;
        double xTemp = y*q.z - z*q.y + w*q.x + x*q.w;
        double yTemp = z*q.x - x*q.z + w*q.y + y*q.w;
        double zTemp = x*q.y - y*q.x + w*q.z + z*q.w;
        w = wTemp;
        x = xTemp;
        y = yTemp;
        z = zTemp;
        return *this;
    }

    double Quaternion::norm()
    {
        return sqrt(w*w + x*x + y*y + z*z);
    }

    Quaternion &Quaternion::normalize()
    {
        double n = norm();
        if (n > 1e-9)
        {
            w /= n;
            x /= n;
            y /= n;
            z /= n;
        }
        return *this;
    }

    void Quaternion::rotate(Vector3 &v)
    {
        Quaternion qV(0.0, v[0], v[1], v[2]);
        Quaternion qRes = (*this) * qV * this->conj();
        v[0] = qRes.x;
        v[1] = qRes.y;
        v[2] = qRes.z;
    }

    Quaternion Quaternion::quaternionFromTwoVectors(Vector3 v1, Vector3 v2)
    {
        Quaternion q;
        Vector3 vAxis;
        double angle;
        Vector3::axisAngleFromTwoVectors(v1, v2, vAxis, angle);
        q.w = cos(angle/2.0);
        q.x = vAxis[0] * sin(angle/2.0);
        q.y = vAxis[1] * sin(angle/2.0);
        q.z = vAxis[2] * sin(angle/2.0);
        return q;
    }

    Quaternion Quaternion::quaternionFromAngularVelocity(Vector3 omega, double dt)
    {
        double thetaHalf = omega.norm() * dt / 2.0;
        double sineHalf  = sin(thetaHalf);
        omega.normalize();
        Quaternion q(cos(thetaHalf), omega[0] * sineHalf, omega[1] * sineHalf, omega[2] * sineHalf);
        return q;
    }

    Quaternion Quaternion::quaternionFromAxisAngle(Vector3 axis, double theta)
    {
        double sineHalf = sin(theta / 2.0);
        Quaternion q(cos(theta / 2.0), axis[0] * sineHalf, axis[1] * sineHalf, axis[2] * sineHalf);
        return q;
    }

    Quaternion Quaternion::slerp(Quaternion qTarget, double t)
    {
        Quaternion qSlerp;

        double cosHalfTheta = w * qTarget.w + x * qTarget.x + y * qTarget.y + z * qTarget.z;

        if (abs(cosHalfTheta) >= 1.0)
        {
            qSlerp = *this;
            return qSlerp;
        }
        // Calculate temporary values.
        double halfTheta = acos(cosHalfTheta);
        double sinHalfTheta = sqrt(1.0 - cosHalfTheta * cosHalfTheta);
        // if theta = 180 degrees then result is not fully defined
        // we could rotate around any axis normal to qa or qb
        if (abs(sinHalfTheta) < 0.001)
        {
            qSlerp.w = (w * 0.5 + qTarget.w * 0.5);
            qSlerp.x = (x * 0.5 + qTarget.x * 0.5);
            qSlerp.y = (y * 0.5 + qTarget.y * 0.5);
            qSlerp.z = (z * 0.5 + qTarget.z * 0.5);
            return qSlerp;
        }
        double ratioA = sin((1 - t) * halfTheta) / sinHalfTheta;
        double ratioB = sin(t * halfTheta) / sinHalfTheta;
        //calculate Quaternion.
        qSlerp.w = (w * ratioA + qTarget.w * ratioB);
        qSlerp.x = (x * ratioA + qTarget.x * ratioB);
        qSlerp.y = (y * ratioA + qTarget.y * ratioB);
        qSlerp.z = (z * ratioA + qTarget.z * ratioB);

        return qSlerp;
    }

    Quaternion Quaternion::lerp(Quaternion qTarget, double t)
    {
        Quaternion qLerp;
        qLerp.w = w * (1 - t) + qTarget.w * t;
        qLerp.x = x * (1 - t) + qTarget.x * t;
        qLerp.y = y * (1 - t) + qTarget.y * t;
        qLerp.z = z * (1 - t) + qTarget.z * t;
        qLerp.normalize();
        return qLerp;
    }

    void Quaternion::print()
    {
        std::cout << "(" << w << " " << x << " " << y << " " << z << ")" << std::endl;
    }

    Quaternion operator*(const Quaternion &lhs, const Quaternion &rhs)
    {
        Quaternion qRes;
        qRes.w = lhs.w*rhs.w - lhs.x*rhs.x - lhs.y*rhs.y - lhs.z*rhs.z;
        qRes.x = lhs.y*rhs.z - lhs.z*rhs.y + lhs.w*rhs.x + lhs.x*rhs.w;
        qRes.y = lhs.z*rhs.x - lhs.x*rhs.z + lhs.w*rhs.y + lhs.y*rhs.w;
        qRes.z = lhs.x*rhs.y - lhs.y*rhs.x + lhs.w*rhs.z + lhs.z*rhs.w;
        return qRes;
    }

}
