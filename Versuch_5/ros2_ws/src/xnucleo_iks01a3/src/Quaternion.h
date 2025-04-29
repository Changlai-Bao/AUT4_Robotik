#ifndef __QUATERNION_H
#define __QUATERNION_H

#include "Vector3.h"

namespace aut4
{

    /**
     * @class Quaternion
     * @brief Quaternion math implementation
     * @author Stefan May
     * @date 15.4.2021
     */
    class Quaternion
    {
    public:
        /**
         * Default constructor, initializes with identity quaternion
         */
        Quaternion();

        /**
         * Element-wise Constructor
         * @param[in] qw real element
         * @param[in] qx imaginary element for i dimension
         * @param[in] qy imaginary element for j dimension
         * @param[in] qz imaginary element for k dimension
         */
        Quaternion(double qw, double qx, double qy, double qz);

        /**
         * Destructor
         */
        ~Quaternion();

        /**
         * Get conjugate of this quaternion
         */
        Quaternion conj();

        /**
         * Multiply-assign operator
         * @param[in] q RHS of multiplication: this = this * q
         */
        Quaternion &operator*=(const Quaternion &q);

        /**
         * Calculate norm
         * @return norm of quaternion
         */
        double norm();

        /**
         * Normalize quaternion to length 1
         * @return normalized quaternion (this instance)
         */
        Quaternion &normalize();

        /**
         * Rotate vector with q v conj(q)
         * @param[in] v vector to be rotated
         */
        void rotate(Vector3 &v);

        /**
         * Create quaternion from two vectors, that rotates one vector to the other.
         * @param[in] v1 first vector (to be rotated to the second one)
         * @param[in] v2 second vector
         * @return resulting unit rotation quaternion to be applied in equation q q_a conj(q)
         */
        static Quaternion quaternionFromTwoVectors(Vector3 v1, Vector3 v2);

        /**
        * Calculate quaternion from angular velocity
        * @param[in] omega Interprete angular velocity as axis
        * @param[in] dt sampling time
        * @return Quaternion representing the incremental rotation
        */
        static Quaternion quaternionFromAngularVelocity(aut4::Vector3 omega, double dt);
        
        /**
         * Caclulate quaterion from axis and angle
         * @param[in] axis Rotation axis
         * @param[in] theta Rotation angle
         * @return Quaternion representing the desired rotation
         */
        static Quaternion quaternionFromAxisAngle(Vector3 axis, double theta);
    
        /*
         * Spherical linear interpolation
         * Code is from: http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/slerp/
         * @param[in] qTarget Targeting quaternion
         * @param[in] t Weighting factor (0 -> current quaternion, 1 -> qTarget)
         * @return Resulting quaternion
         */
        Quaternion slerp(Quaternion qTarget, double t);
        
        /*
         * Linear interpolation: Weight current quaternion w.r.t. qTarget with weight t
         * @param[in] qTarget Targeting quaternion
         * @param[in] t Weighting factor (0 -> current quaternion, 1 -> qTarget)
         * @return Resulting quaternion
         */
        Quaternion lerp(Quaternion qTarget, double t); 
        
        /**
         * Print quaternion elements to stdout
         */
        void print();

    public:

        /**
         * Real part 
         */
        double w;

        /**
         * Imaginary part (dimension i)
         */
        double x;

        /**
         * Imaginary part (dimension j)
         */
        double y;

        /**
         * Imaginary part (dimension k)
         */
        double z;
    };

    /**
     * Multiply operation, performing qRes = lhs * rhs 
     * @param[in] lhs left hand side of operands
     * @param[in] rhs right hand side of operands
     * @return resulting quaternion
     */
    Quaternion operator*(const Quaternion &lhs, const Quaternion &rhs);

}

#endif
