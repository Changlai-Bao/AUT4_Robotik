#ifndef __VECTOR3_H
#define __VECTOR3_H

#include <vector>

namespace aut4
{

    /**
     * @class Vector3
     * @brief 3 dimensional vector math implementation
     * @author Stefan May
     * @date 15.4.2021
     */
    class Vector3
    {
    public:
        /**
       * Default constructor
       */
        Vector3();

        /**
         *  Element-wise constructor
         * @param[in] x first element
         * @param[in] x first element
         * @param[in] x first element
         */
        Vector3(double x, double y, double z);

        /**
         * Destructor
         */
        ~Vector3();

        /**
         * Array operator
         * @param[in] index Element index
         * return Element value
         */
        double &operator[](unsigned int index);

        /**
         * Assign operator
         * @param[in] v Vector to be assigned
         */
        Vector3 &operator=(const Vector3 &v);

        /**
         * Multiply-assign operator
         * @param[in] val Scalar value to be multiply with
         */

        Vector3 &operator*=(const double val);
        /**
         * Addition-assign operator
         * @param[in] v Vector to be added to this instance
         */

        Vector3 &operator+=(const Vector3 &v);
        /**
         * Subtract-assign operator
         * @param[in] v Vector to be subtracted from this instance
         */

        Vector3 &operator-=(const Vector3 &v);

        /**
         * Calculate norm
         * @return norm of vector
         */
        double norm();

        /**
         * Normalize vector to length 1
         * @return Normalized vector
         */
        void normalize();

        /**
         * Calculate dot product
         * @param[in] v2 Second element of operation: v dot v2
         * return dot product
         */
        double dot(Vector3 &v2);

        /**
         * Calculate cross product
         * @param[in] v2 Second element of operation: v x v2
         * return cross product
         */
        Vector3 cross(Vector3 &v2);

        /**
         * Calculate rotation axis and rotation angle from two vectors.
         * @param[in] v1 first vector (to be rotated to the second one)
         * @param[in] v2 second vector
         * @param[out] axis axis of rotation
         * @param[out] angle angle of rotation
         */
        static void axisAngleFromTwoVectors(Vector3 v1, Vector3 v2, Vector3 &axis, double &angle);

        /**
         * Print vector elements to stdout
         */
        void print();

    private:

        /**
         * Data vector 
         */
        std::vector<double> _v;
    };

}

#endif