#ifndef QUATERNIONS_H
#define QUATERNIONS_H

#include <iostream>
#include <math.h>
#include <array>

// Class layout ===================================================================================

namespace cb
{
    class Quaternion
    {
        private:
            double w, x, y, z;
            double norm_;
            double angle;
            std::array<double, 3> axis;

        public:
            // Constructors
            Quaternion(double angle, std::array<double, 3> axis);
            Quaternion(double w, double x, double y, double z);

            // Static methods
            static double dotProduct(const Quaternion& q1, const Quaternion& q2);

            // Getters
            double getW() const;
            double getX() const;
            double getY() const;
            double getZ() const;
            double getAngle() const;
            std::array<double, 3> getAxis() const;
            double norm();
            bool isNormal();
            Quaternion normalized() const;
            Quaternion conjugate() const;
            Quaternion inverse();
    };
}

// Constructors ===================================================================================

/** \brief Construct a rotation from the axis-angle representation.
 * \param angle The angle in radians by which to rotate the frame.
 * \param axis The axis around which to rotate the frame.
 */
inline cb::Quaternion::Quaternion(double angle, std::array<double, 3> axis)
{
    double axisInvNorm = 1.0/sqrt(axis[0]*axis[0] + axis[1]*axis[1] + axis[2]*axis[2]);
    this->angle = angle;
    this->axis = axis;
    this->w = sin(angle/2.0);
    this->x = cos(angle/2.0) * axis[0] * axisInvNorm;
    this->y = cos(angle/2.0) * axis[1] * axisInvNorm;
    this->z = cos(angle/2.0) * axis[2] * axisInvNorm;
    this->norm_ = -1;
}

/** \brief Construct a quaternion.
 * \param w The real component of the quaternion.
 * \param x The i component of the quaternion.
 * \param y The j component of the quaternion.
 * \param z The k component of the quaternion.
 */
inline cb::Quaternion::Quaternion(double w, double x, double y, double z)
{
    this->w = w;
    this->x = x;
    this->y = y;
    this->z = z;
    this->angle = 2 * acos(w);
    this->axis[0] = x / sqrt(1 - w*w);
    this->axis[1] = y / sqrt(1 - w*w);
    this->axis[2] = z / sqrt(1 - w*w);
    norm_ = -1;
}

// Static methods =================================================================================

/** \brief Compute the inner product of two quaternions.
 * \param q1 The first quaternion.
 * \param q2 The second quaternion.
 * \return Scalar inner product of the two quaternions.
 */
inline double cb::Quaternion::dotProduct(const Quaternion& q1, const Quaternion& q2)
{
    return q1.getW() * q2.getW()
         + q1.getX() * q2.getX()
         + q1.getY() * q2.getY()
         + q1.getZ() * q2.getZ();
}

// Operators ======================================================================================

inline cb::Quaternion operator+(const cb::Quaternion& q1, const cb::Quaternion& q2)
{
    return cb::Quaternion{
        q1.getW() + q2.getW(),
        q1.getX() + q2.getX(),
        q1.getY() + q2.getY(),
        q1.getZ() + q2.getZ()
    };
}

inline cb::Quaternion operator-(const cb::Quaternion& q1, const cb::Quaternion& q2)
{
    return cb::Quaternion{
        q1.getW() - q2.getW(),
        q1.getX() - q2.getX(),
        q1.getY() - q2.getY(),
        q1.getZ() - q2.getZ()
    };
}

inline cb::Quaternion operator-(const cb::Quaternion& quaternion)
{
    return -1 * quaternion;
}

inline cb::Quaternion operator*(const cb::Quaternion& quat1, const cb::Quaternion& quat2)
{
    double p1 = quat1.getW();
    double p2 = quat1.getX();
    double p3 = quat1.getY();
    double p4 = quat1.getZ();

    double q1 = quat2.getW();
    double q2 = quat2.getX();
    double q3 = quat2.getY();
    double q4 = quat2.getZ();

    return cb::Quaternion{
        p1*q1 - p2*q2 - p3*q3 - p4*q4,
        p1*q2 + p2*q1 + p3*q4 - p4*q3,
        p1*q3 - p2*q4 + p3*q1 + p4*q2,
        p1*q4 + p2*q3 - p3*q2 + p4*q1
    };
}

template<typename Scalar>
inline cb::Quaternion operator*(const Scalar scalar, const cb::Quaternion& quaternion)
{
    return cb::Quaternion{
        scalar * quaternion.getW(),
        scalar * quaternion.getX(),
        scalar * quaternion.getY(),
        scalar * quaternion.getZ()
    };
}

template<typename Scalar>
inline cb::Quaternion operator*(const cb::Quaternion& quaternion, const Scalar scalar)
{
    return scalar * quaternion;
}

template<typename Divisor>
inline cb::Quaternion operator/(const cb::Quaternion& quaternion, const Divisor divisor)
{
    return (1.0/divisor) * quaternion;
}

/** \brief Two quaternions are equal if all their elements are equal.
 * \param quat1 The first quaternion.
 * \param quat2 The second quaternion.
 * \return Bool indicating equality.
 */
inline bool operator==(cb::Quaternion quat1, cb::Quaternion quat2)
{
    if (quat1.getW() != quat2.getW())
    {
        return false;
    }
    if (quat1.getX() != quat2.getX())
    {
        return false;
    }
    if (quat1.getY() != quat2.getY())
    {
        return false;
    }
    if (quat1.getZ() != quat2.getZ())
    {
        return false;
    }
    return true;
}

inline std::ostream& operator<<(std::ostream& stream, const cb::Quaternion& quaternion)
{
    stream << "quaternion(" << quaternion.getW() << ", " << quaternion.getX()
                    << ", " << quaternion.getY() << ", " << quaternion.getZ() << ")";
    return stream;
}

// Getters ========================================================================================

inline double cb::Quaternion::getW() const
{
    return w;
}

inline double cb::Quaternion::getX() const
{
    return x;
}

inline double cb::Quaternion::getY() const
{
    return y;
}

inline double cb::Quaternion::getZ() const
{
    return z;
}

double cb::Quaternion::getAngle() const
{
    return angle;
}

std::array<double, 3> cb::Quaternion::getAxis() const
{
    return axis;
}

/** \brief Compute the magnitude of the quaternion. */
inline double cb::Quaternion::norm()
{
    if (norm_ == -1)
    {
        norm_ = sqrt(w*w + x*x + y*y + z*z);
    }
    return norm_;
}

/** \brief Whether the quaternion has unit magnitude. */
inline bool cb::Quaternion::isNormal()
{
    return norm() == 1;
}

/** \brief The conjugate is formed by negating all imaginary components. */
inline cb::Quaternion cb::Quaternion::conjugate() const
{
    return Quaternion{w, -x, -y, -z};
}

/** \brief The inverse of a quaternion is its conjugate over the square of its magnitude. */
inline cb::Quaternion cb::Quaternion::inverse()
{
    double n = norm();
    return conjugate()/(n*n);
}

#endif
