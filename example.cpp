#include "quaternions.h"

int main()
{
    // This is the identity quaternion.
    cb::Quaternion identity{1, 0, 0, 0};                      

    // This is some random rotation quaternion.
    cb::Quaternion someRotation = cb::Quaternion{1, 0.7, 0.4, 1.5}.normalized();
    std::cout << "Some random rotation:                               " << someRotation << std::endl;

    // This is a quaternion representation of the x axis unit vector.
    cb::Quaternion xVector = cb::Quaternion{0, 1, 0, 0};
    std::cout << "x axis unit vector as quaternion:                   " << xVector << std::endl;

    // This rotates the x axis vector using someRotation.
    auto rotatedXVector = someRotation * xVector * someRotation.conjugate();
    std::cout << "x axis unit vector after the random rotation above: " << rotatedXVector << std::endl;

    // This creates a rotation from the axis angle form; a 35 degree rotation about the y axis.
    cb::Quaternion yRotation = cb::Quaternion{35*3.14159/180.0, {0, 1, 0}};
    std::cout << "Rotation of 35 degrees around y axis:               " << yRotation << std::endl;

    // This applies that y rotation to the x unit vector/
    auto yRotatedXVector = yRotation * xVector * yRotation.conjugate();
    std::cout << "x unit vector after that y rotation:                " << yRotatedXVector << std::endl;

    // This demonstrates recovery of the axis angle form from the quaternion.
    std::cout << "Angle recovered from the y rotation quaternion:     " << yRotation.getAngle() << std::endl;
    auto axis = yRotation.getAxis();
    std::cout << "Axis recovered from the y rotation quaternion:      " << axis[0] << " " << axis[1] << " " << axis[2] << std::endl;

    std::cout << "Angle recovered from the random quaternion:         " << someRotation.getAngle() << std::endl;
    auto axis2 = someRotation.getAxis();
    std::cout << "Axis recovered from the random quaternion:          " << axis2[0] << " " << axis2[1] << " " << axis2[2] << std::endl;

    return 0;
}
