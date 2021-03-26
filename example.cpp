#include "quaternions.h"

int main()
{
    // This is the identity quaternion.
    cb::Quaternion identity{1, 0, 0, 0};                      

    // This is some random rotation quaternion.
    cb::Quaternion someRotation = cb::Quaternion{1, 0.7, 0.4, 1.5}.normalized();

    // This is a quaternion representation of the x axis unit vector.
    cb::Quaternion xVector = cb::Quaternion{0, 1, 0, 0};

    // This rotates the x axis vector using someRotation.
    auto rotatedVector = someRotation * xVector * someRotation.conjugate();

    // Print result.
    std::cout << "The original 3-vector:    " << xVector << std::endl;
    std::cout << "The rotation quaternion:  " << someRotation << std::endl;
    std::cout << "The rotated 3-vector:     " << rotatedVector << std::endl;

    // This creates a rotation from the axis angle form; a 35 degree rotation about the y axis.
    cb::Quaternion yRotation = cb::Quaternion{35*3.14159/180.0, {0, 1, 0}};

    // This demonstrates recovery of the axis angle form from the quaternion.
    std::cout << "Angle:                   " << yRotation.getAngle() << "\n";
    auto axis = yRotation.getAxis();
    std::cout << axis[0] << " " << axis[1] << " " << axis[2] << "\n";

    std::cout << yRotation << std::endl;
    std::cout << yRotation * xVector * yRotation.conjugate() << std::endl;


    return 0;
}