#include "quaternions.h"

int main()
{
    cb::Quaternion quat1 = cb::Quaternion{1, 0, 0, 0};                      // The identity quaternion
    cb::Quaternion quat2 = cb::Quaternion{1, 0.7, 0.4, 1.5}.normalized();   // A random rotation

    cb::Quaternion xvector = cb::Quaternion{0, 1, 0, 0};                     // A 3-vector to be rotated

    // Rotate the 3 vector using quat2.
    auto rotatedVector = quat2 * xvector * quat2.conjugate();

    // Print result.
    std::cout << "The original 3-vector:    " << xvector << std::endl;
    std::cout << "The rotation quaternion:  " << quat2 << std::endl;
    std::cout << "The rotated 3-vector:     " << rotatedVector << std::endl;

    cb::Quaternion quat3 = cb::Quaternion{35*3.14159/180.0, {0, 1, 0}}.normalized();                      // Rotation around the y axis by 35 degrees.

    std::cout << quat3 << std::endl;
    std::cout << quat3 * xvector * quat3.conjugate() << std::endl;


    return 0;
}