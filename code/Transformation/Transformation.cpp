#include<cmath>
#include"Eigen/Core"
#include"Eigen/Dense"
#include<iostream>

float getRadian(float angle);

int main(){

    // Basic Example of cpp
    std::cout << "Example of cpp \n";
    float a = 1.0, b = 2.0;
    std::cout << a << std::endl;
    std::cout << a/b << std::endl;
    std::cout << std::sqrt(b) << std::endl;
    std::cout << std::acos(-1) << std::endl;
    // C++ 三角函数运算采样弧度制
    // 度数乘以π/180就可以变成弧度，acos(-1)就是π
    std::cout << std::sin(90.0/180.0*acos(-1)) << std::endl;

    // Example of vector
    std::cout << "Example of vector \n";
    // 三维向量的定义
    Eigen::Vector3f v(1.0f,2.0f,3.0f);
    Eigen::Vector3f w(1.0f,0.0f,0.0f);
    // vector output
    std::cout << "Example of output \n";
    std::cout << v << std::endl;
    // vector add
    std::cout << "Example of add \n";
    std::cout << v + w << std::endl;
    // vector scalar multiply
    std::cout << "Example of scalar multiply \n";
    std::cout << v * 3.0f << std::endl;
    std::cout << 2.0f * v << std::endl;

    // Example of matrix
    std::cout << "Example of matrix \n";
    // matrix definition
    Eigen::Matrix3f i,j;
    i << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0;
    j << 2.0, 3.0, 1.0, 4.0, 6.0, 5.0, 9.0, 7.0, 8.0;
    // matrix output
    std::cout << "Example of output \n";
    std::cout << i << std::endl;
//    // matrix add i + j
//    std::cout << i + j << std::endl;
//    // matrix scalar multiply i * 2.0
//    std::cout << i * 2.0 << std::endl;
//    // matrix multiply i * j
//    std::cout << i * j << std::endl;
//    // matrix multiply vector i * v
//    std::cout << i * v << std::endl;

    // 给定一个点 P=(2,1), 将该点绕原点先逆时针旋转 45◦，再平移 (1,2)
    std::cout << "Transformation \n";
    Eigen::Vector3f p(2,1,1);
    Eigen::Matrix3f rotation, translation;
    float angle = 45.0f;
    rotation << cos(getRadian(angle)), -sin(getRadian(angle)),0,
                sin(getRadian(angle)), cos(getRadian(angle)),0,
                0,0,1;
    translation << 1, 0, 1,
                    0, 1, 2,
                    0, 0, 1;
    std::cout << translation*rotation*p << std::endl;

    return 0;
}

float getRadian(float angle) { return angle / 180.0 * acos(-1); }
