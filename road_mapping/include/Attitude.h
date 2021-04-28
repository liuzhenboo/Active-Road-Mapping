#ifndef Attitude_H_
#define Attitude_H_

#include <vector>
#include <string>
#include <map>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>

class Attitude
{
public:
    // Zero by default
    Attitude();
    // rotation matrix r## and origin o##
    Attitude(float r11, float r12, float r13, float o14,
             float r21, float r22, float r23, float o24,
             float r31, float r32, float r33, float o34);
    // should have 3 rows, 4 cols and type CV_32FC1
    Attitude(const cv::Mat &AttitudeationMatrix);
    // x,y,z, roll,pitch,yaw
    Attitude(float x, float y, float z, float roll, float pitch, float yaw);
    // x,y,z, qx,qy,qz,qw
    Attitude(float x, float y, float z, float qx, float qy, float qz, float qw);
    // x,y, theta
    Attitude(float x, float y, float theta);

    Attitude clone() const;

    float r11() const { return data()[0]; }
    float r12() const { return data()[1]; }
    float r13() const { return data()[2]; }
    float r21() const { return data()[4]; }
    float r22() const { return data()[5]; }
    float r23() const { return data()[6]; }
    float r31() const { return data()[8]; }
    float r32() const { return data()[9]; }
    float r33() const { return data()[10]; }

    float o14() const { return data()[3]; }
    float o24() const { return data()[7]; }
    float o34() const { return data()[11]; }

    float &operator[](int index) { return data()[index]; }
    const float &operator[](int index) const { return data()[index]; }
    float &operator()(int row, int col) { return data()[row * 4 + col]; }
    const float &operator()(int row, int col) const { return data()[row * 4 + col]; }

    bool isNull() const;
    bool isIdentity() const;

    void setNull();
    void setIdentity();

    const cv::Mat &dataMatrix() const { return data_; }
    const float *data() const { return (const float *)data_.data; }
    float *data() { return (float *)data_.data; }
    int size() const { return 12; }

    float &x() { return data()[3]; }
    float &y() { return data()[7]; }
    float &z() { return data()[11]; }
    const float &x() const { return data()[3]; }
    const float &y() const { return data()[7]; }
    const float &z() const { return data()[11]; }

    float theta() const;

    Attitude inverse() const;
    Attitude rotation() const;
    Attitude translation() const;
    Attitude to3DoF() const;

    cv::Mat rotationMatrix() const;
    cv::Mat translationMatrix() const;

    void getTranslationAndEulerAngles(float &x, float &y, float &z, float &roll, float &pitch, float &yaw) const;
    void getEulerAngles(float &roll, float &pitch, float &yaw) const;
    void getTranslation(float &x, float &y, float &z) const;
    float getAngle(float x = 1.0f, float y = 0.0f, float z = 0.0f) const;

    Attitude interpolate(float t, const Attitude &other) const;
    void normalizeRotation();

    Attitude operator*(const Attitude &t) const;
    Attitude &operator*=(const Attitude &t);
    bool operator==(const Attitude &t) const;
    bool operator!=(const Attitude &t) const;

    Eigen::Matrix4f toEigen4f() const;
    Eigen::Matrix4d toEigen4d() const;
    Eigen::Affine3f toEigen3f() const;
    Eigen::Affine3d toEigen3d() const;

    Eigen::Quaternionf getQuaternionf() const;
    Eigen::Quaterniond getQuaterniond() const;

public:
    static Attitude getIdentity();
    static Attitude fromEigen4f(const Eigen::Matrix4f &matrix);
    static Attitude fromEigen4d(const Eigen::Matrix4d &matrix);
    static Attitude fromEigen3f(const Eigen::Affine3f &matrix);
    static Attitude fromEigen3d(const Eigen::Affine3d &matrix);
    static Attitude fromEigen3f(const Eigen::Isometry3f &matrix);
    static Attitude fromEigen3d(const Eigen::Isometry3d &matrix);

private:
    cv::Mat data_;
};

// std::ostream &operator<<(std::ostream &os, const Attitude &s);

// class AttitudeStamped
// {
// public:
//     AttitudeStamped(const Attitude &Attitude, const double &stamp) : Attitude_(Attitude),
//                                                                         stamp_(stamp)
//     {
//     }
//     const Attitude &Attitude() const { return Attitude_; }
//     const double &stamp() const { return stamp_; }

// private:
//     Attitude Attitude_;
//     double stamp_;
// };

#endif /* Attitude_H_ */
