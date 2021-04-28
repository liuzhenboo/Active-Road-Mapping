

#include <pcl/common/eigen.h>
#include <pcl/common/common.h>

#include <iomanip>

#include <Attitude.h>
Attitude::Attitude() : data_(cv::Mat::zeros(3, 4, CV_32FC1))
{
}

// rotation matrix r## and origin o##
Attitude::Attitude(
    float r11, float r12, float r13, float o14,
    float r21, float r22, float r23, float o24,
    float r31, float r32, float r33, float o34)
{
    data_ = (cv::Mat_<float>(3, 4) << r11, r12, r13, o14,
             r21, r22, r23, o24,
             r31, r32, r33, o34);
}

Attitude::Attitude(const cv::Mat &AttitudeationMatrix)
{
    if (AttitudeationMatrix.type() == CV_32FC1)
    {
        data_ = AttitudeationMatrix;
    }
    else
    {
        AttitudeationMatrix.convertTo(data_, CV_32F);
    }
}

Attitude::Attitude(float x, float y, float z, float roll, float pitch, float yaw)
{
    Eigen::Affine3f t = pcl::getTransformation(x, y, z, roll, pitch, yaw);
    *this = fromEigen3f(t);
}

Attitude::Attitude(float x, float y, float z, float qx, float qy, float qz, float qw) : data_(cv::Mat::zeros(3, 4, CV_32FC1))
{
    Eigen::Matrix3f rotation = Eigen::Quaternionf(qw, qx, qy, qz).normalized().toRotationMatrix();
    data()[0] = rotation(0, 0);
    data()[1] = rotation(0, 1);
    data()[2] = rotation(0, 2);
    data()[3] = x;
    data()[4] = rotation(1, 0);
    data()[5] = rotation(1, 1);
    data()[6] = rotation(1, 2);
    data()[7] = y;
    data()[8] = rotation(2, 0);
    data()[9] = rotation(2, 1);
    data()[10] = rotation(2, 2);
    data()[11] = z;
}

Attitude::Attitude(float x, float y, float theta)
{
    Eigen::Affine3f t = pcl::getTransformation(x, y, 0, 0, 0, theta);
    *this = fromEigen3f(t);
}

Attitude Attitude::clone() const
{
    return Attitude(data_.clone());
}

bool Attitude::isNull() const
{
    return (data_.empty() ||
            (data()[0] == 0.0f &&
             data()[1] == 0.0f &&
             data()[2] == 0.0f &&
             data()[3] == 0.0f &&
             data()[4] == 0.0f &&
             data()[5] == 0.0f &&
             data()[6] == 0.0f &&
             data()[7] == 0.0f &&
             data()[8] == 0.0f &&
             data()[9] == 0.0f &&
             data()[10] == 0.0f &&
             data()[11] == 0.0f) ||
            std::isnan(data()[0]) ||
            std::isnan(data()[1]) ||
            std::isnan(data()[2]) ||
            std::isnan(data()[3]) ||
            std::isnan(data()[4]) ||
            std::isnan(data()[5]) ||
            std::isnan(data()[6]) ||
            std::isnan(data()[7]) ||
            std::isnan(data()[8]) ||
            std::isnan(data()[9]) ||
            std::isnan(data()[10]) ||
            std::isnan(data()[11]));
}

bool Attitude::isIdentity() const
{
    return data()[0] == 1.0f &&
           data()[1] == 0.0f &&
           data()[2] == 0.0f &&
           data()[3] == 0.0f &&
           data()[4] == 0.0f &&
           data()[5] == 1.0f &&
           data()[6] == 0.0f &&
           data()[7] == 0.0f &&
           data()[8] == 0.0f &&
           data()[9] == 0.0f &&
           data()[10] == 1.0f &&
           data()[11] == 0.0f;
}

void Attitude::setNull()
{
    *this = Attitude();
}

void Attitude::setIdentity()
{
    *this = getIdentity();
}

float Attitude::theta() const
{
    float roll, pitch, yaw;
    this->getEulerAngles(roll, pitch, yaw);
    return yaw;
}

Attitude Attitude::inverse() const
{
    return fromEigen4f(toEigen4f().inverse());
}

Attitude Attitude::rotation() const
{
    return Attitude(
        data()[0], data()[1], data()[2], 0,
        data()[4], data()[5], data()[6], 0,
        data()[8], data()[9], data()[10], 0);
}

Attitude Attitude::translation() const
{
    return Attitude(1, 0, 0, data()[3],
                    0, 1, 0, data()[7],
                    0, 0, 1, data()[11]);
}

Attitude Attitude::to3DoF() const
{
    float x, y, z, roll, pitch, yaw;
    this->getTranslationAndEulerAngles(x, y, z, roll, pitch, yaw);
    return Attitude(x, y, 0, 0, 0, yaw);
}

cv::Mat Attitude::rotationMatrix() const
{
    return data_.colRange(0, 3).clone();
}

cv::Mat Attitude::translationMatrix() const
{
    return data_.col(3).clone();
}

void Attitude::getTranslationAndEulerAngles(float &x, float &y, float &z, float &roll, float &pitch, float &yaw) const
{
    pcl::getTranslationAndEulerAngles(toEigen3f(), x, y, z, roll, pitch, yaw);
}

void Attitude::getEulerAngles(float &roll, float &pitch, float &yaw) const
{
    float x, y, z;
    pcl::getTranslationAndEulerAngles(toEigen3f(), x, y, z, roll, pitch, yaw);
}

void Attitude::getTranslation(float &x, float &y, float &z) const
{
    x = this->x();
    y = this->y();
    z = this->z();
}

float Attitude::getAngle(float x, float y, float z) const
{
    Eigen::Vector3f vA(x, y, z);
    Eigen::Vector3f vB = this->toEigen3f().linear() * Eigen::Vector3f(1, 0, 0);
    return pcl::getAngle3D(Eigen::Vector4f(vA[0], vA[1], vA[2], 0), Eigen::Vector4f(vB[0], vB[1], vB[2], 0));
}

Attitude Attitude::interpolate(float t, const Attitude &other) const
{
    Eigen::Quaternionf qa = this->getQuaternionf();
    Eigen::Quaternionf qb = other.getQuaternionf();
    Eigen::Quaternionf qres = qa.slerp(t, qb);

    float x = this->x() + t * (other.x() - this->x());
    float y = this->y() + t * (other.y() - this->y());
    float z = this->z() + t * (other.z() - this->z());

    return Attitude(x, y, z, qres.x(), qres.y(), qres.z(), qres.w());
}

void Attitude::normalizeRotation()
{
    if (!this->isNull())
    {
        Eigen::Affine3f m = toEigen3f();
        m.linear() = Eigen::Quaternionf(m.linear()).normalized().toRotationMatrix();
        *this = fromEigen3f(m);
    }
}

Attitude Attitude::operator*(const Attitude &t) const
{
    Eigen::Affine3f m = Eigen::Affine3f(toEigen4f() * t.toEigen4f());
    // make sure rotation is always normalized!
    m.linear() = Eigen::Quaternionf(m.linear()).normalized().toRotationMatrix();
    return fromEigen3f(m);
}

Attitude &Attitude::operator*=(const Attitude &t)
{
    *this = *this * t;
    return *this;
}

bool Attitude::operator==(const Attitude &t) const
{
    return memcmp(data_.data, t.data_.data, data_.total() * sizeof(float)) == 0;
}

bool Attitude::operator!=(const Attitude &t) const
{
    return !(*this == t);
}

std::ostream &operator<<(std::ostream &os, const Attitude &s)
{
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            os << std::left << std::setw(12) << s.data()[i * 4 + j] << " ";
        }
        os << std::endl;
    }
    return os;
}

Eigen::Matrix4f Attitude::toEigen4f() const
{
    Eigen::Matrix4f m;
    m << data()[0], data()[1], data()[2], data()[3],
        data()[4], data()[5], data()[6], data()[7],
        data()[8], data()[9], data()[10], data()[11],
        0, 0, 0, 1;
    return m;
}
Eigen::Matrix4d Attitude::toEigen4d() const
{
    Eigen::Matrix4d m;
    m << data()[0], data()[1], data()[2], data()[3],
        data()[4], data()[5], data()[6], data()[7],
        data()[8], data()[9], data()[10], data()[11],
        0, 0, 0, 1;
    return m;
}

Eigen::Affine3f Attitude::toEigen3f() const
{
    return Eigen::Affine3f(toEigen4f());
}

Eigen::Affine3d Attitude::toEigen3d() const
{
    return Eigen::Affine3d(toEigen4d());
}

Eigen::Quaternionf Attitude::getQuaternionf() const
{
    return Eigen::Quaternionf(this->toEigen3f().linear()).normalized();
}

Eigen::Quaterniond Attitude::getQuaterniond() const
{
    return Eigen::Quaterniond(this->toEigen3d().linear()).normalized();
}

Attitude Attitude::getIdentity()
{
    return Attitude(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0);
}

Attitude Attitude::fromEigen4f(const Eigen::Matrix4f &matrix)
{
    return Attitude(matrix(0, 0), matrix(0, 1), matrix(0, 2), matrix(0, 3),
                    matrix(1, 0), matrix(1, 1), matrix(1, 2), matrix(1, 3),
                    matrix(2, 0), matrix(2, 1), matrix(2, 2), matrix(2, 3));
}
Attitude Attitude::fromEigen4d(const Eigen::Matrix4d &matrix)
{
    return Attitude(matrix(0, 0), matrix(0, 1), matrix(0, 2), matrix(0, 3),
                    matrix(1, 0), matrix(1, 1), matrix(1, 2), matrix(1, 3),
                    matrix(2, 0), matrix(2, 1), matrix(2, 2), matrix(2, 3));
}

Attitude Attitude::fromEigen3f(const Eigen::Affine3f &matrix)
{
    return Attitude(matrix(0, 0), matrix(0, 1), matrix(0, 2), matrix(0, 3),
                    matrix(1, 0), matrix(1, 1), matrix(1, 2), matrix(1, 3),
                    matrix(2, 0), matrix(2, 1), matrix(2, 2), matrix(2, 3));
}
Attitude Attitude::fromEigen3d(const Eigen::Affine3d &matrix)
{
    return Attitude(matrix(0, 0), matrix(0, 1), matrix(0, 2), matrix(0, 3),
                    matrix(1, 0), matrix(1, 1), matrix(1, 2), matrix(1, 3),
                    matrix(2, 0), matrix(2, 1), matrix(2, 2), matrix(2, 3));
}

Attitude Attitude::fromEigen3f(const Eigen::Isometry3f &matrix)
{
    return Attitude(matrix(0, 0), matrix(0, 1), matrix(0, 2), matrix(0, 3),
                    matrix(1, 0), matrix(1, 1), matrix(1, 2), matrix(1, 3),
                    matrix(2, 0), matrix(2, 1), matrix(2, 2), matrix(2, 3));
}
Attitude Attitude::fromEigen3d(const Eigen::Isometry3d &matrix)
{
    return Attitude(matrix(0, 0), matrix(0, 1), matrix(0, 2), matrix(0, 3),
                    matrix(1, 0), matrix(1, 1), matrix(1, 2), matrix(1, 3),
                    matrix(2, 0), matrix(2, 1), matrix(2, 2), matrix(2, 3));
}
