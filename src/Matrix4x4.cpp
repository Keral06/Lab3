#include "Matrix4x4.hpp"
#include <cmath>
#include <stdexcept>

#define TOL 1e-6

Matrix4x4 Matrix4x4::Identity()
{
    Matrix4x4 I;
    I.At(0, 0) = 1; I.At(1, 1) = 1; I.At(2, 2) = 1; I.At(3, 3) = 1;
    return I;
}

Matrix4x4 Matrix4x4::Multiply(const Matrix4x4& B) const
{
    Matrix4x4 C{};
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            double sum = 0.0;
            for (int k = 0; k < 4; ++k) {
                sum += At(i, k) * B.At(k, j);
            }
            C.At(i, j) = sum;
        }
    }
    return C;
}

Vec4 Matrix4x4::Multiply(const Vec4& v) const
{
    Vec4 res;
    res.x = At(0, 0) * v.x + At(0, 1) * v.y + At(0, 2) * v.z + At(0, 3) * v.w;
    res.y = At(1, 0) * v.x + At(1, 1) * v.y + At(1, 2) * v.z + At(1, 3) * v.w;
    res.z = At(2, 0) * v.x + At(2, 1) * v.y + At(2, 2) * v.z + At(2, 3) * v.w;
    res.w = At(3, 0) * v.x + At(3, 1) * v.y + At(3, 2) * v.z + At(3, 3) * v.w;
    return res;
}

// --------------------------------------------------------------------------
// TODO LAB 3
// --------------------------------------------------------------------------

bool Matrix4x4::IsAffine() const
{
    if (std::abs(At(3, 0)) > TOL) {
        return false;
    }
    if (std::abs(At(3, 1)) > TOL) {
        return false;
    }
    if (std::abs(At(3, 2)) > TOL) {
        return false;
    }
    if (std::abs(At(3, 3) - 1) > TOL) {
        return false;
    }
    else {
        return true;
    }
}

Vec3 Matrix4x4::TransformPoint(const Vec3& p) const
{
    return Vec3{};
}

Vec3 Matrix4x4::TransformVector(const Vec3& v) const
{
    Vec4 v4(v.x, v.y, v.z, 1.0f);

    Vec4 result = this->Multiply(v4);
    if (std::abs(result.w) > TOL && std::abs(result.w - 1.0f) > TOL) {
        float div = 1.0f / result.w;
        return Vec3(result.x * div, result.y * div, result.z * div);
    }

    return Vec3(result.x, result.y, result.z);
}

Matrix4x4 Matrix4x4::Translate(const Vec3& t)
{
    Matrix4x4 M;

    M.Identity();

    M.At(0,3) = t.x;
    M.At(1,3) = t.y;
    M.At(2,3) = t.z;

    return M;
}

Matrix4x4 Matrix4x4::Scale(const Vec3& s)
{
    Matrix4x4 M;
    M.Identity();

    M.At(0, 0) = s.x;  
    M.At(1, 1) = s.y;  
    M.At(2, 2) = s.z;  


    return M;
}

Matrix4x4 Matrix4x4::Rotate(const Matrix3x3& R)
{
    Matrix4x4 M;
    M.Identity();

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            M.At(i, j) = R.At(i, j);
        }
    }

    return M;
}

Matrix4x4 Matrix4x4::Rotate(const Quat& q)
{
    Matrix4x4 M;
    Matrix3x3 R;
    R = q.ToMatrix3x3();
    M = Rotate(R);

    return M;
}

Matrix4x4 Matrix4x4::FromTRS(const Vec3& t, const Matrix3x3& R, const Vec3& s)
{
    Matrix4x4 M;
    M = Rotate(R);

    float scales[3] = { s.x, s.y, s.z };

    for (int j = 0; j < 3; ++j)
    {
        for (int i = 0; i < 3; ++i)
        {
            M.At(i, j) = M.At(i, j) * scales[j];
        }
    }

    M.At(0, 3) = t.x;
    M.At(1, 3) = t.y;
    M.At(2, 3) = t.z;

    return M;
}

Matrix4x4 Matrix4x4::FromTRS(const Vec3& t, const Quat& q, const Vec3& s)
{
    Matrix4x4 M;
    M = Rotate(q);

    float scales[3] = { s.x, s.y, s.z };

    for (int j = 0; j < 3; ++j)
    {
        for (int i = 0; i < 3; ++i)
        {
            M.At(i, j) = M.At(i, j) * scales[j];
        }
    }

    M.At(0, 3) = t.x;
    M.At(1, 3) = t.y;
    M.At(2, 3) = t.z;

    return M;
}

Matrix4x4 Matrix4x4::InverseTR() const
{
    Matrix4x4 M;
    Matrix3x3 R;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            R.At(i, j) = At(j, i);
        }
    }

    Vec3 p(At(0, 3), At(1, 3), At(2, 3));
    Vec3 pi;
    pi.x = -(R.At(0, 0) * p.x + R.At(0, 1) * p.y + R.At(0, 2) * p.z);
    pi.y = -(R.At(1, 0) * p.x + R.At(1, 1) * p.y + R.At(1, 2) * p.z);
    pi.z = -(R.At(2, 0) * p.x + R.At(2, 1) * p.y + R.At(2, 2) * p.z);

    M = FromTRS(pi, R, Vec3(1.0f, 1.0f, 1.0f));
    return M;
}

Matrix4x4 Matrix4x4::InverseTRS() const
{
    Matrix4x4 M;
    return M;
}

Vec3 Matrix4x4::GetTranslation() const
{
    return Vec3{};
}

Matrix3x3 Matrix4x4::GetRotationScale() const
{
    Matrix3x3 M;
    return M;
}

Vec3 Matrix4x4::GetScale() const
{
	return Vec3{};
}

Matrix3x3 Matrix4x4::GetRotation() const
{
    Matrix3x3 M;
    return M;
}

Quat Matrix4x4::GetRotationQuat() const
{
	return Quat{};
}

void Matrix4x4::SetTranslation(const Vec3& t)
{
    
}

void Matrix4x4::SetScale(const Vec3& s)
{
    
}

void Matrix4x4::SetRotation(const Matrix3x3& R)
{
    
}

void Matrix4x4::SetRotation(const Quat& q)
{
   
}

void Matrix4x4::SetRotationScale(const Matrix3x3& RS)
{
    
}