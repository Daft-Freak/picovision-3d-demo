#pragma once

#include "types/mat4.hpp"
#include "types/vec3.hpp"
#include "fixed-mat4.hpp"

class Vec4
{
public:
    constexpr Vec4() : Vec4(0.0f, 0.0f, 0.0f) {}
    constexpr Vec4(float x, float y, float z, float w = 1.0f) : x(x), y(y), z(z), w(w){}
    explicit constexpr Vec4(const blit::Vec3 &v, float w = 1.0f) : Vec4(v.x, v.y, v.z, w) {}

    explicit constexpr operator blit::Vec3(){return {x, y, z};};

    float x, y, z, w;
};

inline Vec4 operator*(const blit::Mat4 &mat, const Vec4 &vec)
{
    Vec4 out;
    out.x = (vec.x * mat.v00) + (vec.y * mat.v01) + (vec.z * mat.v02) + (vec.w * mat.v03);
    out.y = (vec.x * mat.v10) + (vec.y * mat.v11) + (vec.z * mat.v12) + (vec.w * mat.v13);
    out.z = (vec.x * mat.v20) + (vec.y * mat.v21) + (vec.z * mat.v22) + (vec.w * mat.v23);
    out.w = (vec.x * mat.v30) + (vec.y * mat.v31) + (vec.z * mat.v32) + (vec.w * mat.v33);

    return out;
}

template <class T = int32_t, int frac_bits = sizeof(T) * 4>
class FixedVec4
{
public:
    using FixedT = Fixed<T, frac_bits>;

    constexpr FixedVec4() : FixedVec4(0, 0, 0){}
    constexpr FixedVec4(FixedT x, FixedT y, FixedT z, FixedT w = 1) : x(x), y(y), z(z), w(w){}
    constexpr FixedVec4(float x, float y, float z, float w = 1.0f) : x(x), y(y), z(z), w(w){} 

    FixedT x, y, z, w;
};

template <class T, int frac_bits>
inline FixedVec4<T, frac_bits> operator*(const FixedMat4<T, frac_bits> &mat, const FixedVec4<T, frac_bits> &vec)
{
    FixedVec4<T, frac_bits> out;
    out.x = (vec.x * mat.v00) + (vec.y * mat.v01) + (vec.z * mat.v02) + (vec.w * mat.v03);
    out.y = (vec.x * mat.v10) + (vec.y * mat.v11) + (vec.z * mat.v12) + (vec.w * mat.v13);
    out.z = (vec.x * mat.v20) + (vec.y * mat.v21) + (vec.z * mat.v22) + (vec.w * mat.v23);
    out.w = (vec.x * mat.v30) + (vec.y * mat.v31) + (vec.z * mat.v32) + (vec.w * mat.v33);

    return out;
}