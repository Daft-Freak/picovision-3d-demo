#pragma once

#include "fixed.hpp"

#include "types/mat4.hpp"

template <class T = int32_t, int frac_bits = sizeof(T) * 4>
struct FixedMat4
{
    Fixed<T, frac_bits> v00, v01, v02, v03;
    Fixed<T, frac_bits> v10, v11, v12, v13;
    Fixed<T, frac_bits> v20, v21, v22, v23;
    Fixed<T, frac_bits> v30, v31, v32, v33;

    FixedMat4()
    {
        v00 = 0.0f; v01 = 0.0f; v02 = 0.0f; v03 = 0.0f;
        v10 = 0.0f; v11 = 0.0f; v12 = 0.0f; v13 = 0.0f;
        v20 = 0.0f; v21 = 0.0f; v22 = 0.0f; v23 = 0.0f;
        v30 = 0.0f; v31 = 0.0f; v32 = 0.0f; v33 = 0.0f;
    }

    FixedMat4(const blit::Mat4 &m)
    {
        v00 = m.v00; v01 = m.v01; v02 = m.v02; v03 = m.v03;
        v10 = m.v10; v11 = m.v11; v12 = m.v12; v13 = m.v13;
        v20 = m.v20; v21 = m.v21; v22 = m.v22; v23 = m.v23;
        v30 = m.v30; v31 = m.v31; v32 = m.v32; v33 = m.v33;
    }

    inline FixedMat4& operator*=(const FixedMat4 &m)
    {
        auto r00 = v00 * m.v00 + v01 * m.v10 + v02 * m.v20 + v03 * m.v30;
        auto r01 = v00 * m.v01 + v01 * m.v11 + v02 * m.v21 + v03 * m.v31;
        auto r02 = v00 * m.v02 + v01 * m.v12 + v02 * m.v22 + v03 * m.v32;
        auto r03 = v00 * m.v03 + v01 * m.v13 + v02 * m.v23 + v03 * m.v33;
        auto r10 = v10 * m.v00 + v11 * m.v10 + v12 * m.v20 + v13 * m.v30;
        auto r11 = v10 * m.v01 + v11 * m.v11 + v12 * m.v21 + v13 * m.v31;
        auto r12 = v10 * m.v02 + v11 * m.v12 + v12 * m.v22 + v13 * m.v32;
        auto r13 = v10 * m.v03 + v11 * m.v13 + v12 * m.v23 + v13 * m.v33;
        auto r20 = v20 * m.v00 + v21 * m.v10 + v22 * m.v20 + v23 * m.v30;
        auto r21 = v20 * m.v01 + v21 * m.v11 + v22 * m.v21 + v23 * m.v31;
        auto r22 = v20 * m.v02 + v21 * m.v12 + v22 * m.v22 + v23 * m.v32;
        auto r23 = v20 * m.v03 + v21 * m.v13 + v22 * m.v23 + v23 * m.v33;
        auto r30 = v30 * m.v00 + v31 * m.v10 + v32 * m.v20 + v33 * m.v30;
        auto r31 = v30 * m.v01 + v31 * m.v11 + v32 * m.v21 + v33 * m.v31;
        auto r32 = v30 * m.v02 + v31 * m.v12 + v32 * m.v22 + v33 * m.v32;
        auto r33 = v30 * m.v03 + v31 * m.v13 + v32 * m.v23 + v33 * m.v33;

        v00 = r00; v01 = r01; v02 = r02; v03 = r03;
        v10 = r10; v11 = r11; v12 = r12; v13 = r13;
        v20 = r20; v21 = r21; v22 = r22; v23 = r23;
        v30 = r30; v31 = r31; v32 = r32; v33 = r33;

        return *this;
    }
};

template <class T = int32_t, int frac_bits = sizeof(T) * 4>
inline FixedMat4<T, frac_bits> operator*(FixedMat4<T, frac_bits> lhs, const FixedMat4<T, frac_bits> &m) { lhs *= m; return lhs; }
