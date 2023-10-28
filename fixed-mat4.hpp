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
  };

