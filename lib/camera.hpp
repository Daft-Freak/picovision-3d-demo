#pragma once

#include "types/mat4.hpp"
#include "types/vec3.hpp"

class Camera
{
public:
    blit::Mat4 get_look_matrix() const
    {
        auto front = look_at - position;
        front.normalize();

        auto side = front.cross(up);
        auto up2 = side.cross(front);

        auto mat = blit::Mat4::identity();

        mat.v00 =   side.x; mat.v01 =   side.y; mat.v02 =   side.z; mat.v03 = (-position).dot(  side);
        mat.v10 =    up2.x; mat.v11 =    up2.y; mat.v12 =    up2.z; mat.v13 = (-position).dot(   up2);
        mat.v20 = -front.x; mat.v21 = -front.y; mat.v22 = -front.z; mat.v23 = (-position).dot(-front);

        return mat;
    }

    static blit::Mat4 frustum_matrix(float left, float right, float bottom, float top, float near, float far)
    {
        auto mat = blit::Mat4::identity();

        mat.v00 = (2.0f * near) / (right - left);

        mat.v11 = (2.0f * near) / (top - bottom);

        mat.v02 = (right + left) / (right - left);
        mat.v12 = (top + bottom) / (top - bottom);
        mat.v22 = -((far + near) / (far - near));
        mat.v32 = -1.0f;

        mat.v23 = -((2 * far * near) / (far - near));
        mat.v33 = 0.0f;

        return mat;
    }

    static blit::Mat4 perspective_matrix(float near, float far, float fov, float aspect)
    {
        auto tan_fov = std::tan(fov);

        auto near_h = near * tan_fov;
        auto near_w = near_h * aspect;

        return frustum_matrix(-near_w, near_w, near_h, -near_h, near, far);
    }

    blit::Vec3 position;
	blit::Vec3 look_at;
	blit::Vec3 up = {0.0f, 1.0f, 0.0f};
};