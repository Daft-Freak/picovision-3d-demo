#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>

#include "32blit.hpp"

#include "render-3d.hpp"
#include "vec4.hpp"

using namespace blit;

// mat helpers

Mat4 frustum(float left, float right, float bottom, float top, float nearVal, float farVal)
{
    Mat4 mat = Mat4::identity();

    mat.v00 = (2.0f * nearVal) / (right - left);

    mat.v11 = (2.0f * nearVal) / (top - bottom);

    mat.v02 = (right + left) / (right - left);
    mat.v12 = (top + bottom) / (top - bottom);
    mat.v22 = -((farVal + nearVal) / (farVal - nearVal));
    mat.v32 = -1.0f;

    mat.v23 = -((2 * farVal * nearVal) / (farVal - nearVal));
    mat.v33 = 0.0f;

    return mat;
}

// "shaders"
// in is xyz
// out is xyzw rgb
void noColShader(const float *in, Render3D::VertexOutData *out, const Render3D &r)
{
    FixedVec4 tmp(in[0], in[1], in[2], 1.0f);
    
    // transform
    tmp = r.get_model_view_projection() * tmp;

    // pos
    out->x = tmp.x;
    out->y = tmp.y;
    out->z = tmp.z;
    out->w = tmp.w;

    // col
    out->r = 255;
    out->g = 255;
    out->b = 255;
}

// in is xyz rgb
// out is xyzw rgb
void colPassthroughShader(const float *in, Render3D::VertexOutData *out, const Render3D &r)
{
    FixedVec4 tmp(in[0], in[1], in[2], 1.0f);
    
    // transform
    tmp = r.get_model_view_projection() * tmp;

    // pos
    out->x = tmp.x;
    out->y = tmp.y;
    out->z = tmp.z;
    out->w = tmp.w;

    // col
    out->r = in[3] * 255.0f;
    out->g = in[4] * 255.0f;
    out->b = in[5] * 255.0f;
}

// in is xyz nx ny nx
// out is xyzw rgb
void litShader(const float *in, Render3D::VertexOutData *out, const Render3D &r)
{
    FixedVec4 tmp(in[0], in[1], in[2], 1.0f);
    
    // transform
    tmp = r.get_model_view_projection() * tmp;

    // pos
    out->x = tmp.x;
    out->y = tmp.y;
    out->z = tmp.z;
    out->w = tmp.w;

    // col
    Vec3 light(-0.577350269f, -0.577350269f, -0.577350269f);

    Vec4 nor(in[3], in[4], in[5], 1.0f);
    nor = r.get_model_view() * nor;
    float len = sqrt(nor.x * nor.x + nor.y * nor.y + nor.z * nor.z);
    nor.x /= len;
    nor.y /= len;
    nor.z /= len;

    float dot = light.x * nor.x + light.y * nor.y + light.z * nor.z;

    out->r = std::max(dot, 0.0f) * 255.0f;
    out->g = std::max(dot, 0.0f) * 255.0f;
    out->b = std::max(dot, 0.0f) * 255.0f;
}

//

Render3D r3d;
float ang = 0.0f, ang2 = 0.0f;

void init()
{
    set_screen_mode(ScreenMode::hires);

    auto near = 0.1, far = 10.0;
    auto tanFov = tan(0.785398163);
    auto nearH = near * tanFov;
    auto nearW = nearH * (4.0 / 3.0);
    r3d.set_projection(frustum(-nearW, nearW, -nearH, nearH, near, far));
}

void update(uint32_t time)
{

}

void render(uint32_t time)
{

    // cube with normals
    const float vertices_with_normals[]
    {
        -1.0f, -1.0f, -1.0f,  0.0f,  0.0f, -1.0f, //0
        -1.0f,  1.0f, -1.0f,  0.0f,  0.0f, -1.0f, //2
         1.0f,  1.0f, -1.0f,  0.0f,  0.0f, -1.0f, //3
         1.0f,  1.0f, -1.0f,  0.0f,  0.0f, -1.0f, //3
         1.0f, -1.0f, -1.0f,  0.0f,  0.0f, -1.0f, //1
        -1.0f, -1.0f, -1.0f,  0.0f,  0.0f, -1.0f, //0

        -1.0f, -1.0f,  1.0f,  0.0f,  0.0f,  1.0f, //4
         1.0f, -1.0f,  1.0f,  0.0f,  0.0f,  1.0f, //5
         1.0f,  1.0f,  1.0f,  0.0f,  0.0f,  1.0f, //7
         1.0f,  1.0f,  1.0f,  0.0f,  0.0f,  1.0f, //7
        -1.0f,  1.0f,  1.0f,  0.0f,  0.0f,  1.0f, //6
        -1.0f, -1.0f,  1.0f,  0.0f,  0.0f,  1.0f, //4

        -1.0f, -1.0f, -1.0f,  0.0f, -1.0f,  0.0f, //0
         1.0f, -1.0f, -1.0f,  0.0f, -1.0f,  0.0f, //1
         1.0f, -1.0f,  1.0f,  0.0f, -1.0f,  0.0f, //5
         1.0f, -1.0f,  1.0f,  0.0f, -1.0f,  0.0f, //5
        -1.0f, -1.0f,  1.0f,  0.0f, -1.0f,  0.0f, //4
        -1.0f, -1.0f, -1.0f,  0.0f, -1.0f,  0.0f, //0

         1.0f, -1.0f, -1.0f,  1.0f,  0.0f,  0.0f, //1
         1.0f,  1.0f, -1.0f,  1.0f,  0.0f,  0.0f, //3
         1.0f,  1.0f,  1.0f,  1.0f,  0.0f,  0.0f, //7
         1.0f,  1.0f,  1.0f,  1.0f,  0.0f,  0.0f, //7
         1.0f, -1.0f,  1.0f,  1.0f,  0.0f,  0.0f, //5
         1.0f, -1.0f, -1.0f,  1.0f,  0.0f,  0.0f, //1

         1.0f,  1.0f, -1.0f,  0.0f,  1.0f,  0.0f, //3
        -1.0f,  1.0f, -1.0f,  0.0f,  1.0f,  0.0f, //2
        -1.0f,  1.0f,  1.0f,  0.0f,  1.0f,  0.0f, //6
        -1.0f,  1.0f,  1.0f,  0.0f,  1.0f,  0.0f, //6
         1.0f,  1.0f,  1.0f,  0.0f,  1.0f,  0.0f, //7
         1.0f,  1.0f, -1.0f,  0.0f,  1.0f,  0.0f, //3

        -1.0f,  1.0f, -1.0f, -1.0f,  0.0f,  0.0f, //2
        -1.0f, -1.0f, -1.0f, -1.0f,  0.0f,  0.0f, //0
        -1.0f, -1.0f,  1.0f, -1.0f,  0.0f,  0.0f, //4
        -1.0f, -1.0f,  1.0f, -1.0f,  0.0f,  0.0f, //4
        -1.0f,  1.0f,  1.0f, -1.0f,  0.0f,  0.0f, //6
        -1.0f,  1.0f, -1.0f, -1.0f,  0.0f,  0.0f, //2

    };

    /*Mat4 m;
    m.identity();
    m.rotate(ang, Vec3{0.0f, 1.0f, 0.0f});
    m.translate({0.0f, 0.0f, -4.0f});
    r3d.set_model_view(m);*/
    r3d.set_model_view(Mat4::translation(Vec3(0.0f, 0.0f, -4.0f)) * Mat4::rotation(ang, Vec3{0.0f, 1.0f, 0.0f}) * Mat4::rotation(ang2, Vec3{1.0f, 0.0f, 0.0f}));

    r3d.clear();
    r3d.set_vertex_stride(6);

    auto start = now_us();

    r3d.set_vertex_shader(litShader);
    r3d.draw(6 * 6, vertices_with_normals);

    auto end = now_us();

    ang += 0.5f;
    ang2 -= 0.3f;

    screen.pen = {255, 0, 0};
    screen.text(std::to_string(us_diff(start, end)), minimal_font, {0, 0});
}