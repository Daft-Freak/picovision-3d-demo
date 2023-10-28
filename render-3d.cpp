#include <cstring>

#include "engine/engine.hpp"
#include "types/vec3.hpp"

#include "render-3d.hpp"

using namespace blit;

void Render3D::clear()
{
    //memset(framebuffer, 0, 320 * 240 * 3);
    screen.pen = {127, 127, 127};
    screen.clear();
    memset(depth_buffer, 0xFF, 320 * 240 * 2);
}

void Render3D::draw(int count, const float *ptr)
{
    // triangles
    for(int i = 0; i < count; i += 3)
    {
        int out_stride = 7; //xyzw, rgb
        float trans[out_stride * 3];

        for(int j = 0; j < 3; j++)
        {
            vertex_shader(ptr + (i + j) * vertex_stride, trans + out_stride * j, *this);
            transform_vertex(trans + out_stride * j);
        }

        fill_triangle(trans);
    }
}

const Mat4 &Render3D::get_model_view() const
{
    return model_view;
}

void Render3D::set_model_view(Mat4 m)
{
    model_view = m;
    //mvp = model_view * projection;
    mvp = projection * model_view;
}

const Mat4 &Render3D::get_projection() const
{
    return projection;
}

void Render3D::set_projection(Mat4 m)
{
    projection = m;
    //mvp = model_view * projection;
    mvp = projection * model_view;
}

const FixedMat4<> &Render3D::get_model_view_projection() const
{
    return mvp;
}

void Render3D::set_vertex_stride(int stride)
{
    vertex_stride = stride;
}

void Render3D::set_vertex_shader(VertexShaderFunc shader)
{
    vertex_shader = shader;
}

void Render3D::transform_vertex(float *pos)
{
    // perspective
    pos[0] /= pos[3];
    pos[1] /= pos[3];
    pos[2] /= pos[3];

    Rect viewport{0, 0, 320, 240};

    // viewport
    pos[0] = viewport.x + (pos[0] * 0.5f + 0.5f) * viewport.w;
    pos[1] = viewport.y + (pos[1] * 0.5f + 0.5f) * viewport.h;
    pos[2] = (pos[2] + 1.0f) * 32767.5f;
}

void Render3D::fill_triangle(float *data)
{
    // data = 7 floats per vertex
    int stride = 7;

    Vec3 cols[3]{
        {data[stride * 0 + 4], data[stride * 0 + 5], data[stride * 0 + 6]},
        {data[stride * 1 + 4], data[stride * 1 + 5], data[stride * 1 + 6]},
        {data[stride * 2 + 4], data[stride * 2 + 5], data[stride * 2 + 6]}
    };

    // sort points
    Vec3 p0{std::floor(data[stride * 0 + 0]), std::floor(data[stride * 0 + 1]), std::floor(data[stride * 0 + 2])};
    Vec3 p1{std::floor(data[stride * 1 + 0]), std::floor(data[stride * 1 + 1]), std::floor(data[stride * 1 + 2])};
    Vec3 p2{std::floor(data[stride * 2 + 0]), std::floor(data[stride * 2 + 1]), std::floor(data[stride * 2 + 2])};

    if(p0.y > p2.y)
    {
        std::swap(p0, p2);
        std::swap(cols[0], cols[2]);
    }
    if(p0.y > p1.y)
    {
        std::swap(p0, p1);
        std::swap(cols[0], cols[1]);
    }
    if(p1.y > p2.y)
    {
        std::swap(p1, p2);
        std::swap(cols[1], cols[2]);
    }

    auto p1M0 = p1 - p0;
    auto p2M1 = p2 - p1;

    auto col1M0 = cols[1] - cols[0];
    auto col2M0 = cols[2] - cols[0];
    auto col2M1 = cols[2] - cols[1];

    if(p1M0.y)
    {
        for(int y = 0; y <= p1M0.y; y++)
        {
            float yD1 = y / (p2.y - p0.y);
            float yD2 = y / p1M0.y;
            auto startX = p0.x + (p2.x - p0.x) * yD1;
            auto endX   = p0.x + p1M0.x * yD2;

            if(endX - startX == 0)
                continue;

            auto startZ = p0.z + (p2.z - p0.z) * yD1;
            auto endZ   = p0.z + p1M0.z * yD2;

            gradient_h_line(startX, endX, startZ, endZ, y + p0.y, cols[0] + col2M0 * yD1, cols[0] + col1M0 * yD2);
        }

    }

    if(p2M1.y)
    {
        for(int y = 0; y <= p2M1.y; y++)
        {
            float yD1 = (y + p1M0.y) / (p2.y - p0.y);
            float yD2 = y / p2M1.y;
            auto startX = p0.x + (p2.x - p0.x) * yD1;
            auto endX   = p1.x + p2M1.x * yD2;

            if(endX - startX == 0)
                continue;

            auto startZ = p0.z + (p2.z - p0.z) * yD1;
            auto endZ   = p1.z + p2M1.z * yD2;

            gradient_h_line(startX, endX, startZ, endZ, y + p1.y, cols[0] + col2M0 * yD1, cols[1] + col2M1 * yD2);
        }
    }
    
}

void Render3D::gradient_h_line(int x1, int x2, float z1, float z2, int y, const Vec3 &col1, const Vec3 &col2)
{
    if(y < 0 || y >= 240)
        return;

    int sign = x1 > x2 ? -1 : 1;

    for(int x = 0; x != x2 - x1; x += sign)
    {
        if(x1 + x < 0 || x1 + x >= 320)
            continue;

        float xD = static_cast<float>(x) / (x2 - x1);

        auto z = z1 + (z2 - z1) * xD;

        if(z > depth_buffer[x1 + x + y * 320])
            continue;

        auto col = col1 + (col2 - col1) * xD;

        screen.pen = {col.x, col.y, col.z};
        screen.pixel({x1 + x, y});

        depth_buffer[x1 + x + y * 320] = z;
    }
}
