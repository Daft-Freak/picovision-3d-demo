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
        VertexOutData trans[3];

        for(int j = 0; j < 3; j++)
        {
            vertex_shader(ptr + (i + j) * vertex_stride, trans + j, *this);
            transform_vertex(trans[j]);
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

void Render3D::transform_vertex(VertexOutData &pos)
{
    // perspective
    pos.x /= pos.w;
    pos.y /= pos.w;
    pos.z /= pos.w;

    Rect viewport{0, 0, 320, 240};

    // viewport
    pos.x = viewport.x + (pos.x * 0.5f + 0.5f) * viewport.w;
    pos.y = viewport.y + (pos.y * 0.5f + 0.5f) * viewport.h;
    pos.z = (pos.z + 1.0f) * 32767.5f;
}

void Render3D::fill_triangle(VertexOutData *data)
{
    Vec3 cols[3]{
        {data[0].r, data[0].g, data[0].b},
        {data[1].r, data[1].g, data[1].b},
        {data[2].r, data[2].g, data[2].b}
    };

    // sort points
    Vec3 p0{std::floor(data[0].x), float(data[0].y), std::floor(data[0].z)};
    Vec3 p1{std::floor(data[1].x), float(data[1].y), std::floor(data[1].z)};
    Vec3 p2{std::floor(data[2].x), float(data[2].y), std::floor(data[2].z)};

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
