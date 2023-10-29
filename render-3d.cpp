#include <cstring>

#include "engine/engine.hpp"
#include "types/vec3.hpp"

#include "render-3d.hpp"

using namespace blit;

// helper for blending pens
struct PenDelta
{
    int32_t r;
    int32_t g;
    int32_t b;
    int32_t a;

    PenDelta operator *(Fixed32<> s)
    {
        return {int32_t(s * r), int32_t(s * g), int32_t(s * b), int32_t(s * a)};
    }
};

PenDelta operator -(Pen a, Pen b)
{
    return {a.r - b.r, a.b - b.b, a.g - b.g, a.a - b.a};
}

Pen operator +(Pen p, PenDelta d)
{
    return {uint8_t(p.r + d.r), uint8_t(p.g + d.g), uint8_t(p.b + d.b), uint8_t(p.a + d.a)};
}

Render3D::Render3D() : tile_surf(reinterpret_cast<uint8_t *>(tile_colour_buffer), PixelFormat::BGR555, {tile_width, tile_height})
{}

void Render3D::draw(int count, const float *ptr)
{
    if(!transformed_vertex_ptr)
        transformed_vertex_ptr = transformed_vertices;

    // triangles
    for(int i = 0; i < count; i += 3)
    {
        // vertex limit reached
        if(transformed_vertex_ptr + 3 >= transformed_vertices + std::size(transformed_vertices))
            break;

        auto trans = transformed_vertex_ptr;

        for(int j = 0; j < 3; j++)
        {
            vertex_shader(ptr + (i + j) * vertex_stride, trans + j, *this);
            transform_vertex(trans[j]);
        }

        // TODO: clipping
        transformed_vertex_ptr += 3;
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

void Render3D::rasterise()
{
    if(!transformed_vertex_ptr)
        return;

    uint16_t clear_col = pack_colour({127, 127, 127});

    uint32_t clear_col32 = clear_col | clear_col << 16;
    uint32_t clear_depth32 = 0xFFFFFFFF;

    // rasterise triangles for each screen tile
    for(int y = 0; y < screen.bounds.h; y += tile_height)
    {
        for(int x = 0; x < screen.bounds.w; x += tile_width)
        {
            // clear
            // TODO: load/store for multi-pass? (UNLIMITED PO... triangles)
            auto tile_ptr32 = reinterpret_cast<uint32_t *>(tile_colour_buffer);
            for(size_t i = 0; i < sizeof(tile_colour_buffer) / 4; i++)
                *tile_ptr32++ = clear_col32;

            tile_ptr32 = reinterpret_cast<uint32_t *>(tile_depth_buffer);
            for(size_t i = 0; i < sizeof(tile_depth_buffer) / 4; i++)
                *tile_ptr32++ = clear_depth32;
    
            // now the triangles
            for(auto ptr = transformed_vertices; ptr != transformed_vertex_ptr; ptr += 3)
                fill_triangle(ptr, {x, y});

            // store colour tile
            if(screen.format == PixelFormat::BGR555)
            {
                // assume picovision, which has a 555 -> 555 blit
                screen.blit(&tile_surf, {0, 0, tile_width, tile_height}, {x, y});
            }
            else
            {
                // fallback
                // TODO: optimise based on screen format
                for(int ty = 0; ty < tile_height; ty++)
                {
                    auto offset = screen.offset(x, y + ty);
                    for(int tx = 0; tx < tile_width; tx++)
                    {
                        auto pen = unpack_colour(tile_colour_buffer[tx + ty * tile_width]);

                        screen.pbf(&pen, &screen, offset + tx, 1);
                    }
                }
            }
        }
    }


    transformed_vertex_ptr = nullptr;
}

void Render3D::transform_vertex(VertexOutData &pos)
{
    // perspective
    pos.x /= pos.w;
    pos.y /= pos.w;
    pos.z /= pos.w;

    Rect viewport{0, 0, screen.bounds.w, screen.bounds.h};

    // viewport
    pos.x = Fixed32<>(viewport.x) + (pos.x * Fixed32<>(0.5f) + 0.5f) * viewport.w;
    pos.y = Fixed32<>(viewport.y) + (pos.y * Fixed32<>(0.5f) + 0.5f) * viewport.h;
    pos.z = (pos.z + 1.0f) * Fixed32<>(32767.5f);
}

void Render3D::fill_triangle(VertexOutData *data, blit::Point tile_pos)
{
    Pen cols[3]{
        {data[0].r, data[0].g, data[0].b},
        {data[1].r, data[1].g, data[1].b},
        {data[2].r, data[2].g, data[2].b}
    };
    
    struct IntVec3
    {
        int32_t x, y, z;

        IntVec3 operator -(const IntVec3 &v)
        {
            return {x - v.x, y - v.y, z - v.z};
        }
    };

    IntVec3 p0{int32_t(data[0].x), int32_t(data[0].y), int32_t(UFixed32<>(data[0].z))};
    IntVec3 p1{int32_t(data[1].x), int32_t(data[1].y), int32_t(UFixed32<>(data[1].z))};
    IntVec3 p2{int32_t(data[2].x), int32_t(data[2].y), int32_t(UFixed32<>(data[2].z))};

    // back-face culling
    auto ab = p1 - p0;
    auto ac = p2 - p0;

    int32_t z = ab.x * ac.y - ab.y * ac.x;

    if(z < 0)
        return;

    // sort points
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
    auto p2M0 = p2 - p0;
    auto p2M1 = p2 - p1;

    auto col1M0 = cols[1] - cols[0];
    auto col2M0 = cols[2] - cols[0];
    auto col2M1 = cols[2] - cols[1];

    if(p1M0.y)
    {
        for(int y = 0; y <= p1M0.y; y++)
        {
            if(y + p0.y < tile_pos.y)
                continue;

            if(y + p0.y >= tile_pos.y + tile_height)
                break;

            auto yD1 = Fixed32<>(y) / (p2M0.y);
            auto yD2 = Fixed32<>(y) / p1M0.y;
            auto startX = int32_t(Fixed32<>(p0.x) + yD1 * p2M0.x);
            auto endX   = int32_t(Fixed32<>(p0.x) + yD2 * p1M0.x);

            if(endX - startX == 0)
                continue;

            auto startZ = p0.z + int32_t(yD1 * p2M0.z);
            auto endZ   = p0.z + int32_t(yD2 * p1M0.z);

            gradient_h_line(startX - tile_pos.x, endX - tile_pos.x, startZ, endZ, y + p0.y - tile_pos.y, cols[0] + col2M0 * yD1, cols[0] + col1M0 * yD2);
        }

    }

    if(p2M1.y)
    {
        for(int y = 0; y <= p2M1.y; y++)
        {
            if(y + p1.y < tile_pos.y)
                continue;
                
            if(y + p1.y >= tile_pos.y + tile_height)
                break;

            auto yD1 = Fixed32<>(y + p1M0.y) / p2M0.y;
            auto yD2 = Fixed32<>(y) / p2M1.y;
            auto startX = int32_t(Fixed32<>(p0.x) + yD1 * p2M0.x);
            auto endX   = int32_t(Fixed32<>(p1.x) + yD2 * p2M1.x);

            if(endX - startX == 0)
                continue;

            auto startZ = p0.z + int32_t(yD1 * p2M0.z);
            auto endZ   = p1.z + int32_t(yD2 * p2M1.z);

            gradient_h_line(startX - tile_pos.x, endX - tile_pos.x, startZ, endZ, y + p1.y - tile_pos.y, cols[0] + col2M0 * yD1, cols[1] + col2M1 * yD2);
        }
    }
    
}

void Render3D::gradient_h_line(int x1, int x2, uint16_t z1, uint16_t z2, int y, Pen col1, Pen col2)
{
    if(x1 > x2)
    {
        std::swap(x1, x2);
        std::swap(z1, z2);
        std::swap(col1, col2);
    }

    for(int x = x1; x < x2; x++)
    {
        if(x < 0 || x >= tile_width)
            continue;

        auto xD = Fixed32<>(x - x1) / (x2 - x1);

        auto z = z1 + int32_t(xD * (z2 - z1));

        if(z > tile_depth_buffer[x + y * tile_width])
            continue;

        auto col = col1 + (col2 - col1) * xD;

        tile_colour_buffer[x + y * tile_width] = pack_colour(col);

        tile_depth_buffer[x + y * tile_width] = z;
    }
}

uint16_t Render3D::pack_colour(Pen p)
{
#ifdef BLIT_BOARD_PIMORONI_PICOVISION
    // 555
    return (p.b >> 3) | ((p.g >> 3) << 5) | ((p.r >> 3) << 10);
#else
    // 565
    return (p.r >> 3) | ((p.g >> 2) << 5) | ((p.b >> 3) << 11);
#endif
}

blit::Pen Render3D::unpack_colour(uint16_t c)
{
    // not used for picovision

    // 565
    uint8_t r = c & 0x1F;
    uint8_t g = (c >> 5) & 0x3F;
    uint8_t b = c >> 11;

    r = r << 3 | r >> 2;
    g = g << 2 | g >> 4;
    b = b << 3 | b >> 2;

    return {r, g, b};
}