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
    return {a.r - b.r, a.g - b.g, a.b - b.b, a.a - b.a};
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

    // check if outside tile
    auto get_outside_sides = [&](IntVec3 &p)
    {
        int ret = 0;
        if(p.x < tile_pos.x)
            ret = 1;
        else if(p.x >= tile_pos.x + tile_width)
            ret = 2;

        if(p.y < tile_pos.y)
            ret += 4;
        else if(p.y >= tile_pos.y + tile_height)
            ret += 8;

        return ret;
    };

    // all points outside on the same side
    if(get_outside_sides(p0) & get_outside_sides(p1) & get_outside_sides(p2))
        return;

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
        auto start_x_step = Fixed32<>(p2M0.x) / p2M0.y;
        auto end_x_step = Fixed32<>(p1M0.x) / p1M0.y;
        auto start_x = Fixed32<>(p0.x);
        auto end_x = start_x;

        auto y_step1 = Fixed32<>(1) / p2M0.y;
        auto y_step2 = Fixed32<>(1) / p1M0.y;
        Fixed32<> y_frac1 = 0;
        Fixed32<> y_frac2 = 0;

        // clamp to tile
        int y_start = p0.y;
        int y_end = std::min(p1.y, tile_pos.y + tile_height);

        if(y_start < tile_pos.y)
        {
            start_x += start_x_step * (tile_pos.y - y_start);
            end_x += end_x_step * (tile_pos.y - y_start);

            y_frac1 += y_step1 * (tile_pos.y - y_start);
            y_frac2 += y_step2 * (tile_pos.y - y_start);
            y_start = tile_pos.y;
        }

        for(int y = y_start; y <= y_end; y++, start_x += start_x_step, end_x += end_x_step, y_frac1 += y_step1, y_frac2 += y_step2)
        {
            if(int32_t(start_x) == int32_t(end_x))
                continue;

            auto start_z = p0.z + int32_t(y_frac1 * p2M0.z);
            auto end_z   = p0.z + int32_t(y_frac2 * p1M0.z);

            auto start_col = cols[0] + col2M0 * y_frac1;
            auto end_col = cols[0] + col1M0 * y_frac2;

            gradient_h_line(int32_t(start_x) - tile_pos.x, int32_t(end_x) - tile_pos.x, start_z, end_z, y - tile_pos.y, start_col, end_col);
        }
    }

    if(p2M1.y)
    {
        auto start_x_step = Fixed32<>(p2M0.x) / p2M0.y;
        auto end_x_step = Fixed32<>(p2M1.x) / p2M1.y;
        auto start_x = Fixed32<>(p0.x) + start_x_step * p1M0.y;
        auto end_x = Fixed32<>(p1.x);

        auto y_step1 = Fixed32<>(1) / p2M0.y;
        auto y_step2 = Fixed32<>(1) / p2M1.y;
        Fixed32<> y_frac1 = y_step1 * p1M0.y;
        Fixed32<> y_frac2 = 0;

        // clamp to tile
        int y_start = p1.y;
        int y_end = std::min(p2.y, tile_pos.y + tile_height);

        if(y_start < tile_pos.y)
        {
            start_x += start_x_step * (tile_pos.y - y_start);
            end_x += end_x_step * (tile_pos.y - y_start);

            y_frac1 += y_step1 * (tile_pos.y - y_start);
            y_frac2 += y_step2 * (tile_pos.y - y_start);
            y_start = tile_pos.y;
        }

        for(int y = y_start; y <= y_end; y++, start_x += start_x_step, end_x += end_x_step, y_frac1 += y_step1, y_frac2 += y_step2)
        {
            if(int32_t(start_x) == int32_t(end_x))
                continue;

            auto start_z = p0.z + int32_t(y_frac1 * p2M0.z);
            auto end_z   = p1.z + int32_t(y_frac2 * p2M1.z);

            auto start_col = cols[0] + col2M0 * y_frac1;
            auto end_col = cols[1] + col2M1 * y_frac2;

            gradient_h_line(int32_t(start_x) - tile_pos.x, int32_t(end_x) - tile_pos.x, start_z, end_z, y - tile_pos.y, start_col, end_col);
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

    // depth
    int z_diff = z2 - z1;
    auto z_step = Fixed32<15>(z_diff) / (x2 - x1);
    auto z = Fixed32<15>(z1);

    // colour
    auto col_diff = col2 - col1;
    // could do some packing here...
    auto r_step = Fixed32<>(col_diff.r) / (x2 - x1);
    auto g_step = Fixed32<>(col_diff.g) / (x2 - x1);
    auto b_step = Fixed32<>(col_diff.b) / (x2 - x1);

    auto r = Fixed32<>(col1.r);
    auto g = Fixed32<>(col1.g);
    auto b = Fixed32<>(col1.b);

    // clamp x
    if(x2 > tile_width)
        x2 = tile_width;

    if(x1 < 0)
    {
        z += z_step * -x1;
        r += r_step * -x1;
        g += g_step * -x1;
        b += b_step * -x1;
        x1 = 0;
    }

    auto col_ptr = tile_colour_buffer + x1 + y * tile_width;
    auto depth_ptr = tile_depth_buffer + x1 + y * tile_width;

    for(int x = x1; x < x2; x++, col_ptr++, depth_ptr++, z += z_step, r += r_step, g += g_step, b += b_step)
    {
        if(int32_t(z) > *depth_ptr)
            continue;

        Pen col{uint8_t(r), uint8_t(g), uint8_t(b)};

        *col_ptr = pack_colour(col);
        *depth_ptr = int32_t(z);
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