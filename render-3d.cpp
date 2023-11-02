#include <cstring>

#ifdef PICO_BUILD
#include "hardware/interp.h"
#include "pico/multicore.h"
#include "pico/sync.h"
#endif

#include "engine/engine.hpp"
#include "engine/fast_code.hpp"
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

#ifdef PICO_MULTICORE
auto_init_mutex(blit_mutex);

static void core1_entry()
{
    // get renderer
    auto render3d = reinterpret_cast<Render3D *>(multicore_fifo_pop_blocking());

    // render the other half
    render3d->rasterise();

    // done
    multicore_fifo_push_blocking(0);
}
#endif

Render3D::Render3D() : tile_surf(reinterpret_cast<uint8_t *>(tile_colour_buffer), PixelFormat::BGR555, {tile_width, tile_height})
{}

void Render3D::draw(int count, const uint8_t *ptr)
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

        // back-face culling
        auto ab_x = int32_t(trans[1].x) - int32_t(trans[0].x);
        auto ab_y = int32_t(trans[1].y) - int32_t(trans[0].y);
        auto ac_x = int32_t(trans[2].x) - int32_t(trans[0].x);
        auto ac_y = int32_t(trans[2].y) - int32_t(trans[0].y);

        int32_t z = ab_x * ac_y - ab_y * ac_x;

        if(z < 0)
            continue;

        // TODO: clipping
        transformed_vertex_ptr += 3;
    }
}

const Mat4 &Render3D::get_model_view() const
{
    return model_view;
}

const FixedMat4<> &Render3D::get_fixed_model_view() const
{
    return fixed_model_view;
}

void Render3D::set_model_view(Mat4 m)
{
    model_view = m;
    fixed_model_view = m;
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

int Render3D::get_transformed_vertex_count() const
{
    if(!transformed_vertex_ptr)
        return 0;

    return transformed_vertex_ptr - transformed_vertices;
}

void Render3D::rasterise()
{
    if(!transformed_vertex_ptr)
        return;

#ifdef PICO_MULTICORE
    // launch the other core if needed
    auto core_num = get_core_num();
    if(core_num == 0)
    {
        multicore_launch_core1(core1_entry);
        multicore_fifo_push_blocking(reinterpret_cast<uintptr_t>(this));
    }
#endif

#ifdef PICO_INTERP
    // setup interpolators
    auto config = interp_default_config();

    // start/end x in fill_triangle
    interp_config_set_shift(&config, 16);     // shift out 16 fractional bits
    interp_config_set_mask(&config, 0, 15);   // mask out the nothing in the upper 16 bits
    interp_config_set_signed(&config, true);  // ... then sign extend (which is why we need the mask)
    interp_config_set_add_raw(&config, true); // skip everything for the value stored to the accumulator

    interp_set_config(interp0, 0, &config);
    interp_set_config(interp0, 1, &config);
#endif

    uint16_t clear_col = pack_colour({127, 127, 127});

    uint32_t clear_col32 = clear_col | clear_col << 16;
    uint32_t clear_depth32 = 0xFFFFFFFF;

    auto col_buf = tile_colour_buffer;
    auto depth_buf = tile_depth_buffer;
    const auto tile_buf_size = sizeof(tile_colour_buffer) / num_tile_bufs;

#ifdef PICO_MULTICORE
    // offset for per-core tile buffers
    col_buf += core_num * tile_width * tile_height;
    depth_buf += core_num * tile_width * tile_height;
#endif

    // rasterise triangles for each screen tile
    for(int y = 0; y < screen.bounds.h; y += tile_height)
    {
        for(int x = 0; x < screen.bounds.w; x += tile_width)
        {
#ifdef PICO_MULTICORE
            // split tiles between cores
            if((((x / tile_width) + (y / tile_height)) & 1) != core_num)
                continue;
#endif

            // clear
            // TODO: load/store for multi-pass? (UNLIMITED PO... triangles)
            auto tile_ptr32 = reinterpret_cast<uint32_t *>(col_buf);
            for(size_t i = 0; i < tile_buf_size / 4; i++)
                *tile_ptr32++ = clear_col32;

            tile_ptr32 = reinterpret_cast<uint32_t *>(depth_buf);
            for(size_t i = 0; i < tile_buf_size / 4; i++)
                *tile_ptr32++ = clear_depth32;
    
            // now the triangles
            for(auto ptr = transformed_vertices; ptr != transformed_vertex_ptr; ptr += 3)
                fill_triangle(ptr, {x, y});

            // store colour tile
            if(screen.format == PixelFormat::BGR555)
            {
                // assume picovision, which has a 555 -> 555 blit
#ifdef PICO_MULTICORE
                // blitting on both cores at once would blow up
                mutex_enter_blocking(&blit_mutex);
#endif

                tile_surf.data = reinterpret_cast<uint8_t *>(col_buf);
                screen.blit(&tile_surf, {0, 0, tile_width, tile_height}, {x, y});

#ifdef PICO_MULTICORE
                mutex_exit(&blit_mutex);
#endif
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
                        auto pen = unpack_colour(col_buf[tx + ty * tile_width]);

                        screen.pbf(&pen, &screen, offset + tx, 1);
                    }
                }
            }
        }
    }

#ifdef PICO_MULTICORE
    if(core_num == 0)
    {
        // wait for core1 and reset
        multicore_fifo_pop_blocking();
        multicore_reset_core1();
    }
    else
        return; // don't reset vertex ptr on core1
#endif

    transformed_vertex_ptr = nullptr;
}

void Render3D::transform_vertex(VertexOutData &pos)
{
    // perspective
    pos.w = Fixed32<>(1) / pos.w;
    pos.x *= pos.w;
    pos.y *= pos.w;
    pos.z *= pos.w;

    Rect viewport{0, 0, screen.bounds.w, screen.bounds.h};

    // viewport
    pos.x = Fixed32<>(viewport.x) + (pos.x + 1) * (viewport.w / 2);
    pos.y = Fixed32<>(viewport.y) + (pos.y + 1) * (viewport.h / 2);
    pos.z = (pos.z + 1) * Fixed32<>(32767.5f);
}

void blit_fast_code(Render3D::fill_triangle)(VertexOutData *data, blit::Point tile_pos)
{
    struct IntVec3
    {
        int32_t x, y, z;

        IntVec3 operator -(const IntVec3 &v)
        {
            return {x - v.x, y - v.y, z - v.z};
        }
    };

    IntVec3 p0{int32_t(data[0].x) - tile_pos.x, int32_t(data[0].y) - tile_pos.y, int32_t(UFixed32<>(data[0].z))};
    IntVec3 p1{int32_t(data[1].x) - tile_pos.x, int32_t(data[1].y) - tile_pos.y, int32_t(UFixed32<>(data[1].z))};
    IntVec3 p2{int32_t(data[2].x) - tile_pos.x, int32_t(data[2].y) - tile_pos.y, int32_t(UFixed32<>(data[2].z))};

    // check if outside tile
    auto get_outside_sides = [&](IntVec3 &p)
    {
        int ret = 0;
        if(p.x < 0)
            ret = 1;
        else if(p.x >= tile_width)
            ret = 2;

        if(p.y < 0)
            ret += 4;
        else if(p.y >= tile_height)
            ret += 8;

        return ret;
    };

    // all points outside on the same side
    if(get_outside_sides(p0) & get_outside_sides(p1) & get_outside_sides(p2))
        return;

    Pen cols[3]{
        {data[0].r, data[0].g, data[0].b},
        {data[1].r, data[1].g, data[1].b},
        {data[2].r, data[2].g, data[2].b}
    };

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

    auto col2M0 = cols[2] - cols[0];

    auto start_x_step = Fixed32<>(p2M0.x) / p2M0.y;
    auto start_x = Fixed32<>(p0.x);

    auto y_step1 = Fixed32<>(1) / p2M0.y;
    Fixed32<> y_frac1 = 0;

    if(p1M0.y && p1.y > 0)
    {
        auto col1M0 = cols[1] - cols[0];

        auto end_x_step = Fixed32<>(p1M0.x) / p1M0.y;
        auto end_x = start_x;

        auto y_step2 = Fixed32<>(1) / p1M0.y;

        Fixed32<> y_frac2 = 0;

        // clamp to tile
        int y_start = p0.y;
        int y_end = std::min(p1.y, int32_t(tile_height));

        if(y_start < 0)
        {
            start_x += start_x_step * -y_start;
            end_x += end_x_step * -y_start;

            y_frac1 += y_step1 * -y_start;
            y_frac2 += y_step2 * -y_start;
            y_start = 0;
        }

#ifdef PICO_INTERP
        interp0->accum[0] = start_x.raw();
        interp0->base[0] = start_x_step.raw();
        interp0->accum[1] = end_x.raw();
        interp0->base[1] = end_x_step.raw();

        for(int y = y_start; y < y_end; y++, y_frac1 += y_step1, y_frac2 += y_step2)
        {
            int32_t start_x = interp0->add_raw[0];
            int32_t end_x = interp0->add_raw[1];
            interp0->pop[0];
#else
        for(int y = y_start; y < y_end; y++, start_x += start_x_step, end_x += end_x_step, y_frac1 += y_step1, y_frac2 += y_step2)
        {
#endif
            if(int32_t(start_x) == int32_t(end_x))
                continue;

            auto start_z = p0.z + int32_t(y_frac1 * p2M0.z);
            auto end_z   = p0.z + int32_t(y_frac2 * p1M0.z);

            auto start_col = cols[0] + col2M0 * y_frac1;
            auto end_col = cols[0] + col1M0 * y_frac2;

            gradient_h_line(int32_t(start_x), int32_t(end_x), start_z, end_z, y, start_col, end_col);
        }

#ifdef PICO_INTERP
        // read back out
        start_x = Fixed32<>::from_raw(interp0->accum[0]);
#endif
    }
    else if(p1M0.y)
    {
        // still need to adjust these for the second part
        start_x += start_x_step * p1M0.y;
        y_frac1 += y_step1 * p1M0.y;
    }

    if(p2M1.y && p1.y < tile_height)
    {
        auto col2M1 = cols[2] - cols[1];

        auto end_x_step = Fixed32<>(p2M1.x) / p2M1.y;
        auto end_x = Fixed32<>(p1.x);

        auto y_step2 = Fixed32<>(1) / p2M1.y;
        Fixed32<> y_frac2 = 0;

        // clamp to tile
        int y_start = p1.y;
        int y_end = std::min(p2.y, int32_t(tile_height));

        if(y_start < 0)
        {
            start_x += start_x_step * -y_start;
            end_x += end_x_step * -y_start;

            y_frac1 += y_step1 * -y_start;
            y_frac2 += y_step2 * -y_start;
            y_start = 0;
        }

#ifdef PICO_INTERP
        interp0->accum[0] = start_x.raw();
        interp0->base[0] = start_x_step.raw();
        interp0->accum[1] = end_x.raw();
        interp0->base[1] = end_x_step.raw();

        for(int y = y_start; y < y_end; y++, y_frac1 += y_step1, y_frac2 += y_step2)
        {
            int32_t start_x = interp0->add_raw[0];
            int32_t end_x = interp0->add_raw[1];
            interp0->pop[0];
#else
        for(int y = y_start; y < y_end; y++, start_x += start_x_step, end_x += end_x_step, y_frac1 += y_step1, y_frac2 += y_step2)
        {
#endif
            if(int32_t(start_x) == int32_t(end_x))
                continue;

            auto start_z = p0.z + int32_t(y_frac1 * p2M0.z);
            auto end_z   = p1.z + int32_t(y_frac2 * p2M1.z);

            auto start_col = cols[0] + col2M0 * y_frac1;
            auto end_col = cols[1] + col2M1 * y_frac2;

            gradient_h_line(int32_t(start_x), int32_t(end_x), start_z, end_z, y, start_col, end_col);
        }
    }
}

void blit_fast_code(Render3D::gradient_h_line)(int x1, int x2, uint16_t z1, uint16_t z2, int y, Pen col1, Pen col2)
{
    if(x1 > x2)
    {
        std::swap(x1, x2);
        std::swap(z1, z2);
        std::swap(col1, col2);
    }

    // the triangle may be in the tile, but this line is definitely not
    if(x1 > tile_width || x2 < 0)
        return;

    auto x_scale = Fixed32<>(1) / (x2 - x1);

    // depth
    int z_diff = z2 - z1;
    auto z_step = Fixed32<15>(x_scale) * z_diff;
    auto z = Fixed32<15>(z1);

    // colour
    auto col_diff = col2 - col1;

    auto r_step = x_scale * col_diff.r;
    auto g_step = x_scale * col_diff.g;
    auto b_step = x_scale * col_diff.b;

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

#ifdef PICO_MULTICORE
    auto core_num = get_core_num();
    col_ptr += core_num * tile_width * tile_height;
    depth_ptr += core_num * tile_width * tile_height;
#endif

    auto end_ptr = col_ptr + (x2 - x1);

    for(; col_ptr < end_ptr; col_ptr++, depth_ptr++, z += z_step, r += r_step, g += g_step, b += b_step)
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