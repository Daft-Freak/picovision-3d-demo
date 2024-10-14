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

// integer vector helper used by triangle drawing
struct IntVec3
{
    int32_t x, y, z;

    IntVec3 operator -(const IntVec3 &v)
    {
        return {x - v.x, y - v.y, z - v.z};
    }
};


#if THR3E_PICO_MULTICORE
auto_init_mutex(blit_mutex);

enum class Core1Job
{
    NoOp,
    Draw,
    Rasterise,
};

static void core1_entry()
{
    while(true)
    {
        // get renderer
        auto render3d = reinterpret_cast<Render3D *>(multicore_fifo_pop_blocking());
        auto job_type = Core1Job(multicore_fifo_pop_blocking());

        switch(job_type)
        {
            case Core1Job::NoOp:
                multicore_fifo_push_blocking(0);
                break;

            case Core1Job::Draw:
            {
                int count = multicore_fifo_pop_blocking();
                auto ptr = reinterpret_cast<const uint8_t *>(multicore_fifo_pop_blocking());
                render3d->draw(count, ptr);
                break;
            }

            case Core1Job::Rasterise:
                // render the other half
                render3d->rasterise();
                break;
        }
    }
}
#endif

void fixed32_mvp_pos_shader(const uint8_t *in, Render3D::VertexOutData *out, const Render3D &r)
{
    auto pos = reinterpret_cast<const Fixed32<> *>(in);

    auto &mat = r.get_model_view_projection();

    out->x = (pos[0] * mat.v00) + (pos[1] * mat.v01) + (pos[2] * mat.v02) + mat.v03;
    out->y = (pos[0] * mat.v10) + (pos[1] * mat.v11) + (pos[2] * mat.v12) + mat.v13;
    out->z = (pos[0] * mat.v20) + (pos[1] * mat.v21) + (pos[2] * mat.v22) + mat.v23;
    out->w = (pos[0] * mat.v30) + (pos[1] * mat.v31) + (pos[2] * mat.v32) + mat.v33;
}

Render3D::Render3D() : tile_surf(reinterpret_cast<uint8_t *>(tile_colour_buffer), PixelFormat::BGR555, {tile_width, tile_height})
{
    for(int i = 0; i < max_textures; i++)
        textures[i] = nullptr;

    clear_colour = pack_colour({127, 127, 127});

#if THR3E_PICO_MULTICORE
    // we own core1 now
    // theoretically this could handle multiple instances
    multicore_launch_core1(core1_entry);
#endif
}

void Render3D::draw(int count, const uint8_t *ptr)
{
    if(!transformed_vertex_ptr)
        transformed_vertex_ptr = transformed_vertices;

    auto trans = transformed_vertex_ptr;

#if THR3E_PICO_MULTICORE
    auto core_num = get_core_num();
    if(core_num == 0)
    {
        multicore_fifo_push_blocking(reinterpret_cast<uintptr_t>(this));

        if(count > 3)
        {
            multicore_fifo_push_blocking(uint32_t(Core1Job::Draw));
            multicore_fifo_push_blocking(count);
            multicore_fifo_push_blocking(reinterpret_cast<uintptr_t>(ptr));
        }
        else // push no-op job for the wait at the end
            multicore_fifo_push_blocking(uint32_t(Core1Job::NoOp));
    }
    else
    {
        // offset by one triangle
        ptr += 3 * vertex_stride;
        trans += 3;
        count -= 3;
    }

    const int stride = 6; // two triangles
#else
    const int stride = 3; // a triangle
#endif

    // triangles
    for(int i = 0; i < count; i += stride)
    {
        // vertex limit reached
        // check two triangles for clipping pessimism
        if(trans + stride + 3 >= transformed_vertices + std::size(transformed_vertices))
            break;

        // calculate final positions
        for(int j = 0; j < 3; j++)
            position_shader(ptr + (i + j) * vertex_stride, trans + j, *this);


        // test against near/far planes
        int far_out = 0, near_out = 0;

        for(int i = 0; i < 3; i++)
        {
            if(trans[i].z > trans[i].w)
                far_out += 1 << i;
            else if(trans[i].z < -trans[i].w)
                near_out += 1 << i;
        }

        // crosses both planes, that's one big triangle...
        if(near_out && far_out)
            continue;

        // entirely outside
        if(near_out == 7 || far_out == 7)
            continue;

        Fixed32<20> clipped_factors[2 * 3 * 3];
        bool add_triangle = false;

        // far plane
        if(far_out)
            add_triangle = clip_triangle(trans, 1, far_out, clipped_factors);
        // near plane
        else if(near_out)
            add_triangle = clip_triangle(trans, -1, near_out, clipped_factors);

        bool clipped = near_out || far_out;

        // calculate the rest of the attributes
        for(int j = 0; j < 3; j++)
            vertex_shader(ptr + (i + j) * vertex_stride, trans + j, *this);

        // apply clipping to attributes now that we have them
        if(clipped)
        {
            VertexOutData temp[3];
            // blend ALL the attributes
            for(int j = 0; j < 3; j++)
            {
                temp[j].x = trans[0].x * clipped_factors[j * 3 + 0] + trans[1].x * clipped_factors[j * 3 + 1] + trans[2].x * clipped_factors[j * 3 + 2];
                temp[j].y = trans[0].y * clipped_factors[j * 3 + 0] + trans[1].y * clipped_factors[j * 3 + 1] + trans[2].y * clipped_factors[j * 3 + 2];
                temp[j].z = trans[0].z * clipped_factors[j * 3 + 0] + trans[1].z * clipped_factors[j * 3 + 1] + trans[2].z * clipped_factors[j * 3 + 2];
                temp[j].w = trans[0].w * clipped_factors[j * 3 + 0] + trans[1].w * clipped_factors[j * 3 + 1] + trans[2].w * clipped_factors[j * 3 + 2];

                temp[j].r = uint8_t(Fixed32<>(trans[0].r) * clipped_factors[j * 3 + 0] + Fixed32<>(trans[1].r) * clipped_factors[j * 3 + 1] + Fixed32<>(trans[2].r) * clipped_factors[j * 3 + 2]);
                temp[j].g = uint8_t(Fixed32<>(trans[0].g) * clipped_factors[j * 3 + 0] + Fixed32<>(trans[1].g) * clipped_factors[j * 3 + 1] + Fixed32<>(trans[2].g) * clipped_factors[j * 3 + 2]);
                temp[j].b = uint8_t(Fixed32<>(trans[0].b) * clipped_factors[j * 3 + 0] + Fixed32<>(trans[1].b) * clipped_factors[j * 3 + 1] + Fixed32<>(trans[2].b) * clipped_factors[j * 3 + 2]);

                temp[j].tex_index = trans[0].tex_index; // tex index can't be blended

                temp[j].u = Fixed16<12>(clipped_factors[j * 3 + 0] * trans[0].u + clipped_factors[j * 3 + 1] * trans[1].u + clipped_factors[j * 3 + 2] * trans[2].u);
                temp[j].v = Fixed16<12>(clipped_factors[j * 3 + 0] * trans[0].v + clipped_factors[j * 3 + 1] * trans[1].v + clipped_factors[j * 3 + 2] * trans[2].v);
            }

            // second tri
            if(add_triangle)
            {
                auto trans2 = trans + stride;
                auto factors = clipped_factors + 3 * 3;
                for(int j = 0; j < 3; j++)
                {
                    trans2[j].x = trans[0].x * factors[j * 3 + 0] + trans[1].x * factors[j * 3 + 1] + trans[2].x * factors[j * 3 + 2];
                    trans2[j].y = trans[0].y * factors[j * 3 + 0] + trans[1].y * factors[j * 3 + 1] + trans[2].y * factors[j * 3 + 2];
                    trans2[j].z = trans[0].z * factors[j * 3 + 0] + trans[1].z * factors[j * 3 + 1] + trans[2].z * factors[j * 3 + 2];
                    trans2[j].w = trans[0].w * factors[j * 3 + 0] + trans[1].w * factors[j * 3 + 1] + trans[2].w * factors[j * 3 + 2];

                    trans2[j].r = uint8_t(Fixed32<>(trans[0].r) * factors[j * 3 + 0] + Fixed32<>(trans[1].r) * factors[j * 3 + 1] + Fixed32<>(trans[2].r) * factors[j * 3 + 2]);
                    trans2[j].g = uint8_t(Fixed32<>(trans[0].g) * factors[j * 3 + 0] + Fixed32<>(trans[1].g) * factors[j * 3 + 1] + Fixed32<>(trans[2].g) * factors[j * 3 + 2]);
                    trans2[j].b = uint8_t(Fixed32<>(trans[0].b) * factors[j * 3 + 0] + Fixed32<>(trans[1].b) * factors[j * 3 + 1] + Fixed32<>(trans[2].b) * factors[j * 3 + 2]);

                    trans2[j].tex_index = trans[0].tex_index; // tex index can't be blended

                    trans2[j].u = Fixed16<12>(factors[j * 3 + 0] * trans[0].u + factors[j * 3 + 1] * trans[1].u + factors[j * 3 + 2] * trans[2].u);
                    trans2[j].v = Fixed16<12>(factors[j * 3 + 0] * trans[0].v + factors[j * 3 + 1] * trans[1].v + factors[j * 3 + 2] * trans[2].v);

                    transform_vertex(trans2[j]);
                }

                // cull new triangle
                add_triangle = !cull_triangle(trans2);
            }

            memcpy(trans, temp, sizeof(VertexOutData) * 3);
        }

        // w divide and viewport transform
        for(int j = 0; j < 3; j++)
            transform_vertex(trans[j]);

        // cull back faces and empty
        if(cull_triangle(trans))
        {
            if(add_triangle)
            {
                // we still need to keep the second triangle though
                // this might happen due to precision issues causing the first triangle to get culled
                memcpy(trans, trans + stride, sizeof(VertexOutData) * 3);
                trans += stride;
            }
            continue;
        }

        trans += stride;

        if(add_triangle)
            trans += stride;
    }

#if THR3E_PICO_MULTICORE
    if(core_num == 0)
    {
        auto trans2 = reinterpret_cast<VertexOutData *>(multicore_fifo_pop_blocking());

        if(trans2 > trans)
            std::swap(trans2, trans);

        while(trans2 < trans - 3)
        {
            // uhoh, we have gaps
            // move a triangle from the longer list to the shorter one
            memcpy(trans2, trans - 6, sizeof(VertexOutData) * 3);
            trans2 += 6;
            trans -= 6;
        }

        // might've tipped the balance the other way
        if(trans2 > trans)
            trans = trans2;

        // adjust back (we skipped one triangle ahead)
        trans -= 3;
    }
    else
    {
        multicore_fifo_push_blocking(reinterpret_cast<intptr_t>(trans));
        return;
    }
#endif

    transformed_vertex_ptr = trans;
}

const FixedMat4<> &Render3D::get_model_view() const
{
    return model_view;
}

void Render3D::set_model_view(FixedMat4<> m)
{
    model_view = m;
    //mvp = model_view * projection;
    mvp = projection * model_view;
}

const FixedMat4<> &Render3D::get_projection() const
{
    return projection;
}

void Render3D::set_projection(FixedMat4<> m)
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

void Render3D::set_position_shader(VertexShaderFunc shader)
{
    position_shader = shader;
}

void Render3D::set_vertex_shader(VertexShaderFunc shader)
{
    vertex_shader = shader;
}

void Render3D::set_shader_params(void *params)
{
    shader_params = params;
}

const void *Render3D::get_shader_params() const
{
    return shader_params;
}

void Render3D::set_texture(blit::Surface *tex, int index)
{
    textures[index] = tex;
}

bool Render3D::get_fill_triangles() const
{
    return filled_triangles;
}

void Render3D::set_fill_triangles(bool filled)
{
    filled_triangles = filled;
}

void Render3D::set_clear_colour(blit::Pen colour)
{
    clear_colour = pack_colour(colour);
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

#if THR3E_PICO_MULTICORE
    // launch the other core if needed
    auto core_num = get_core_num();
    if(core_num == 0)
    {
        multicore_fifo_push_blocking(reinterpret_cast<uintptr_t>(this));
        multicore_fifo_push_blocking(uint32_t(Core1Job::Rasterise));
    }
#endif

#if THR3E_PICO_INTERP
    // setup interpolators
    auto config = interp_default_config();

    // start/end x in fill_triangle
    interp_config_set_shift(&config, 16);     // shift out 16 fractional bits
    interp_config_set_mask(&config, 0, 15);   // mask out the nothing in the upper 16 bits
    interp_config_set_signed(&config, true);  // ... then sign extend (which is why we need the mask)
    interp_config_set_add_raw(&config, true); // skip everything for the value stored to the accumulator

    interp_set_config(interp0, 0, &config);
    interp_set_config(interp0, 1, &config);

    // TODO: config?
    // also duplicated
    constexpr int tex_size_bits = 8; // can't be > 8

    // texture mapping
    config = interp_default_config();
    // x
    interp_config_set_shift(&config, 16 - tex_size_bits); // 16 bit fraction, shift out unneeded low bits
    interp_config_set_mask(&config, 0, tex_size_bits - 1);
    interp_config_set_add_raw(&config, true); 
    interp_set_config(interp1, 0, &config);

    // y
    interp_config_set_shift(&config, (16 - tex_size_bits) - tex_size_bits); // similar but less shifted to effectively multiply by width
    interp_config_set_mask(&config, tex_size_bits, tex_size_bits + tex_size_bits - 1);
    interp_set_config(interp1, 1, &config);

    // this should result in the texture offset in the full result

#endif

    uint32_t clear_col32 = clear_colour | clear_colour << 16;
    uint32_t clear_depth32 = 0xFFFFFFFF;

    auto col_buf = tile_colour_buffer;
    auto depth_buf = tile_depth_buffer;
    const auto tile_buf_size = sizeof(tile_colour_buffer) / num_tile_bufs;

#if THR3E_PICO_MULTICORE
    // offset for per-core tile buffers
    col_buf += core_num * tile_width * tile_height;
    depth_buf += core_num * tile_width * tile_height;
    bool offset_row = core_num == 1;
#endif

    // filled or wireframe triangle
    auto do_triangle = filled_triangles ? &Render3D::fill_triangle : &Render3D::wireframe_triangle;

    // rasterise triangles for each screen tile
    for(int y = 0; y < screen.bounds.h; y += tile_height)
    {
#if THR3E_PICO_MULTICORE
        // split tiles between cores
        // C0 C1 C0 ...
        // C1 C0 C1 ...
        // ...
        int start_x = offset_row ? 0 : tile_width;
        offset_row = !offset_row;
        for(int x = start_x; x < screen.bounds.w; x += tile_width * 2)
#else
        for(int x = 0; x < screen.bounds.w; x += tile_width)
#endif
        {
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
                (this->*do_triangle)(ptr, {x, y});

            // store colour tile
            if(screen.format == PixelFormat::BGR555)
            {
                // assume picovision, which has a 555 -> 555 blit
#if THR3E_PICO_MULTICORE
                // blitting on both cores at once would blow up
                mutex_enter_blocking(&blit_mutex);
#endif

                tile_surf.data = reinterpret_cast<uint8_t *>(col_buf);
                screen.blit(&tile_surf, {0, 0, tile_width, tile_height}, {x, y});

#if THR3E_PICO_MULTICORE
                mutex_exit(&blit_mutex);
#endif
            }
            else if(screen.format == PixelFormat::RGB565)
            {
                // matching format copy
                for(int ty = 0; ty < tile_height; ty++)
                    memcpy(screen.ptr(x, y + ty), col_buf + ty * tile_width, tile_width * 2);
            }
        }
    }

#if THR3E_PICO_MULTICORE
    if(core_num == 0)
    {
        // wait for core1 and reset
        multicore_fifo_pop_blocking();
    }
    else
    {
        multicore_fifo_push_blocking(0); // done
        return; // don't reset vertex ptr on core1
    }
#endif

    transformed_vertex_ptr = nullptr;
}

void Render3D::transform_vertex(VertexOutData &pos)
{
    // perspective
    auto recip_w = Fixed32<20>(1) / Fixed32<20>(pos.w); // .reciprocal is way too inaccurate above 16 frac bits

    pos.x *= recip_w;
    pos.y *= recip_w;
    pos.z *= recip_w;

    Rect viewport{0, 0, screen.bounds.w, screen.bounds.h};

    // viewport
    // store the result as integers
    pos.x = Fixed32<>::from_raw(viewport.x + int32_t(Fixed32<0>(viewport.w / 2) * (pos.x + 1)));
    pos.y = Fixed32<>::from_raw(viewport.y + int32_t(Fixed32<0>(viewport.h / 2) * (pos.y + 1)));

    if(pos.z < -1)
        pos.z = -1;
    else if(pos.z > 1)
        pos.z = 1;

    pos.z = (pos.z + 1) * Fixed32<>(32767.5f);
}

bool Render3D::cull_triangle(VertexOutData *verts)
{
    // back-face culling
    auto ab_x = verts[1].x.raw() - verts[0].x.raw();
    auto ab_y = verts[1].y.raw() - verts[0].y.raw();
    auto ac_x = verts[2].x.raw() - verts[0].x.raw();
    auto ac_y = verts[2].y.raw() - verts[0].y.raw();

    // nothing to draw
    if((ab_x == 0 && ab_y == 0) || (ac_x == 0 && ac_y == 0))
        return true;

    int32_t z = ab_x * ac_y - ab_y * ac_x;

    if(z > 0)
        return true;

    return false;
}

bool Render3D::clip_triangle(VertexOutData *verts, Fixed32<> bound, int verts_outside, Fixed32<20> out_factors[2 * 3 * 3])
{
    // there's one vertex on one side and two on the other
    // just need to figure out which is which... and which way around it is
    VertexOutData *a, *b, *c;

    bool one_outside = !(verts_outside & (verts_outside - 1));
    int a_index;

    if(one_outside)
    {
        // one vert outside, two inside (result is two triangles)
        a_index = verts_outside / 2; // 1,2,4 -> 0,1,2
    }
    else
    {
        // two verts outside, one inside (result is one triangle)
        auto inside = ~verts_outside & 7;
        a_index = inside / 2; // 1,2,4 -> 0,1,2
    }

    a = verts + a_index;
    b = verts + (a_index + 1) % 3;
    c = verts + (a_index + 2) % 3;

    // we've got our vertices, now intersect the edges (a-b and a-c)
    // (bound is +/-1 used to flip for far/near plane)
    Fixed32<20> bf, cf;

    bf = Fixed32<20>(a->w - a->z * bound) / Fixed32<20>((a->w - a->z * bound) - (b->w - b->z * bound));
    cf = Fixed32<20>(a->w - a->z * bound) / Fixed32<20>((a->w - a->z * bound) - (c->w - c->z * bound));

    int b_index = b - verts;
    int c_index = c - verts;

    // and setup the factors for the new triangles
    // (can't build them yet, we don't have the attributes)
    if(one_outside)
    {
        // A = B * bf + A * (1 - bf)
        out_factors[a_index * 3 + a_index] = Fixed32<20>(1) - bf;
        out_factors[a_index * 3 + b_index] = bf;
        out_factors[a_index * 3 + c_index] = 0;

        // B = B
        out_factors[b_index * 3 + a_index] = 0;
        out_factors[b_index * 3 + b_index] = 1;
        out_factors[b_index * 3 + c_index] = 0;

        // C = C
        out_factors[c_index * 3 + a_index] = 0;
        out_factors[c_index * 3 + b_index] = 0;
        out_factors[c_index * 3 + c_index] = 1;

        // second triangle

        // A = B * bf + A * (1 - bf)
        out_factors[(a_index + 3) * 3 + a_index] = Fixed32<20>(1) - bf;
        out_factors[(a_index + 3) * 3 + b_index] = bf;
        out_factors[(a_index + 3) * 3 + c_index] = 0;

        // B = C
        out_factors[(b_index + 3) * 3 + a_index] = 0;
        out_factors[(b_index + 3) * 3 + b_index] = 0;
        out_factors[(b_index + 3) * 3 + c_index] = 1;

        // C = C * cf + A * (1 - cf)
        out_factors[(c_index + 3) * 3 + a_index] = Fixed32<20>(1) - cf;
        out_factors[(c_index + 3) * 3 + b_index] = 0;
        out_factors[(c_index + 3) * 3 + c_index] = cf;
        return true;
    }
    else
    {
        // A = A
        out_factors[a_index * 3 + a_index] = 1;
        out_factors[a_index * 3 + b_index] = 0;
        out_factors[a_index * 3 + c_index] = 0;

        // B = B * bf + A * (1 - bf)
        out_factors[b_index * 3 + a_index] = Fixed32<20>(1) - bf;
        out_factors[b_index * 3 + b_index] = bf;
        out_factors[b_index * 3 + c_index] = 0;

        // C = C * cf + A * (1 - cf)
        out_factors[c_index * 3 + a_index] = Fixed32<20>(1) - cf;
        out_factors[c_index * 3 + b_index] = 0;
        out_factors[c_index * 3 + c_index] = cf;

        // no second triangle
        return false;
    }
}

void blit_fast_code(Render3D::fill_triangle)(VertexOutData *data, blit::Point tile_pos)
{
    IntVec3 p0{data[0].x.raw() - tile_pos.x, data[0].y.raw() - tile_pos.y, int32_t(UFixed32<>(data[0].z))};
    IntVec3 p1{data[1].x.raw() - tile_pos.x, data[1].y.raw() - tile_pos.y, int32_t(UFixed32<>(data[1].z))};
    IntVec3 p2{data[2].x.raw() - tile_pos.x, data[2].y.raw() - tile_pos.y, int32_t(UFixed32<>(data[2].z))};

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

    // nothing to draw, also would divide by zero later
    if(p0.y == p1.y && p1.y == p2.y)
        return;

    Pen cols[3]{
        {data[0].r, data[0].g, data[0].b},
        {data[1].r, data[1].g, data[1].b},
        {data[2].r, data[2].g, data[2].b}
    };

    // tex coords/enable
    auto tex = data[0].tex_index ? textures[data[0].tex_index - 1] : nullptr;

    Fixed16<12> u[3]{data[0].u, data[1].u, data[2].u};
    Fixed16<12> v[3]{data[0].v, data[1].v, data[2].v};

    // sort points
    if(p0.y > p2.y)
    {
        std::swap(p0, p2);
        std::swap(cols[0], cols[2]);
        if(tex)
        {
            std::swap(u[0], u[2]);
            std::swap(v[0], v[2]);
        }
    }
    if(p0.y > p1.y)
    {
        std::swap(p0, p1);
        std::swap(cols[0], cols[1]);
        if(tex)
        {
            std::swap(u[0], u[1]);
            std::swap(v[0], v[1]);
        }
    }
    if(p1.y > p2.y)
    {
        std::swap(p1, p2);
        std::swap(cols[1], cols[2]);
        if(tex)
        {
            std::swap(u[1], u[2]);
            std::swap(v[1], v[2]);
        }
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

#if THR3E_PICO_INTERP
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

            if(tex)
            {
                auto start_u = u[0] + (u[2] - u[0]) * y_frac1;
                auto start_v = v[0] + (v[2] - v[0]) * y_frac1;
                auto end_u = u[0] + (u[1] - u[0]) * y_frac2;
                auto end_v = v[0] + (v[1] - v[0]) * y_frac2;
                textured_h_line(int32_t(start_x), int32_t(end_x), start_z, end_z, y, start_col, end_col, tex, start_u, end_u, start_v, end_v);
            }
            else
                gradient_h_line(int32_t(start_x), int32_t(end_x), start_z, end_z, y, start_col, end_col);
        }

#if THR3E_PICO_INTERP
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

#if THR3E_PICO_INTERP
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
            if(tex)
            {
                auto start_u = u[0] + (u[2] - u[0]) * y_frac1;
                auto start_v = v[0] + (v[2] - v[0]) * y_frac1;
                auto end_u = u[1] + (u[2] - u[1]) * y_frac2;
                auto end_v = v[1] + (v[2] - v[1]) * y_frac2;
                textured_h_line(int32_t(start_x), int32_t(end_x), start_z, end_z, y, start_col, end_col, tex, start_u, end_u, start_v, end_v);
            }
            else
                gradient_h_line(int32_t(start_x), int32_t(end_x), start_z, end_z, y, start_col, end_col);
        }
    }
}

void Render3D::wireframe_triangle(VertexOutData *data, blit::Point tile_pos)
{
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

    // nothing to draw
    if(p0.y == p1.y && p1.y == p2.y)
        return;

    Pen cols[3]{
        {data[0].r, data[0].g, data[0].b},
        {data[1].r, data[1].g, data[1].b},
        {data[2].r, data[2].g, data[2].b}
    };

    // no texture handling, could be done but not sure how much use it would be...
    gradient_line({p0.x, p0.y}, {p1.x, p1.y}, p0.z, p1.z, cols[0], cols[1]);
    gradient_line({p1.x, p1.y}, {p2.x, p2.y}, p1.z, p2.z, cols[1], cols[2]);
    gradient_line({p2.x, p2.y}, {p0.x, p0.y}, p2.z, p0.z, cols[2], cols[0]);
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

#if THR3E_PICO_MULTICORE
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

// the above, but with texture mapping
void blit_fast_code(Render3D::textured_h_line)(int x1, int x2, uint16_t z1, uint16_t z2, int y, Pen col1, Pen col2, Surface *tex, Fixed16<12> u1, Fixed16<12> u2, Fixed16<12> v1, Fixed16<12> v2)
{
    if(x1 > x2)
    {
        std::swap(x1, x2);
        std::swap(z1, z2);
        std::swap(col1, col2);
        std::swap(u1, u2);
        std::swap(v1, v2);
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

    // tex
    auto u_step = Fixed32<>(u2 - u1) * x_scale;
    auto v_step = Fixed32<>(v2 - v1) * x_scale;
    auto u = Fixed32<>(u1);
    auto v = Fixed32<>(v1);

    // clamp x
    if(x2 > tile_width)
        x2 = tile_width;

    if(x1 < 0)
    {
        z += z_step * -x1;
        r += r_step * -x1;
        g += g_step * -x1;
        b += b_step * -x1;
        u += u_step * -x1;
        v += v_step * -x1;
        x1 = 0;
    }

    auto col_ptr = tile_colour_buffer + x1 + y * tile_width;
    auto depth_ptr = tile_depth_buffer + x1 + y * tile_width;

#if THR3E_PICO_MULTICORE
    auto core_num = get_core_num();
    col_ptr += core_num * tile_width * tile_height;
    depth_ptr += core_num * tile_width * tile_height;
#endif

    auto end_ptr = col_ptr + (x2 - x1);

#if THR3E_PICO_INTERP
    interp1->accum[0] = u.raw();
    interp1->accum[1] = v.raw();

    interp1->base[0] = u_step.raw();
    interp1->base[1] = v_step.raw();
    interp1->base[2] = 0; // could be the data address if we assume a paletted surface...

    for(; col_ptr < end_ptr; col_ptr++, depth_ptr++, z += z_step, r += r_step, g += g_step, b += b_step)
    {
        auto tex_offset = interp1->pop[2];
#else

    // TODO: config?
    constexpr int tex_size_bits = 8;
    constexpr int tex_size = 1 << tex_size_bits;

    for(; col_ptr < end_ptr; col_ptr++, depth_ptr++, z += z_step, r += r_step, g += g_step, b += b_step, u += u_step, v += v_step)
    {
#endif
        if(int32_t(z) > *depth_ptr)
            continue;

        // this could be optimised
#if THR3E_PICO_INTERP
        auto tex_col = tex->get_pixel(tex_offset);
#else
        auto tex_col = tex->get_pixel({(u.raw() >> (16 - tex_size_bits)) & (tex_size - 1), (v.raw() >> (16 - tex_size_bits)) & (tex_size - 1)});
#endif

        Pen col{(uint8_t(r) * tex_col.r) >> 8, (uint8_t(g) * tex_col.g) >> 8, (uint8_t(b) * tex_col.b) >> 8};

        *col_ptr = pack_colour(col);
        *depth_ptr = int32_t(z);
    }
}

// used for wireframe
void Render3D::gradient_line(Point p1, Point p2, uint16_t z1, uint16_t z2, Pen col1, Pen col2)
{
    if(p1 == p2)
        return;

    int32_t dx = int32_t(abs(p2.x - p1.x));
    int32_t dy = -int32_t(abs(p2.y - p1.y));

    int32_t sx = (p1.x < p2.x) ? 1 : -1;
    int32_t sy = (p1.y < p2.y) ? 1 : -1;

    int32_t err = dx + dy;

    Point p(p1);

    int count = std::max(dx, -dy);
    auto scale = Fixed32<>(1) / count;

    // depth
    int z_diff = z2 - z1;
    auto z_step = Fixed32<15>(scale) * z_diff;
    auto z = Fixed32<15>(z1);

    // colour
    auto col_diff = col2 - col1;

    auto r_step = scale * col_diff.r;
    auto g_step = scale * col_diff.g;
    auto b_step = scale * col_diff.b;

    auto r = Fixed32<>(col1.r);
    auto g = Fixed32<>(col1.g);
    auto b = Fixed32<>(col1.b);

    while(true)
    {
        if(p.x >= 0 && p.y >= 0 && p.x < tile_width && p.y < tile_height)
        {
            // depth test
            if(int32_t(z) <= tile_depth_buffer[p.x + p.y * tile_width])
            {
                tile_depth_buffer[p.x + p.y * tile_width] = int32_t(z);
                tile_colour_buffer[p.x + p.y * tile_width] = pack_colour({uint8_t(r), uint8_t(g), uint8_t(b)});
            }
        }

        if(count-- == 0) break;

        int32_t e2 = err * 2;
        if(e2 >= dy) { err += dy; p.x += sx; }
        if(e2 <= dx) { err += dx; p.y += sy; }

        // increment depth/colour
        z += z_step;

        r += r_step;
        g += g_step;
        b += b_step;
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