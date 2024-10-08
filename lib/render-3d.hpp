#pragma once
#include <cstdint>

#include "graphics/surface.hpp"
#include "types/point.hpp"

#include "config.h"
#include "fixed-mat4.hpp"


class Render3D
{
public:
    struct VertexOutData
    {
        Fixed32<> x, y, z, w;
        uint8_t r, g, b, tex_index;
        Fixed16<12> u, v;
    };

    using VertexShaderFunc = void(*)(const uint8_t *, VertexOutData *, const Render3D &);

    Render3D();

    void draw(int count, const uint8_t *ptr);

    const FixedMat4<> &get_model_view() const;
    void set_model_view(FixedMat4<> m);

    const FixedMat4<> &get_projection() const;
    void set_projection(FixedMat4<> m);

    const FixedMat4<> &get_model_view_projection() const;

    void set_vertex_stride(int stride);

    void set_position_shader(VertexShaderFunc shader);
    void set_vertex_shader(VertexShaderFunc shader);

    // tex would be const but blit::Surface is missing some consts...
    void set_texture(blit::Surface *tex, int index = 0);

    int get_transformed_vertex_count() const;

    void rasterise();

protected:
    void transform_vertex(VertexOutData &pos);

    bool cull_triangle(VertexOutData *verts);
    bool clip_triangle(VertexOutData *verts, Fixed32<> bound, int verts_outside, Fixed32<20> out_factors[2 * 3 * 3]);

    void fill_triangle(VertexOutData *data, blit::Point tile_pos);

    void gradient_h_line(int x1, int x2, uint16_t z1, uint16_t z2, int y, blit::Pen col1, blit::Pen col2);
    void textured_h_line(int x1, int x2, uint16_t z1, uint16_t z2, int y, blit::Pen col1, blit::Pen col2, blit::Surface *tex, Fixed16<12> u1, Fixed16<12> u2, Fixed16<12> v1, Fixed16<12> v2);

    uint16_t pack_colour(blit::Pen p);
    blit::Pen unpack_colour(uint16_t c);

    FixedMat4<> model_view, projection;
    FixedMat4<> mvp;

    int vertex_stride = 3;
    VertexShaderFunc position_shader = nullptr, vertex_shader = nullptr;

    static constexpr int max_textures = 1;
    blit::Surface *textures[max_textures];

    VertexOutData transformed_vertices[THR3E_MAX_OUTPUT_VERTICES];
    VertexOutData *transformed_vertex_ptr = nullptr;

    static constexpr int tile_width = THR3E_TILE_WIDTH, tile_height = THR3E_TILE_HEIGHT;
#if THR3E_PICO_MULTICORE
    static constexpr int num_tile_bufs = 2;
#else
    static constexpr int num_tile_bufs = 1;
#endif
    uint16_t tile_colour_buffer[tile_width * tile_height * num_tile_bufs];
    uint16_t tile_depth_buffer[tile_width * tile_height * num_tile_bufs];

    // used for picovision blit
    blit::Surface tile_surf;
};

// 16.16 fixed point positions, multiply by mvp
void fixed32_mvp_pos_shader(const uint8_t *in, Render3D::VertexOutData *out, const Render3D &r);
