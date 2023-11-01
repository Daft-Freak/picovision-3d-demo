#include <cstdint>

#include "types/mat4.hpp"

#include "fixed-mat4.hpp"

class Render3D
{
public:
    struct VertexOutData
    {
        Fixed32<> x, y, z, w;
        uint8_t r, g, b, a;
    };

    using VertexShaderFunc = void(*)(const float *, VertexOutData *, const Render3D &);

    Render3D();

    void draw(int count, const float *ptr);

    const blit::Mat4 &get_model_view() const;
    void set_model_view(blit::Mat4 m);

    const blit::Mat4 &get_projection() const;
    void set_projection(blit::Mat4 m);

    const FixedMat4<> &get_model_view_projection() const;

    void set_vertex_stride(int stride);

    void set_vertex_shader(VertexShaderFunc shader);

    void rasterise();

protected:
    void transform_vertex(VertexOutData &pos);

    void fill_triangle(VertexOutData *data, blit::Point tile_pos);

    void gradient_h_line(int x1, int x2, uint16_t z1, uint16_t z2, int y, blit::Pen col1, blit::Pen col2);

    uint16_t pack_colour(blit::Pen p);
    blit::Pen unpack_colour(uint16_t c);

    blit::Mat4 model_view, projection;
    FixedMat4<> mvp;

    int vertex_stride = 3;
    VertexShaderFunc vertex_shader = nullptr;

    VertexOutData transformed_vertices[1024];
    VertexOutData *transformed_vertex_ptr = nullptr;

    static constexpr int tile_width = 80, tile_height = 80;
    static constexpr int num_tile_bufs = 1;
    uint16_t tile_colour_buffer[tile_width * tile_height * num_tile_bufs];
    uint16_t tile_depth_buffer[tile_width * tile_height * num_tile_bufs];

    // used for picovision blit
    blit::Surface tile_surf;
};
