#include <cstdint>

#include "types/mat4.hpp"

#include "fixed-mat4.hpp"

class Render3D;
using VertexShaderFunc = void(*)(const float *, float *, const Render3D &);

class Render3D
{
public:
    void clear();

    void draw(int count, const float *ptr);

    const blit::Mat4 &get_model_view() const;
    void set_model_view(blit::Mat4 m);

    const blit::Mat4 &get_projection() const;
    void set_projection(blit::Mat4 m);

    const FixedMat4<> &get_model_view_projection() const;

    void set_vertex_stride(int stride);

    void set_vertex_shader(VertexShaderFunc shader);

protected:
    void transform_vertex(float *pos);

    void fill_triangle(float *data);

    void gradient_h_line(int x1, int x2, float z1, float z2, int y, const blit::Vec3 &col1, const blit::Vec3 &col2);

    //uint8_t framebuffer[320 * 240 * 3];
    uint16_t depth_buffer[320 * 240];
    blit::Mat4 model_view, projection;
    FixedMat4<> mvp;

    int vertex_stride = 3;
    VertexShaderFunc vertex_shader = nullptr;
};
