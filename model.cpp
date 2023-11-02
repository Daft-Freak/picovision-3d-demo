#include "model.hpp"

#include "types/vec3.hpp"

#include "vec4.hpp"

void model_lit_shader(const uint8_t *in, Render3D::VertexOutData *out, const Render3D &r)
{
    auto vertex = reinterpret_cast<const Model::Vertex *>(in);

    FixedVec4 tmp(Fixed32<>::from_raw(vertex->x), Fixed32<>::from_raw(vertex->y), Fixed32<>::from_raw(vertex->z), Fixed32<>(1.0f));
    
    // transform
    tmp = r.get_model_view_projection() * tmp;

    // pos
    out->x = tmp.x;
    out->y = tmp.y;
    out->z = tmp.z;
    out->w = tmp.w;

    // col
    // TODO: store light vec... somewhere
    blit::Vec3 light(-0.577350269f, -0.577350269f, -0.577350269f);

    // TODO: more fixed
    Vec4 nor(vertex->nx / 32767.0f, vertex->ny / 32767.0f, vertex->nz / 32767.0f, 1.0f);
    nor = r.get_model_view() * nor;
    float len = sqrt(nor.x * nor.x + nor.y * nor.y + nor.z * nor.z);
    nor.x /= len;
    nor.y /= len;
    nor.z /= len;

    float dot = light.x * nor.x + light.y * nor.y + light.z * nor.z;

    out->r = std::max(dot, 0.0f) * vertex->r;
    out->g = std::max(dot, 0.0f) * vertex->g;
    out->b = std::max(dot, 0.0f) * vertex->b;
}

Model::Model(const uint8_t *asset_data)
{
    // TODO: check magic header

    auto ptr32 = reinterpret_cast<const uint32_t *>(asset_data + 4);

    num_meshes = *ptr32++;
    meshes = new Mesh[num_meshes];

    for(uint32_t i = 0; i < num_meshes; i++)
    {
        meshes[i].num_vertices = *ptr32++;
        meshes[i].vertices = reinterpret_cast<const Vertex *>(ptr32);

        ptr32 += meshes[i].num_vertices * sizeof(Vertex) / sizeof(uint32_t);
    }
}

Model::~Model()
{
    delete[] meshes;
}

void Model::draw(Render3D &r3d)
{
    r3d.set_vertex_stride(sizeof(Vertex));

    for(uint32_t i = 0; i < num_meshes; i++)
        r3d.draw(meshes[i].num_vertices, reinterpret_cast<const uint8_t *>(meshes[i].vertices));
}