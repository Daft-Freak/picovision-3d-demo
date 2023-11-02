#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>

#include "32blit.hpp"
#include "engine/api_private.hpp"

#include "assets.hpp"
#include "model.hpp"
#include "render-3d.hpp"
#include "vec4.hpp"

using namespace blit;

// copy/paste from an SDK branch so I can compile on master...
bool set_screen_mode(ScreenMode new_mode, PixelFormat format, Size bounds) {
    SurfaceTemplate new_screen;
    new_screen.format = format;
    new_screen.bounds = bounds;

    if(!api.set_screen_mode_format(new_mode, new_screen))
      return false;

    screen = Surface(new_screen.data, new_screen.format, new_screen.bounds);
    screen.palette = new_screen.palette;

    if(new_screen.pen_blend)
        screen.pbf = new_screen.pen_blend;
    
    if(new_screen.blit_blend)
        screen.bbf = new_screen.blit_blend;

    if(new_screen.pen_get)
        screen.pgf = new_screen.pen_get;

    return true;
}

// mat helpers

Mat4 frustum(float left, float right, float bottom, float top, float nearVal, float farVal)
{
    Mat4 mat = Mat4::identity();

    mat.v00 = (2.0f * nearVal) / (right - left);

    mat.v11 = (2.0f * nearVal) / (top - bottom);

    mat.v02 = (right + left) / (right - left);
    mat.v12 = (top + bottom) / (top - bottom);
    mat.v22 = -((farVal + nearVal) / (farVal - nearVal));
    mat.v32 = -1.0f;

    mat.v23 = -((2 * farVal * nearVal) / (farVal - nearVal));
    mat.v33 = 0.0f;

    return mat;
}

static Render3D r3d;
static float ang = 0.0f, ang2 = 0.0f;
static Model model(asset_cube);

void init()
{
    // attempt to go super-hires on picovision
    if(!::set_screen_mode(ScreenMode::hires, PixelFormat::BGR555, {640, 480}))
        set_screen_mode(ScreenMode::hires);

    auto near = 0.1, far = 10.0;
    auto tanFov = tan(0.785398163);
    auto nearH = near * tanFov;
    auto nearW = nearH * (4.0 / 3.0);
    r3d.set_projection(frustum(-nearW, nearW, -nearH, nearH, near, far));
}

void update(uint32_t time)
{

}

void render(uint32_t time)
{
    /*Mat4 m;
    m.identity();
    m.rotate(ang, Vec3{0.0f, 1.0f, 0.0f});
    m.translate({0.0f, 0.0f, -4.0f});
    r3d.set_model_view(m);*/
    r3d.set_model_view(Mat4::translation(Vec3(0.0f, 0.0f, -4.0f)) * Mat4::rotation(ang, Vec3{0.0f, 1.0f, 0.0f}) * Mat4::rotation(ang2, Vec3{1.0f, 0.0f, 0.0f}));

    auto vert_start = now_us();
    r3d.set_vertex_shader(model_lit_shader);
    model.draw(r3d);
    auto vert_end = now_us();

    auto frag_start = now_us();
    r3d.rasterise();
    auto frag_end = now_us();

    ang += 0.5f;
    ang2 -= 0.3f;

    screen.pen = {255, 0, 0};
    screen.text(std::to_string(us_diff(vert_start, vert_end)), minimal_font, {0, 0});
    screen.text(std::to_string(us_diff(frag_start, frag_end)), minimal_font, {0, 10});
}