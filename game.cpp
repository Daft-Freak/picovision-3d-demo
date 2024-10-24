#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>

#include "32blit.hpp"

#include "camera.hpp"
#include "model.hpp"
#include "render-3d.hpp"

#include "assets.hpp"

using namespace blit;

static Render3D r3d;
static float ang = 0.0f, ang2 = 0.0f;
static float scale = 0.1f;

static ModelShaderParams shader_params;
static Model model(asset_picovision);
static Surface *picovision_tex;

static uint32_t start_time;
static bool done_zoom = false;

void init()
{
    // attempt to go super-hires on picovision
    if(!::set_screen_mode(ScreenMode::hires, PixelFormat::BGR555, {640, 480}))
        set_screen_mode(ScreenMode::hires, PixelFormat::RGB565);

    r3d.set_projection(Camera::perspective_matrix(0.1f, 10.0f, blit::pi / 4.0f, float(screen.bounds.w) / screen.bounds.h));

    shader_params.light_direction = {-0.577350269f, 0.577350269f, 0.577350269f};
    r3d.set_shader_params(&shader_params);

    picovision_tex = Surface::load(asset_picovision_tex);
    r3d.set_texture(picovision_tex, 0);

    start_time = now();
}

void update(uint32_t time)
{
    ang += 0.25f;
    ang2 -= 0.15f;

    // zoom in after a delay
    if(!done_zoom && time - start_time > 4000 && scale < 45.0f)
        scale *= 1.05f;
    else if(scale > 40.0f)
    {
        // then back down a little
        done_zoom = true;
        scale = std::max(scale - 0.2f, 40.0f);
    }
}

void render(uint32_t time)
{
    [[maybe_unused]]
    auto render_start = now_us();

    auto translation = FixedMat4<>::translation(Vec3(0.0f, 0.0f, -4.0f));
    auto scale_mat = FixedMat4<>::scale({scale, scale, scale});
    auto rot_y = FixedMat4<>::rotation(ang, Vec3{0.0f, 1.0f, 0.0f});
    auto rot_x = FixedMat4<>::rotation(ang2, Vec3{1.0f, 0.0f, 0.0f});
    r3d.set_model_view(translation * scale_mat * rot_y * rot_x);

    [[maybe_unused]]
    auto vert_start = now_us();

    r3d.set_position_shader(fixed32_mvp_pos_shader);
    r3d.set_vertex_shader(model_lit_shader);
    model.draw(r3d);

    [[maybe_unused]]
    auto vert_end = now_us();

    [[maybe_unused]]
    auto vertex_count = r3d.get_transformed_vertex_count();

    [[maybe_unused]]
    auto frag_start = now_us();

    r3d.rasterise();

    [[maybe_unused]]
    auto frag_end = now_us();

    // debug stats
    /*
    screen.pen = {255, 0, 0};
    char buf[100];
    snprintf(buf, sizeof(buf), "V %6" PRIi32 "us\nF %6" PRIi32 "us\nR %6" PRIi32 "us\nT %6i",
        us_diff(vert_start, vert_end),
        us_diff(frag_start, frag_end),
        us_diff(render_start, frag_end),
        vertex_count
    );

    screen.text(buf, minimal_font, {0, 0}, false);
    */
}