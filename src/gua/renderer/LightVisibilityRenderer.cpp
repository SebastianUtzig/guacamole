// class header
#include <gua/renderer/LightVisibilityRenderer.hpp>

#include <gua/renderer/Pipeline.hpp>
#include <gua/databases/GeometryDatabase.hpp>
#include <gua/databases/Resources.hpp>
#include <gua/renderer/TriMeshRessource.hpp>
#include <gua/node/LightNode.hpp>
#include <gua/utils/Logger.hpp>

#include <scm/gl_core/render_device/opengl/gl_core.h>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

void LightVisibilityRenderer::render(PipelinePass& pass,
                                     Pipeline& pipe,
                                     int tile_power,
                                     unsigned ms_sample_count,
                                     bool enable_conservative,
                                     bool enable_fullscreen_fallback) {

  auto const& ctx(pipe.get_context());
  auto const& glapi = ctx.render_context->opengl_api();

  std::vector<math::mat4> transforms;
  LightTable::array_type lights;

  unsigned sun_lights_num = 0u;
  prepare_light_table(pipe, transforms, lights, sun_lights_num);
  math::vec2ui effective_resolution =
      pipe.get_light_table().invalidate(ctx, pipe.get_scene_camera().config.get_resolution(),
                                        lights, tile_power, sun_lights_num);

  math::vec2ui rasterizer_resolution = (enable_fullscreen_fallback)
      ? pipe.get_scene_camera().config.get_resolution() : effective_resolution;

  if (!empty_fbo_) {
    empty_fbo_ = ctx.render_device->create_frame_buffer();

    auto const& glapi = ctx.render_context->opengl_api();
    // TODO: ideally, FBOs with no attachments should be implemented in schism
    glapi.glNamedFramebufferParameteriEXT(empty_fbo_->object_id(),
                                          GL_FRAMEBUFFER_DEFAULT_WIDTH,   rasterizer_resolution.x);
    glapi.glNamedFramebufferParameteriEXT(empty_fbo_->object_id(),
                                          GL_FRAMEBUFFER_DEFAULT_HEIGHT,  rasterizer_resolution.y);
    glapi.glNamedFramebufferParameteriEXT(empty_fbo_->object_id(),
                                          GL_FRAMEBUFFER_DEFAULT_SAMPLES, ms_sample_count);
  }
  ctx.render_context->set_frame_buffer(empty_fbo_);
  ctx.render_context->set_viewport(scm::gl::viewport(math::vec2ui(0, 0),
                                                     rasterizer_resolution));

  if (pass.depth_stencil_state_)
    ctx.render_context->set_depth_stencil_state(pass.depth_stencil_state_);
  if (pass.rasterizer_state_)
    ctx.render_context->set_rasterizer_state(pass.rasterizer_state_);

  pass.shader_->use(ctx);
  pipe.bind_light_table(pass.shader_);

  if (enable_conservative) {
    glapi.glEnable(GL_CONSERVATIVE_RASTERIZATION_NV);
  }
  if (ms_sample_count > 0) {
    glapi.glEnable(GL_MULTISAMPLE);
  }

  draw_lights(pipe, transforms, lights);

  if (ms_sample_count > 0) {
    glapi.glDisable(GL_MULTISAMPLE);
  }
  if (enable_conservative) {
    glapi.glDisable(GL_CONSERVATIVE_RASTERIZATION_NV);
  }

  ctx.render_context->reset_state_objects();
}

////////////////////////////////////////////////////////////////////////////////

void LightVisibilityRenderer::prepare_light_table(Pipeline& pipe,
                                                  std::vector<math::mat4>& transforms,
                                                  LightTable::array_type& lights,
                                                  unsigned& sun_lights_num
                                                  ) const {

  sun_lights_num = 0u;
  for (auto const& l : pipe.get_scene().nodes[std::type_index(typeid(node::LightNode))]) {
    auto light(reinterpret_cast<node::LightNode*>(l));

    auto model_mat = light->get_cached_world_transform();

    LightTable::LightBlock light_block {};
    light_block.brightness      = light->data.get_brightness();
    light_block.diffuse_enable  = light->data.get_enable_diffuse_shading();
    light_block.specular_enable = light->data.get_enable_specular_shading();
    light_block.color           = math::vec4f(light->data.get_color().vec3f().r, light->data.get_color().vec3f().g, light->data.get_color().vec3f().b, 0.f);
    light_block.type            = static_cast<unsigned>(light->data.get_type());

    if (light->data.get_enable_shadows()) {
      light_block.shadow_map = pipe.render_shadow_map(light)->get_handle(pipe.get_context());
      light_block.shadow_offset = light->data.get_shadow_offset();
    }

    if (light->data.get_type() == node::LightNode::Type::SUN) {
      ++sun_lights_num;

      math::vec3 light_position = model_mat * math::vec4(0.f, 0.f, 1.f, 0.f);

      light_block.position_and_radius = math::vec4f(light_position.x, light_position.y, light_position.z, 0.f);
      light_block.beam_direction_and_half_angle = math::vec4f(0.f, 0.f, 0.f, 0.f);
      light_block.falloff         = 0.0f;
      light_block.softness        = 0.0f;
      light_block.casts_shadow    = light->data.get_enable_shadows();
    } else if (light->data.get_type() == node::LightNode::Type::POINT) {
      math::vec3 light_position = model_mat * math::vec4(0.f, 0.f, 0.f, 1.f);
      float light_radius = scm::math::length(light_position - math::vec3(model_mat * math::vec4(0.f, 0.f, 1.f, 1.f)));

      light_block.position_and_radius = math::vec4f(light_position.x, light_position.y, light_position.z, light_radius);
      light_block.beam_direction_and_half_angle = math::vec4f(0.f, 0.f, 0.f, 0.f);
      light_block.falloff         = light->data.get_falloff();
      light_block.softness        = 0;
      light_block.casts_shadow    = 0;
    } else if (light->data.get_type() == node::LightNode::Type::SPOT) {
      math::vec3 light_position = model_mat * math::vec4(0.f, 0.f, 0.f, 1.f);
      math::vec3 beam_direction = math::vec3(model_mat * math::vec4(0.f, 0.f, -1.f, 1.f)) - light_position;
      float half_beam_angle = scm::math::dot(scm::math::normalize(math::vec3(model_mat * math::vec4(0.f, 0.5f, -1.f, 0.f))),
                                             scm::math::normalize(beam_direction));
      light_block.position_and_radius = math::vec4f(light_position.x, light_position.y, light_position.z, 0);
      light_block.beam_direction_and_half_angle = math::vec4f(beam_direction.x, beam_direction.y, beam_direction.z, half_beam_angle);
      light_block.falloff         = light->data.get_falloff();
      light_block.softness        = light->data.get_softness();
      light_block.casts_shadow    = 0; //light->data.get_enable_shadows();
    }

    lights.push_back(light_block);
    transforms.push_back(model_mat);
  }
}

////////////////////////////////////////////////////////////////////////////////

void LightVisibilityRenderer::draw_lights(Pipeline& pipe,
                                          std::vector<math::mat4>& transforms,
                                          LightTable::array_type& lights) const {

  auto const& ctx(pipe.get_context());
  auto gl_program(ctx.render_context->current_program());

  // proxy geometries
  auto light_sphere =
      std::dynamic_pointer_cast<TriMeshRessource>(GeometryDatabase::instance()->lookup("gua_light_sphere_proxy"));
  auto light_cone =
      std::dynamic_pointer_cast<TriMeshRessource>(GeometryDatabase::instance()->lookup("gua_light_cone_proxy"));

  // draw lights
  for (size_t i = 0; i < lights.size(); ++i) {
    if (lights[i].type == 2) // skip sun lights
      continue;

    math::mat4f light_transform(transforms[i]);
    gl_program->uniform("gua_model_matrix", 0, light_transform);
    gl_program->uniform("light_id", 0, int(i));
    ctx.render_context->bind_image(pipe.get_light_table().get_light_bitset()->get_buffer(ctx), 
                                   scm::gl::FORMAT_R_32UI, scm::gl::ACCESS_READ_WRITE, 0, 0, 0);
    ctx.render_context->apply();

    if (lights[i].type == 0) // point light
      light_sphere->draw(ctx);
    else if (lights[i].type == 1) // spot light
      light_cone->draw(ctx);
  }
}

////////////////////////////////////////////////////////////////////////////////

}
