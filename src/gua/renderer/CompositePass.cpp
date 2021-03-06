/******************************************************************************
 * guacamole - delicious VR                                                   *
 *                                                                            *
 * Copyright: (c) 2011-2013 Bauhaus-Universität Weimar                        *
 * Contact:   felix.lauer@uni-weimar.de / simon.schneegans@uni-weimar.de      *
 *                                                                            *
 * This program is free software: you can redistribute it and/or modify it    *
 * under the terms of the GNU General Public License as published by the Free *
 * Software Foundation, either version 3 of the License, or (at your option)  *
 * any later version.                                                         *
 *                                                                            *
 * This program is distributed in the hope that it will be useful, but        *
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY *
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License   *
 * for more details.                                                          *
 *                                                                            *
 * You should have received a copy of the GNU General Public License along    *
 * with this program. If not, see <http://www.gnu.org/licenses/>.             *
 *                                                                            *
 ******************************************************************************/

// class header
#include <gua/renderer/CompositePass.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/StereoBuffer.hpp>
#include <gua/renderer/GBuffer.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/renderer/LayerMapping.hpp>
#include <gua/renderer/Volume.hpp>
#include <gua/renderer/View.hpp>
#include <gua/databases.hpp>

#include <memory>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

CompositePass::CompositePass(Pipeline* pipeline) :
	Pass(pipeline),
  composite_shader_(new ShaderProgram),
  ray_generation_shader_(new ShaderProgram),
  volume_raygeneration_buffer_(nullptr)
{
  std::string vertex_shader (Resources::lookup_shader(Resources::shaders_uber_shaders_composite_compose_vert));
  std::string fragment_shader(Resources::lookup_shader(Resources::shaders_uber_shaders_composite_compose_frag));

  composite_shader_->create_from_sources(vertex_shader, fragment_shader);

  std::string ray_generation_vertex_shader(Resources::lookup_shader(Resources::shaders_uber_shaders_composite_ray_generation_vert));
  std::string ray_generation_fragment_shader(Resources::lookup_shader(Resources::shaders_uber_shaders_composite_ray_generation_frag));

  ray_generation_shader_->create_from_sources(ray_generation_vertex_shader, ray_generation_fragment_shader);
}

////////////////////////////////////////////////////////////////////////////////

CompositePass::~CompositePass() {

  if (composite_shader_) {
    delete composite_shader_;
  }

  if (volume_raygeneration_buffer_) {
    delete volume_raygeneration_buffer_;
  }

  if (ray_generation_shader_) {
    delete ray_generation_shader_;
  }
}

////////////////////////////////////////////////////////////////////////////////

void CompositePass::create(RenderContext const& ctx, std::vector<std::pair<BufferComponent,
    scm::gl::sampler_state_desc>> const& layers) {

  // reuse gbuffer from shading-pass
  gbuffer_ = inputs_[Pipeline::PipelineStage::shading];
  /*scm::gl::sampler_state_desc tmp(scm::gl::FILTER_MIN_MAG_LINEAR,
    scm::gl::WRAP_CLAMP_TO_EDGE,
    scm::gl::WRAP_CLAMP_TO_EDGE);
  Pass::create(ctx, config, { { BufferComponent::F3, tmp } });*/


  if (volume_raygeneration_buffer_) {
    volume_raygeneration_buffer_->remove_buffers(ctx);
    delete volume_raygeneration_buffer_;
  }

  scm::gl::sampler_state_desc state(scm::gl::FILTER_MIN_MAG_LINEAR,
                                    scm::gl::WRAP_CLAMP_TO_EDGE,
                                    scm::gl::WRAP_CLAMP_TO_EDGE);

  std::vector<std::pair<BufferComponent, scm::gl::sampler_state_desc>> layer_3f_desc;
  layer_3f_desc.push_back(std::make_pair(BufferComponent::F3, state));

  volume_raygeneration_buffer_ = new GBuffer(layer_3f_desc,
                                      pipeline_->config.get_left_resolution()[0],
                                      pipeline_->config.get_left_resolution()[1]);
  volume_raygeneration_buffer_->create(ctx);
}

////////////////////////////////////////////////////////////////////////////////

/* virtual */ void CompositePass::rendering(SerializedScene const& scene,
  RenderContext const& ctx,
  CameraMode eye,
  Camera const& camera,
  FrameBufferObject* target)
{
    init_resources(ctx);

    ctx.render_context->set_depth_stencil_state(depth_stencil_state_);

    // 1. render proxy geometry into fbo
    volume_raygeneration_buffer_->bind(ctx);
    {
        scm::math::vec2f resolution(volume_raygeneration_buffer_->width(), volume_raygeneration_buffer_->height());
        ctx.render_context->set_viewport(scm::gl::viewport(math::vec2(0, 0), resolution));

        volume_raygeneration_buffer_->clear_color_buffers(ctx, gua::utils::Color3f(0.0f, 0.0f, 0.0f));
        volume_raygeneration_buffer_->clear_depth_stencil_buffer(ctx);

        ///TODO: Toplevel
        if (!scene.volumenodes_.empty())
        {
            // gather input textures and set uniforms
            ray_generation_shader_->set_uniform(ctx, static_cast<int>(eye), "gua_eye");
            if (eye == CameraMode::LEFT || eye == CameraMode::CENTER) {
              ctx.render_context->bind_uniform_buffer(pipeline_->camera_block_left_->block().block_buffer(), 0);
            } else {
              ctx.render_context->bind_uniform_buffer(pipeline_->camera_block_right_->block().block_buffer(), 0);
            }

            ray_generation_shader_->set_uniform(ctx, 1.f / gbuffer_->get_eye_buffers()[eye == CameraMode::RIGHT ? 1 : 0]->width(), "gua_texel_width");
            ray_generation_shader_->set_uniform(ctx, 1.f / gbuffer_->get_eye_buffers()[eye == CameraMode::RIGHT ? 1 : 0]->height(), "gua_texel_height");


            volume_raygeneration_buffer_->clear_color_buffers(ctx, gua::utils::Color3f(0.0f, 0.0f, 0.0f));
            volume_raygeneration_buffer_->clear_depth_stencil_buffer(ctx);

            for (auto const& node : scene.volumenodes_) {

                auto volume =
                    std::static_pointer_cast<gua::Volume>(GeometryDatabase::instance()->lookup(node->data.get_volume()));

                if (volume) {
                    ray_generation_shader_->set_uniform(
                        ctx, node->get_world_transform(), "gua_model_matrix");

                    ray_generation_shader_->set_uniform(
                        ctx, 0, "volume_frag_id");

                    ray_generation_shader_->use(ctx);
                    {
                        volume->draw_proxy(ctx);
                    }
                    ray_generation_shader_->unuse(ctx);
                }
            }
       }
    }
    volume_raygeneration_buffer_->unbind(ctx);

    scm::gl::context_all_guard      cug(ctx.render_context);

    // 2. render fullscreen quad for compositing and volume ray castinG
    composite_shader_->set_uniform(ctx, static_cast<int>(eye), "gua_eye");
    if (eye == CameraMode::LEFT || eye == CameraMode::CENTER) {
      ctx.render_context->bind_uniform_buffer(pipeline_->camera_block_left_->block().block_buffer(), 0);
    } else {
      ctx.render_context->bind_uniform_buffer(pipeline_->camera_block_right_->block().block_buffer(), 0);
    }

    auto input_tex(inputs_[Pipeline::shading]->get_eye_buffers()[eye == CameraMode::RIGHT ? 1 : 0]->get_color_buffers(TYPE_FLOAT)[0]);
    auto normal_tex(inputs_[Pipeline::geometry]->get_eye_buffers()[eye == CameraMode::RIGHT ? 1 : 0]->get_color_buffers(TYPE_FLOAT)[0]);
    auto depth_tex(inputs_[Pipeline::geometry]->get_eye_buffers()[eye == CameraMode::RIGHT ? 1 : 0]->get_depth_buffer());

    auto raygen_tex(volume_raygeneration_buffer_->get_color_buffers(TYPE_FLOAT)[0]);

    composite_shader_->set_uniform(ctx, input_tex, "gua_color_gbuffer_in");
    composite_shader_->set_uniform(ctx, normal_tex, "gua_normal_gbuffer_in");
    composite_shader_->set_uniform(ctx, depth_tex, "gua_depth_gbuffer_in");
    composite_shader_->set_uniform(ctx, raygen_tex, "gua_ray_entry_in");

    composite_shader_->set_uniform(ctx, 1.f / gbuffer_->get_eye_buffers()[eye == CameraMode::RIGHT ? 1 : 0]->width(), "gua_texel_width");
    composite_shader_->set_uniform(ctx, 1.f / gbuffer_->get_eye_buffers()[eye == CameraMode::RIGHT ? 1 : 0]->height(), "gua_texel_height");

    // bind target fbo and set viewport
    target->bind(ctx);
    ctx.render_context->set_viewport(scm::gl::viewport(
        math::vec2(0, 0),
        ::scm::math::vec2f(target->width(), target->height())));

    if (!scene.volumenodes_.empty()){

        for (auto const& node : scene.volumenodes_) {

            auto volume =
                std::static_pointer_cast<gua::Volume>(GeometryDatabase::instance()->lookup(node->data.get_volume()));

            if (volume) {
                composite_shader_->set_uniform(
                    ctx, node->get_world_transform(), "gua_model_matrix");

                volume->set_uniforms(ctx, composite_shader_);

                composite_shader_->use(ctx);
                {
                    fullscreen_quad_->draw(ctx.render_context);
                }
                composite_shader_->unuse(ctx);
            }
        }

    }

    target->unbind(ctx);

    ctx.render_context->reset_state_objects();
}

////////////////////////////////////////////////////////////////////////////////

void CompositePass::init_resources(RenderContext const& ctx) {
  if (!initialized_) {
    if (!depth_stencil_state_) {
      depth_stencil_state_ = ctx.render_device->create_depth_stencil_state(false, false, scm::gl::COMPARISON_NEVER);
    }

    if (!fullscreen_quad_) {
      fullscreen_quad_ = scm::gl::quad_geometry_ptr(new scm::gl::quad_geometry(ctx.render_device, math::vec2(-1.f, -1.f), math::vec2(1.f, 1.f)));
    }
    initialized_ = true;
  }
}

////////////////////////////////////////////////////////////////////////////////

/* virtual */ LayerMapping const* CompositePass::get_gbuffer_mapping() const {
  throw std::runtime_error("no gbuffer mapping available for composite pass");
}

////////////////////////////////////////////////////////////////////////////////

void CompositePass::print_shaders(std::string const& directory,
                               std::string const& name) const  {
  composite_shader_->save_to_file(directory, name + "/composite_shader");
  ray_generation_shader_->save_to_file(directory, name + "/ray_generation_shader");
}

////////////////////////////////////////////////////////////////////////////////

bool CompositePass::pre_compile_shaders(RenderContext const& ctx) {

  bool success(true);

  if (composite_shader_)      success &= composite_shader_->upload_to(ctx);
  if (ray_generation_shader_) success &= ray_generation_shader_->upload_to(ctx);

  return success;
}

////////////////////////////////////////////////////////////////////////////////

void CompositePass::render_scene(Camera const& camera,
                                 SceneGraph const&,
                                 RenderContext const& ctx,
                                 std::size_t viewid) {
  bool is_stereo = gbuffer_->get_eye_buffers().size() > 1;
  View view(viewid, is_stereo);

  if (is_stereo) // stereo -> todo: combine frusta?
  {
    view.left = pipeline_->get_current_scene(CameraMode::LEFT).frustum;
    view.right = pipeline_->get_current_scene(CameraMode::RIGHT).frustum;
  }
  else {       // mono -> just use left frustum
    view.left = pipeline_->get_current_scene(CameraMode::LEFT).frustum;
  }

  for (int i(0); i < gbuffer_->get_eye_buffers().size(); ++i) 
  {

    FrameBufferObject* fbo(gbuffer_->get_eye_buffers()[i]);

    CameraMode eye;
    if (!is_stereo) {
      eye = CameraMode::CENTER;
    }
    else {
      eye = (i == 0) ? CameraMode::LEFT : CameraMode::RIGHT;
    }

    fbo->bind(ctx);

    ctx.render_context->set_viewport(scm::gl::viewport(
      math::vec2(0, 0), ::scm::math::vec2f(fbo->width(), fbo->height())));

    rendering(pipeline_->get_current_scene(eye), ctx, eye, camera, fbo);

    fbo->unbind(ctx);
  }
}

}
