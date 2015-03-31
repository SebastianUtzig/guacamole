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
#include <gua/renderer/Pipeline.hpp>

// guacamole headers
#include <gua/config.hpp>
#include <gua/node/CameraNode.hpp>
#include <gua/renderer/GBuffer.hpp>
#include <gua/renderer/WindowBase.hpp>
#include <gua/renderer/GeometryResource.hpp>
#include <gua/databases/WindowDatabase.hpp>
#include <gua/databases/TextureDatabase.hpp>
#include <gua/renderer/Frustum.hpp>
#include <gua/node/CameraNode.hpp>
#include <gua/node/LightNode.hpp>
#include <gua/scenegraph/SceneGraph.hpp>

#include <gua/renderer/CameraUniformBlock.hpp>
#include <gua/renderer/LightTable.hpp>

// external headers
#include <iostream>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

  Pipeline::Pipeline(RenderContext& ctx, math::vec2ui const& resolution)
    : context_(ctx),
      gbuffer_(new GBuffer(ctx, resolution)),
      camera_block_(ctx.render_device),
      light_table_(new LightTable),
      current_graph_(nullptr),
      current_scene_(nullptr),
      current_camera_(),
      last_resolution_(0, 0),
      last_description_(),
      global_substitution_map_(),
      passes_(),
      quad_( new scm::gl::quad_geometry(ctx.render_device,
                                 scm::math::vec2f(-1.f, -1.f),
                                 scm::math::vec2f(1.f, 1.f)))
  {

  const float th = last_description_.get_blending_termination_threshold();
  global_substitution_map_["enable_abuffer"] = "0";
  global_substitution_map_["abuf_insertion_threshold"] = std::to_string(th);
  global_substitution_map_["abuf_blending_termination_threshold"] =
      std::to_string(th);
    global_substitution_map_["max_lights_num"] =
      std::to_string(last_description_.get_max_lights_count());

    for (auto pass : last_description_.get_passes()) {
      passes_.push_back(pass->make_pass(ctx, global_substitution_map_));
    }
  }

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<Texture2D> Pipeline::render_scene(
    CameraMode mode,
    node::SerializedCameraNode const& camera,
    std::vector<std::unique_ptr<const SceneGraph> > const& scene_graphs) {
  // return if pipeline is disabled
  if (!camera.config.get_enabled()) {
    return std::shared_ptr<Texture2D>();
  }

    // store the current camera data
    current_camera_ = camera;

    bool reload_gbuffer(false);
    bool reload_abuffer(false);

    // execute all prerender cameras
    for (auto const& cam : camera.pre_render_cameras) {
      if (!context_.render_pipelines.count(cam.uuid)) {
        context_.render_pipelines.insert(
          std::make_pair(cam.uuid, std::make_shared<Pipeline>(context_, camera.config.get_resolution())));
      }
      context_.render_pipelines.at(cam.uuid)->render_scene(mode, cam, scene_graphs);
    }

    // recreate gbuffer if resolution changed
    if (last_resolution_ != camera.config.get_resolution()) {
      last_resolution_ = camera.config.get_resolution();
      reload_gbuffer = true;
    }

    if (reload_gbuffer) {
      if (gbuffer_) {
        gbuffer_->remove_buffers(get_context());
      }

      math::vec2ui new_gbuf_size(std::max(1U, camera.config.resolution().x), std::max(1U, camera.config.resolution().y));
      gbuffer_.reset(new GBuffer(get_context(), new_gbuf_size));
    }


    // recreate pipeline passes if pipeline description changed
    bool reload_passes(reload_gbuffer);

    if (*camera.pipeline_description != last_description_) {
      reload_passes = true;
      reload_abuffer = true;
      last_description_ = *camera.pipeline_description;
    }
    else {
      // if pipeline configuration is unchanged, update only uniforms of passes
      for (unsigned i(0); i < last_description_.get_passes().size(); ++i) {
        last_description_.get_passes()[i]->uniforms =
          camera.pipeline_description->get_passes()[i]->uniforms;
      }
    }

    if (reload_abuffer) {
      gbuffer_->allocate_a_buffer(context_, last_description_.get_abuffer_size());
    }

    if (reload_passes) {
      for (auto& pass : passes_) {
        pass.on_delete(this);
      }

      passes_.clear();
      global_substitution_map_.clear();

      const float th = last_description_.get_blending_termination_threshold();
      global_substitution_map_["enable_abuffer"] =
        last_description_.get_enable_abuffer() ? "1" : "0";
      global_substitution_map_["abuf_insertion_threshold"] = std::to_string(th);
      global_substitution_map_["abuf_blending_termination_threshold"] =
        std::to_string(th);
      global_substitution_map_["max_lights_num"] =
        std::to_string(last_description_.get_max_lights_count());

      for (auto pass : last_description_.get_passes()) {
        passes_.push_back(pass->make_pass(context_, global_substitution_map_));
      }
    }

    // get scenegraph which shall be rendered
    current_graph_ = nullptr;
    for (auto& graph : scene_graphs) {
      if (graph->get_name() == camera.config.get_scene_graph_name()) {
        current_graph_ = graph.get();
        break;
      }
    }

    context_.mode = mode;

    // serialize this scenegraph
    current_scene_ = current_graph_->serialize(camera, mode);

  camera_block_.update(context_.render_context,
                        current_scene_->frustum,
                        current_scene_->clipping_planes,
                        camera.config.get_view_id(),
                        camera.config.get_resolution());
  bind_camera_uniform_block(0);

  // clear gbuffer
  gbuffer_->clear(context_);

  current_target_ = gbuffer_.get();

    // process all passes
    for (unsigned i(0); i < passes_.size(); ++i) {
      if (passes_[i].needs_color_buffer_as_input()) {
        gbuffer_->toggle_ping_pong();
      }
    passes_[i].process(*last_description_.get_passes()[i], *this, false);
    }


#ifdef GUACAMOLE_ENABLE_PIPELINE_PASS_TIME_QUERIES
    fetch_gpu_query_results(context_);

    if (context_.framecount % 60 == 0) {
      std::cout << "===== Time Queries for Context: " << context_.id
        << " ============================" << std::endl;
      for (auto const& t : queries_.results) {
        std::cout << t.first << " : " << t.second << " ms" << std::endl;
      }
      queries_.results.clear();
      std::cout << ">>===================================================="
        << std::endl;
    }
#endif

    gbuffer_->toggle_ping_pong();

  // add texture to texture database
  auto const& tex(gbuffer_->get_color_buffer());
  auto tex_name(camera.config.get_output_texture_name());


    if (tex_name != "") {
      TextureDatabase::instance()->add(tex_name, tex);
    }

  return tex;
}


////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<Texture2D> Pipeline::render_shadow_map(node::LightNode* light) {

  if (!shadow_map_res_) {
    shadow_map_res_ = context_.resources.get<SharedShadowMapResource>();
  }

  auto& current_mask(current_camera_.config.mask());

  // has the shadow map been rendered this frame already?
  auto cached_shadow_map(shadow_map_res_->used_shadow_maps.find(light));

  if (cached_shadow_map != shadow_map_res_->used_shadow_maps.end() && cached_shadow_map->second.render_mask == current_mask) {
    return cached_shadow_map->second.shadow_map->get_depth_buffer();
  }

  // if not, find an unused shadow map with the correct size
  std::shared_ptr<ShadowMap> shadow_map;
  unsigned map_size(light->data.shadow_map_size());

  for (auto it(shadow_map_res_->unused_shadow_maps.begin()); it != shadow_map_res_->unused_shadow_maps.end(); ++it) {
    if ((*it)->get_width() == map_size) {
      shadow_map = *it;
      shadow_map_res_->unused_shadow_maps.erase(it);
      break;
    }
  } 

  // if there is none, create a new one
  if (!shadow_map) {
    shadow_map = std::make_shared<ShadowMap>(context_, map_size);
  }

  // store shadow map
  shadow_map_res_->used_shadow_maps[light] = {shadow_map, current_mask};

  current_target_ = shadow_map.get();
  auto orig_scene(current_scene_);

  //   // calculate light frustum
  math::mat4 screen_transform(scm::math::make_translation(0., 0., -1.));
  screen_transform = light->get_cached_world_transform() * screen_transform;

  Frustum frustum = Frustum::perspective(
    light->get_cached_world_transform(), screen_transform,
    current_camera_.config.near_clip(), current_camera_.config.far_clip()
  );

  current_scene_ = current_graph_->serialize(frustum, 
                                             current_camera_.config.enable_frustum_culling(),
                                             current_camera_.config.mask());

  camera_block_.update(context_.render_context,
                       current_scene_->frustum,
                       current_scene_->clipping_planes,
                       current_camera_.config.get_view_id(),
                       math::vec2ui(map_size, map_size));
  bind_camera_uniform_block(0);

  // clear shadow map
  shadow_map->clear(context_);

  // process all passes
  for (int i(0); i < passes_.size(); ++i) {
    if (passes_[i].enable_for_shadows()) {
      passes_[i].process(*last_description_.get_passes()[i], *this, true);
    }
  }


  // restore previous configuration

  current_target_ = gbuffer_.get();
  current_scene_ = orig_scene;

  camera_block_.update(context_.render_context,
                       current_scene_->frustum,
                       current_scene_->clipping_planes,
                       current_camera_.config.get_view_id(),
                       current_camera_.config.get_resolution());
  bind_camera_uniform_block(0);

  return shadow_map->get_depth_buffer();
}


  ////////////////////////////////////////////////////////////////////////////////

RenderTarget& Pipeline::get_current_target() const {
  return *current_target_;
}


  ////////////////////////////////////////////////////////////////////////////////

node::SerializedCameraNode const& Pipeline::get_scene_camera() const {
  return current_camera_;
}


  ////////////////////////////////////////////////////////////////////////////////

  RenderContext const& Pipeline::get_context() const {
    return context_;
  }

  ////////////////////////////////////////////////////////////////////////////////

SerializedScene& Pipeline::get_scene() {
  return *current_scene_;
}


  ////////////////////////////////////////////////////////////////////////////////

  SceneGraph const& Pipeline::get_graph() const {
    return *current_graph_;
  }

  ////////////////////////////////////////////////////////////////////////////////

  LightTable& Pipeline::get_light_table() {
    return *light_table_;
  }

  ////////////////////////////////////////////////////////////////////////////////

  void Pipeline::bind_gbuffer_input(
    std::shared_ptr<ShaderProgram> const& shader) const {

    shader->set_uniform(context_, 1.0f / gbuffer_->get_width(), "gua_texel_width");
    shader->set_uniform(context_, 1.0f / gbuffer_->get_height(), "gua_texel_height");

    shader->set_uniform(
      context_,
      math::vec2i(gbuffer_->get_width(), gbuffer_->get_height()),
      "gua_resolution");

  shader->set_uniform(context_,
                      gbuffer_->get_color_buffer()->get_handle(context_),
                      "gua_gbuffer_color");
  shader->set_uniform(context_,
                      gbuffer_->get_pbr_buffer()->get_handle(context_),
                      "gua_gbuffer_pbr");
  shader->set_uniform(context_,
                      gbuffer_->get_normal_buffer()->get_handle(context_),
                      "gua_gbuffer_normal");
  shader->set_uniform(context_,
                      gbuffer_->get_flags_buffer()->get_handle(context_),
                      "gua_gbuffer_flags");
  shader->set_uniform(context_,
                      gbuffer_->get_depth_buffer()->get_handle(context_),
                      "gua_gbuffer_depth");
}

  ////////////////////////////////////////////////////////////////////////////////

  void Pipeline::bind_light_table(
    std::shared_ptr<ShaderProgram> const& shader) const {
    shader->set_uniform(
      context_, int(light_table_->get_lights_num()), "gua_lights_num");
    shader->set_uniform(
      context_, int(light_table_->get_sun_lights_num()), "gua_sun_lights_num");

    if (light_table_->get_light_bitset() && light_table_->get_lights_num() > 0) {
      shader->set_uniform(context_,
        light_table_->get_light_bitset()->get_handle(context_),
        "gua_light_bitset");
      context_.render_context->bind_uniform_buffer(
        light_table_->light_uniform_block().block_buffer(), 1);
    }
  }

  ////////////////////////////////////////////////////////////////////////////////

  void Pipeline::bind_camera_uniform_block(unsigned location) const {
    get_context().render_context->bind_uniform_buffer(
      camera_block_.block().block_buffer(), location);
  }

  ////////////////////////////////////////////////////////////////////////////////

  void Pipeline::draw_quad() {
    quad_->draw(context_.render_context);
  }

  ////////////////////////////////////////////////////////////////////////////////
  void Pipeline::begin_cpu_query(std::string const& query_name) {
#ifdef GUACAMOLE_ENABLE_PIPELINE_PASS_TIME_QUERIES
    std::chrono::steady_clock::time_point start_time =
      std::chrono::steady_clock::now();
    queries_.cpu_queries[query_name] = start_time;
#endif
  }

  ////////////////////////////////////////////////////////////////////////////////
  void Pipeline::end_cpu_query(std::string const& query_name) {
#ifdef GUACAMOLE_ENABLE_PIPELINE_PASS_TIME_QUERIES
    assert(queries_.cpu_queries.count(query_name));

    std::chrono::steady_clock::time_point end_time =
      std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point start_time =
      queries_.cpu_queries.at(query_name);

    double mcs = std::chrono::duration_cast<std::chrono::microseconds>(
      end_time - start_time).count();
    queries_.results[query_name] = mcs / 1000.0;
#endif
  }

  ////////////////////////////////////////////////////////////////////////////////

  void Pipeline::begin_gpu_query(RenderContext const& ctx,
    std::string const& name) {
#ifdef GUACAMOLE_ENABLE_PIPELINE_PASS_TIME_QUERIES

    if (ctx.framecount < 50) {
      queries_.gpu_queries.clear();
      return;
    }

    auto existing_query = queries_.gpu_queries.find(name);

    if (existing_query != queries_.gpu_queries.end()) {
      // delete existing query if it is too old
      const unsigned max_wait_frames = 50;
      if (existing_query->second.collect_attempts > max_wait_frames) {
        queries_.gpu_queries.erase(existing_query);
      }
      else {
        // existing query in process -> nothing to be done!!! -> return!!
        return;
      }
    }

    try {
      // create query
      auto query = ctx.render_device->create_timer_query();
      query_dispatch dispatch = { query, false, 0U };

      queries_.gpu_queries.insert(std::make_pair(name, dispatch));
      ctx.render_context->begin_query(query);
    }
    catch (...) {
      // query dispatch failed
    }

#endif
  }

  ////////////////////////////////////////////////////////////////////////////////

  void Pipeline::end_gpu_query(RenderContext const& ctx,
    std::string const& name) {
#ifdef GUACAMOLE_ENABLE_PIPELINE_PASS_TIME_QUERIES
    // query started
    if (queries_.gpu_queries.count(name)) {
      // query not finished yet
      if (!queries_.gpu_queries.at(name).dispatched) {
        ctx.render_context->end_query(queries_.gpu_queries.at(name).query);
        queries_.gpu_queries.at(name).dispatched = true;
      }
      else {
        // no such query
        return;
      }
    }
#endif
  }

////////////////////////////////////////////////////////////////////////////////

void Pipeline::fetch_gpu_query_results(RenderContext const& ctx) {

#ifdef GUACAMOLE_ENABLE_PIPELINE_PASS_TIME_QUERIES
    bool queries_ready = true;
    for (auto& q : queries_.gpu_queries) {
      bool query_ready =
        ctx.render_context->query_result_available(q.second.query);
      ++q.second.collect_attempts;
      queries_ready &= query_ready;
    }

    if (queries_ready) {
      for (auto const& q : queries_.gpu_queries) {
        ctx.render_context->collect_query_results(q.second.query);
        double draw_time_in_ms =
          static_cast<double>(q.second.query->result()) / 1e6;
        queries_.results[q.first] = draw_time_in_ms;
      }

      queries_.gpu_queries.clear();
    }
#endif
}

////////////////////////////////////////////////////////////////////////////////

void Pipeline::clear_frame_cache() {
  if (!shadow_map_res_) {
    shadow_map_res_ = context_.resources.get<SharedShadowMapResource>();
  }

  for (auto& unused: shadow_map_res_->unused_shadow_maps) {
    unused->remove_buffers(context_);
  }
  shadow_map_res_->unused_shadow_maps.clear();

  for (auto& cached: shadow_map_res_->used_shadow_maps) {
    shadow_map_res_->unused_shadow_maps.push_back(cached.second.shadow_map);
  }

  shadow_map_res_->used_shadow_maps.clear();
}

////////////////////////////////////////////////////////////////////////////////


}
