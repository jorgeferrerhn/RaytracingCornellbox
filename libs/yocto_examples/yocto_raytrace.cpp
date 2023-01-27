//
// Implementation for Yocto/RayTrace.
//

//
// LICENSE:
//
// Copyright (c) 2016 -- 2021 Fabio Pellacini
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#define YOCTO_LIVE_CODING 1

#include "yocto_raytrace.h"

#include <yocto/yocto_cli.h>
#include <yocto/yocto_color.h>
#include <yocto/yocto_geometry.h>
#include <yocto/yocto_parallel.h>
#include <yocto/yocto_sampling.h>
#include <yocto/yocto_shading.h>
#include <yocto/yocto_shape.h>

#if YOCTO_LIVE_CODING

// -----------------------------------------------------------------------------
// IMPLEMENTATION FOR SCENE EVALUATION
// -----------------------------------------------------------------------------
namespace yocto {

// Generates a ray from a camera for yimg::image plane coordinate uv and
// the lens coordinates luv.
static ray3f eval_camera(const camera_data& camera, const vec2f& uv) {
  auto ql = vec3f{(0.5f - uv.x) * camera.film,
      (uv.y - 0.5f) * camera.film / camera.aspect, camera.lens};
  auto q  = transform_point(camera.frame, ql);
  auto e  = camera.frame.o;
  return {e, -normalize(q - e)};
}

}  // namespace yocto

// -----------------------------------------------------------------------------
// IMPLEMENTATION FOR PATH TRACING
// -----------------------------------------------------------------------------
namespace yocto {

// Evaluate an environment map with texture
vec3f eval_environment_(const scene_data& scene,
    const environment_data& environment, const vec3f& direction) {
  if (environment.emission_tex == invalidid) return environment.emission;
  auto ldirection = transform_direction_inverse(environment.frame, direction);
  auto uv         = vec2f{
      atan2(ldirection.z, ldirection.x) / (2 * pif), acos(ldirection.y) / pif};
  return environment.emission *
         xyz(eval_texture(scene, environment.emission_tex, uv, true));
}

// Raytrace renderer.
static vec4f shade_raytrace(const scene_data& scene, const scene_bvh& bvh,
    const ray3f& ray, int bounce, rng_state& rng,
    const raytrace_params& params) {
  auto intersection = intersect_scene_bvh(bvh, scene, ray);
  if (!intersection.hit) {
    auto radiance = vec3f{0, 0, 0};
    for (auto& environment : scene.environments)
      radiance += eval_environment_(scene, environment, ray.d);
    return {radiance.x, radiance.y, radiance.z, 1};
  }
  auto& instance = scene.instances.at(intersection.instance);
  auto& shape    = scene.shapes.at(instance.shape);
  auto& material = scene.materials.at(instance.material);
  auto  outgoing = -ray.d;
  auto  position = transform_point(instance.frame,
       eval_position(shape, intersection.element, intersection.uv));
  auto  normal   = transform_normal(instance.frame,
         eval_normal(shape, intersection.element, intersection.uv));
  auto  texcoord = eval_texcoord(shape, intersection.element, intersection.uv);
  auto  emission =
      material.emission *
      xyz(eval_texture(scene, material.emission_tex, texcoord, true));
  auto color = material.color *
               xyz(eval_texture(scene, material.color_tex, texcoord, true));
  auto opacity = material.opacity *
                 eval_texture(scene, material.color_tex, texcoord, true).w;
  auto roughness = material.roughness;
  auto exponent  = 2 / pow(roughness, 4);

  auto entering = dot(normal, outgoing) > 0;
  if (!shape.lines.empty()) {
    normal = orthonormalize(outgoing, normal);
  } else {
    if (dot(normal, outgoing) < 0) normal = -normal;
  }

  if (rand1f(rng) < 1 - opacity) {
    return shade_raytrace(
        scene, bvh, {position, ray.d}, bounce + 1, rng, params);
  }

  auto radiance = emission;
  if (bounce > params.bounces) return {radiance.x, radiance.y, radiance.z, 1};

  switch (material.type) {
    case material_type::matte: {
      auto incoming = sample_hemisphere_cos(normal, rand2f(rng));
      radiance += color * xyz(shade_raytrace(scene, bvh, {position, incoming},
                              bounce + 1, rng, params));
    } break;
    case material_type::reflective: {
      auto mnormal  = (roughness == 0) ? normal
                                       : sample_hemisphere_cospower(
                                            exponent, normal, rand2f(rng));
      auto incoming = reflect(outgoing, mnormal);
      radiance += fresnel_schlick(color, mnormal, outgoing) *
                  xyz(shade_raytrace(scene, bvh, {position, incoming},
                      bounce + 1, rng, params));
    } break;
    case material_type::transparent: {
      auto mnormal = (roughness == 0) ? normal
                                      : sample_hemisphere_cospower(
                                            exponent, normal, rand2f(rng));
      if (rand1f(rng) <
          mean(fresnel_schlick({0.04, 0.04, 0.04}, mnormal, outgoing))) {
        auto incoming = reflect(outgoing, mnormal);
        radiance += xyz(shade_raytrace(
            scene, bvh, {position, incoming}, bounce + 1, rng, params));
      } else {
        auto incoming = -outgoing;
        radiance += color * xyz(shade_raytrace(scene, bvh, {position, incoming},
                                bounce + 1, rng, params));
      }
    } break;
    case material_type::glossy: {
      auto mnormal = (roughness == 0) ? normal
                                      : sample_hemisphere_cospower(
                                            exponent, normal, rand2f(rng));
      if (rand1f(rng) <
          mean(fresnel_schlick({0.04, 0.04, 0.04}, mnormal, outgoing))) {
        auto incoming = reflect(outgoing, mnormal);
        radiance += xyz(shade_raytrace(
            scene, bvh, {position, incoming}, bounce + 1, rng, params));
      } else {
        auto incoming = sample_hemisphere_cos(normal, rand2f(rng));
        radiance += color * xyz(shade_raytrace(scene, bvh, {position, incoming},
                                bounce + 1, rng, params));
      }
    } break;
    case material_type::refractive: {
      auto mnormal = (roughness == 0) ? normal
                                      : sample_hemisphere_cospower(
                                            exponent, normal, rand2f(rng));
      if (rand1f(rng) <
          fresnel_dielectric(
              entering ? material.ior : 1 / material.ior, mnormal, outgoing)) {
        auto incoming = reflect(outgoing, mnormal);
        radiance += xyz(shade_raytrace(
            scene, bvh, {position, incoming}, bounce + 1, rng, params));
      } else {
        auto incoming = refract(
            outgoing, normal, entering ? 1 / material.ior : material.ior);
        radiance += color * xyz(shade_raytrace(scene, bvh, {position, incoming},
                                bounce + 1, rng, params));
      }
    } break;
    case material_type::volumetric: {
      // material.scaterring = random_in_unit_sphere()
      auto incoming = sample_hemisphere_cos(normal, rand2f(rng));
      radiance += color * xyz(shade_raytrace(scene, bvh, {position, incoming},
                              bounce + 1, rng, params));
    } break;

    default: break;
  }

  return {radiance.x, radiance.y, radiance.z, 1};
}

// Matte renderer.
static vec4f shade_matte(const scene_data& scene, const scene_bvh& bvh,
    const ray3f& ray, int bounce, rng_state& rng,
    const raytrace_params& params) {
  auto intersection = intersect_scene_bvh(bvh, scene, ray);
  if (!intersection.hit) {
    auto radiance = vec3f{0, 0, 0};
    for (auto& environment : scene.environments)
      radiance += eval_environment_(scene, environment, ray.d);
    return {radiance.x, radiance.y, radiance.z, 1};
  }
  auto& instance = scene.instances.at(intersection.instance);
  auto& shape    = scene.shapes.at(instance.shape);
  auto& material = scene.materials.at(instance.material);
  auto  position = transform_point(instance.frame,
       eval_position(shape, intersection.element, intersection.uv));
  auto  normal   = transform_normal(instance.frame,
         eval_normal(shape, intersection.element, intersection.uv));
  auto  texcoord = eval_texcoord(shape, intersection.element, intersection.uv);
  auto  emission =
      material.emission *
      xyz(eval_texture(scene, material.emission_tex, texcoord, true));
  auto color = material.color *
               xyz(eval_texture(scene, material.color_tex, texcoord, true));

  auto radiance = emission;
  if (bounce > params.bounces) return {radiance.x, radiance.y, radiance.z, 1};

  auto incoming = sample_hemisphere_cos(normal, rand2f(rng));
  radiance += color * xyz(shade_matte(scene, bvh, {position, incoming},
                          bounce + 1, rng, params));

  return {radiance.x, radiance.y, radiance.z, 1};
}

// Eyelight for quick previewing.
static vec4f shade_eyelight(const scene_data& scene, const scene_bvh& bvh,
    const ray3f& ray, int bounce, rng_state& rng,
    const raytrace_params& params) {
  auto intersection = intersect_scene_bvh(bvh, scene, ray);
  if (!intersection.hit) return {0, 0, 0, 0};
  auto& instance = scene.instances.at(intersection.instance);
  auto& shape    = scene.shapes.at(instance.shape);
  auto& material = scene.materials.at(instance.material);
  auto  normal   = transform_normal(instance.frame,
         eval_normal(shape, intersection.element, intersection.uv));
  auto  radiance = material.color * dot(normal, -ray.d);
  return {radiance.x, radiance.y, radiance.z, 1};
}

static vec4f shade_normal(const scene_data& scene, const scene_bvh& bvh,
    const ray3f& ray, int bounce, rng_state& rng,
    const raytrace_params& params) {
  auto intersection = intersect_scene_bvh(bvh, scene, ray);
  if (!intersection.hit) return {0, 0, 0, 0};
  auto& instance = scene.instances.at(intersection.instance);
  auto& shape    = scene.shapes.at(instance.shape);
  auto  normal   = transform_normal(instance.frame,
         eval_normal(shape, intersection.element, intersection.uv));
  auto  color    = srgb_to_rgb(normal * 0.5f + 0.5f);
  return {color.x, color.y, color.z, 1};
}

static vec4f shade_texcoord(const scene_data& scene, const scene_bvh& bvh,
    const ray3f& ray, int bounce, rng_state& rng,
    const raytrace_params& params) {
  // YOUR CODE GOES HERE
  return {};
}

static vec4f shade_color(const scene_data& scene, const scene_bvh& bvh,
    const ray3f& ray, int bounce, rng_state& rng,
    const raytrace_params& params) {
  auto intersection = intersect_scene_bvh(bvh, scene, ray);
  if (!intersection.hit) return {0, 0, 0, 0};
  auto& instance = scene.instances.at(intersection.instance);
  auto& material = scene.materials.at(instance.material);
  auto  color    = material.color;
  return {color.x, color.y, color.z, 1};
}

// Trace a single ray from the camera using the given algorithm.
using raytrace_shader_func = vec4f (*)(const scene_data& scene,
    const scene_bvh& bvh, const ray3f& ray, int bounce, rng_state& rng,
    const raytrace_params& params);
static raytrace_shader_func get_shader(const raytrace_params& params) {
  switch (params.shader) {
    case raytrace_shader_type::raytrace: return shade_raytrace;
    case raytrace_shader_type::matte: return shade_matte;
    case raytrace_shader_type::eyelight: return shade_eyelight;
    case raytrace_shader_type::normal: return shade_normal;
    case raytrace_shader_type::texcoord: return shade_texcoord;
    case raytrace_shader_type::color: return shade_color;
    default: {
      throw std::runtime_error("sampler unknown");
      return nullptr;
    }
  }
}

// Build the bvh acceleration structure.
scene_bvh make_raytrace_bvh(
    const scene_data& scene, const raytrace_params& params) {
  return make_scene_bvh(scene, false, params.noparallel);
}

// Init a sequence of random number generators.
raytrace_state make_raytrace_state(
    const scene_data& scene, const raytrace_params& params) {
  auto& camera = scene.cameras[params.camera];
  auto  state  = raytrace_state{};
  if (camera.aspect >= 1) {
    state.width  = params.resolution;
    state.height = (int)round(params.resolution / camera.aspect);
  } else {
    state.height = params.resolution;
    state.width  = (int)round(params.resolution * camera.aspect);
  }
  state.samples = 0;
  state.image.assign(state.width * state.height, {0, 0, 0, 0});
  state.hits.assign(state.width * state.height, 0);
  state.rngs.assign(state.width * state.height, {});
  auto rng_ = make_rng(1301081);
  for (auto& rng : state.rngs) {
    rng = make_rng(961748941ull, rand1i(rng_, 1 << 31) / 2 + 1);
  }
  return state;
}

// Progressively compute an image by calling trace_samples multiple times.
void raytrace_samples(raytrace_state& state, const scene_data& scene,
    const scene_bvh& bvh, const raytrace_params& params) {
  // YOUR CODE GOES HERE
  auto& camera = scene.cameras[params.camera];
  auto  shader = get_shader(params);
  if (params.noparallel) {
    for (auto j : range(state.height)) {
      for (auto i : range(state.width)) {
        auto  index = j * state.width + i;
        auto& rng   = state.rngs[index];
        auto  uv    = vec2f{
            (i + rand1f(rng)) / state.width, (j + rand1f(rng)) / state.height};
        auto ray   = eval_camera(camera, uv);
        auto color = shader(scene, bvh, ray, 0, rng, params);
        state.image[index] += color;
        state.hits[index] += color.w > 0 ? 1 : 0;
      }
    }
  } else {
    parallel_for(state.width, state.height, [&](int i, int j) {
      auto  index = j * state.width + i;
      auto& rng   = state.rngs[index];
      auto  uv    = vec2f{
          (i + rand1f(rng)) / state.width, (j + rand1f(rng)) / state.height};
      auto ray   = eval_camera(camera, uv);
      auto color = shade_color(scene, bvh, ray, params.bounces, rng, params);
      state.image[index] += color;
      state.hits[index] += color.w > 0 ? 1 : 0;
    });
  }
  state.samples += 1;
}

// Trace context
raytrace_context make_raytrace_context(const raytrace_params& params) {
  return {{}, false, false};
}

// Async start
void raytrace_start(raytrace_context& context, raytrace_state& state,
    const scene_data& scene, const scene_bvh& bvh,
    const raytrace_params& params) {
  if (state.samples >= params.samples) return;
  context.stop   = false;
  context.done   = false;
  context.worker = std::async(std::launch::async, [&]() {
    if (context.stop) return;
    auto& camera = scene.cameras[params.camera];
    auto  shader = get_shader(params);
    parallel_for(state.width, state.height, [&](int i, int j) {
      if (context.stop) return;
      auto idx      = j * state.width + i;
      auto u        = (i + rand1f(state.rngs[idx])) / state.width,
           v        = (j + rand1f(state.rngs[idx])) / state.height;
      auto ray      = eval_camera(camera, {u, v});
      auto radiance = shader(scene, bvh, ray, 0, state.rngs[idx], params);
      if (!isfinite(radiance)) radiance = {0, 0, 0, 0};
      state.image[idx] += radiance;
      state.hits[idx] += 1;
    });
    state.samples += 1;
    if (context.stop) return;
    context.done = true;
  });
}

// Async cancel
void raytrace_cancel(raytrace_context& context) {
  context.stop = true;
  if (context.worker.valid()) context.worker.get();
}

// Async done
bool raytrace_done(const raytrace_context& context) { return context.done; }

void raytrace_preview(color_image& image, raytrace_context& context,
    raytrace_state& state, const scene_data& scene, const scene_bvh& bvh,
    const raytrace_params& params) {
  // preview
  auto pparams = params;
  pparams.resolution /= params.pratio;
  pparams.samples = 1;
  auto pstate     = make_raytrace_state(scene, pparams);
  raytrace_samples(pstate, scene, bvh, pparams);
  auto preview = get_image(pstate);
  for (auto idx = 0; idx < state.width * state.height; idx++) {
    auto i = idx % image.width, j = idx / image.width;
    auto pi           = clamp(i / params.pratio, 0, preview.width - 1),
         pj           = clamp(j / params.pratio, 0, preview.height - 1);
    image.pixels[idx] = preview.pixels[pj * preview.width + pi];
  }
}

// Check image type
static void check_image(
    const color_image& image, int width, int height, bool linear) {
  if (image.width != width || image.height != height)
    throw std::invalid_argument{"image should have the same size"};
  if (image.linear != linear)
    throw std::invalid_argument{
        linear ? "expected linear image" : "expected srgb image"};
}

// Get resulting render
color_image get_image(const raytrace_state& state) {
  auto image = make_image(state.width, state.height, true);
  get_image(image, state);
  return image;
}
void get_image(color_image& image, const raytrace_state& state) {
  check_image(image, state.width, state.height, true);
  auto scale = 1.0f / (float)state.samples;
  for (auto idx = 0; idx < state.width * state.height; idx++) {
    image.pixels[idx] = state.image[idx] * scale;
  }
}

}  // namespace yocto

#else

// -----------------------------------------------------------------------------
// IMPLEMENTATION FOR SCENE EVALUATION
// -----------------------------------------------------------------------------
namespace yocto {

// Generates a ray from a camera for yimg::image plane coordinate uv and
// the lens coordinates luv.
static ray3f eval_camera(const camera_data& camera, const vec2f& uv) {
  auto film = camera.aspect >= 1
                  ? vec2f{camera.film, camera.film / camera.aspect}
                  : vec2f{camera.film * camera.aspect, camera.film};
  auto q    = transform_point(camera.frame,
         {film.x * (0.5f - uv.x), film.y * (uv.y - 0.5f), camera.lens});
  auto e    = transform_point(camera.frame, {0, 0, 0});
  return {e, normalize(e - q)};
}

}  // namespace yocto

// -----------------------------------------------------------------------------
// IMPLEMENTATION FOR PATH TRACING
// -----------------------------------------------------------------------------
namespace yocto {

// Raytrace renderer.
static vec4f shade_raytrace(const scene_data& scene, const scene_bvh& bvh,
    const ray3f& ray, int bounce, rng_state& rng,
    const raytrace_params& params) {
  // intersect next point
  auto intersection = intersect_scene_bvh(bvh, scene, ray);
  if (!intersection.hit) {
    auto radiance = eval_environment(scene, ray.d);
    return {radiance.x, radiance.y, radiance.z,
        scene.environments.empty() ? 0.0f : 1.0f};
  }

  // evaluate geometry
  auto& instance = scene.instances[intersection.instance];
  auto  position = eval_position(
      scene, instance, intersection.element, intersection.uv);
  auto normal = eval_normal(
      scene, instance, intersection.element, intersection.uv);
  auto texcoord = eval_texcoord(
      scene, instance, intersection.element, intersection.uv);
  auto outgoing = -ray.d;

  // normal corrections
  auto& shape = scene.shapes[instance.shape];
  if (!shape.triangles.empty() && dot(normal, outgoing) < 0) normal = -normal;
  if (!shape.lines.empty()) normal = orthonormalize(outgoing, normal);

  // evaluate material
  auto& material     = scene.materials[instance.material];
  auto  emission_tex = eval_texture(
      scene, material.emission_tex, texcoord, true);
  auto color_tex = eval_texture(scene, material.color_tex, texcoord, true);
  auto emission  = material.emission * xyz(emission_tex);
  auto color     = material.color * xyz(color_tex);
  auto roughness = material.roughness * material.roughness;
  auto opacity   = material.opacity * color_tex.w;

  // handle opacity
  if (rand1f(rng) >= opacity) {
    return shade_raytrace(
        scene, bvh, {position, ray.d}, bounce + 1, rng, params);
  }

  // accumulate emission
  auto radiance = emission;

  // exit if ray is done
  if (bounce >= params.bounces) return {radiance.x, radiance.y, radiance.z, 1};

  // compute illumination
  if (material.type == material_type::matte) {
    auto incoming = sample_hemisphere_cos(normal, rand2f(rng));
    auto lighting = xyz(shade_raytrace(
        scene, bvh, {position, incoming}, bounce + 1, rng, params));
    radiance += color * lighting;
  } else if (material.type == material_type::transparent) {
    auto fresnel = fresnel_schlick(vec3f{0.04, 0.04, 0.04}, normal, outgoing);
    if (rand1f(rng) < mean(fresnel)) {
      auto incoming = reflect(outgoing, normal);
      auto lighting = xyz(shade_raytrace(
          scene, bvh, {position, incoming}, bounce + 1, rng, params));
      radiance += lighting;
    } else {
      auto incoming = -outgoing;
      auto lighting = xyz(shade_raytrace(
          scene, bvh, {position, incoming}, bounce + 1, rng, params));
      radiance += color * lighting;
    }
  } else if (material.type == material_type::reflective && roughness == 0) {
    auto incoming = reflect(outgoing, normal);
    auto lighting = xyz(shade_raytrace(
        scene, bvh, {position, incoming}, bounce + 1, rng, params));
    radiance += fresnel_schlick(color, normal, outgoing) * lighting;
  } else if (material.type == material_type::reflective && roughness > 0) {
    auto exponent   = 2 / (roughness * roughness);
    auto microfacet = sample_hemisphere_cospower(exponent, normal, rand2f(rng));
    auto incoming   = reflect(outgoing, microfacet);
    auto lighting   = xyz(shade_raytrace(
        scene, bvh, {position, incoming}, bounce + 1, rng, params));
    radiance += color * lighting;
  } else if (material.type == material_type::glossy && roughness == 0) {
    auto fresnel = fresnel_schlick(vec3f{0.04, 0.04, 0.04}, normal, outgoing);
    if (rand1f(rng) < mean(fresnel)) {
      auto incoming = reflect(outgoing, normal);
      auto lighting = xyz(shade_raytrace(
          scene, bvh, {position, incoming}, bounce + 1, rng, params));
      radiance += lighting;
    } else {
      auto incoming = sample_hemisphere_cos(normal, rand2f(rng));
      auto lighting = xyz(shade_raytrace(
          scene, bvh, {position, incoming}, bounce + 1, rng, params));
      radiance += color * lighting;
    }
  } else if (material.type == material_type::glossy && roughness > 0) {
    auto fresnel = fresnel_schlick(vec3f{0.04, 0.04, 0.04}, normal, outgoing);
    if (rand1f(rng) < mean(fresnel)) {
      auto exponent   = 2 / (roughness * roughness);
      auto microfacet = sample_hemisphere_cospower(
          exponent, normal, rand2f(rng));
      auto incoming = reflect(outgoing, microfacet);
      auto lighting = xyz(shade_raytrace(
          scene, bvh, {position, incoming}, bounce + 1, rng, params));
      radiance += lighting;
    } else {
      auto incoming = sample_hemisphere_cos(normal, rand2f(rng));
      auto lighting = xyz(shade_raytrace(
          scene, bvh, {position, incoming}, bounce + 1, rng, params));
      radiance += color * lighting;
    }
  }

  // done
  return {radiance.x, radiance.y, radiance.z, 1};
}

// Matte renderer.
static vec4f shade_matte(const scene_data& scene, const scene_bvh& bvh,
    const ray3f& ray, int bounce, rng_state& rng,
    const raytrace_params& params) {
  // intersect next point
  auto intersection = intersect_scene_bvh(bvh, scene, ray);
  if (!intersection.hit) {
    auto radiance = eval_environment(scene, ray.d);
    return {radiance.x, radiance.y, radiance.z,
        scene.environments.empty() ? 0.0f : 1.0f};
  }

  // evaluate geometry
  auto& instance = scene.instances[intersection.instance];
  auto  position = eval_position(
      scene, instance, intersection.element, intersection.uv);
  auto normal = eval_normal(
      scene, instance, intersection.element, intersection.uv);
  auto texcoord = eval_texcoord(
      scene, instance, intersection.element, intersection.uv);

  // evaluate material
  auto& material     = scene.materials[instance.material];
  auto  emission_tex = eval_texture(
      scene, material.emission_tex, texcoord, true);
  auto color_tex = eval_texture(scene, material.color_tex, texcoord, true);
  auto emission  = material.emission * xyz(emission_tex);
  auto color     = material.color * xyz(color_tex);

  // accumulate emission
  auto radiance = emission;

  // exit if ray is done
  if (bounce >= params.bounces) return {radiance.x, radiance.y, radiance.z, 1};

  // compute illumination
  auto incoming = sample_hemisphere_cos(normal, rand2f(rng));
  auto lighting = xyz(shade_raytrace(
      scene, bvh, {position, incoming}, bounce + 1, rng, params));
  radiance += color * lighting;

  // done
  return {radiance.x, radiance.y, radiance.z, 1};
}

// Eyelight for quick previewing.
static vec4f shade_eyelight(const scene_data& scene, const scene_bvh& bvh,
    const ray3f& ray, int bounce, rng_state& rng,
    const raytrace_params& params) {
  // intersect next point
  auto intersection = intersect_scene_bvh(bvh, scene, ray);
  if (!intersection.hit) {
    auto radiance = eval_environment(scene, ray.d);
    return {radiance.x, radiance.y, radiance.z,
        scene.environments.empty() ? 0.0f : 1.0f};
  }

  // evaluate geometry
  auto& instance = scene.instances[intersection.instance];
  auto  normal   = eval_normal(
      scene, instance, intersection.element, intersection.uv);
  auto texcoord = eval_texcoord(
      scene, instance, intersection.element, intersection.uv);
  auto outgoing = -ray.d;

  // evaluate material
  auto& material = scene.materials[instance.material];
  auto  emission = material.emission *
                  xyz(eval_texture(scene, material.emission_tex, texcoord));
  auto color = material.color *
               xyz(eval_texture(scene, material.color_tex, texcoord));

  auto radiance = emission + abs(dot(normal, outgoing)) * color;

  // add simple shading
  return {radiance.x, radiance.y, radiance.z, 1};
}

static vec4f shade_normal(const scene_data& scene, const scene_bvh& bvh,
    const ray3f& ray, int bounce, rng_state& rng,
    const raytrace_params& params) {
  // intersect next point
  auto intersection = intersect_scene_bvh(bvh, scene, ray);
  if (!intersection.hit) return {0, 0, 0, 0};

  // prepare shading point
  auto& instance = scene.instances[intersection.instance];
  auto  normal   = eval_normal(
      scene, instance, intersection.element, intersection.uv);

  auto radiance = normal * 0.5f + 0.5f;

  // return color
  return {radiance.x, radiance.y, radiance.z, 1};
}

static vec4f shade_texcoord(const scene_data& scene, const scene_bvh& bvh,
    const ray3f& ray, int bounce, rng_state& rng,
    const raytrace_params& params) {
  // intersect next point
  auto intersection = intersect_scene_bvh(bvh, scene, ray);
  if (!intersection.hit) return {0, 0, 0, 0};

  // prepare shading point
  auto& instance = scene.instances[intersection.instance];
  auto  texcoord = eval_texcoord(
      scene, instance, intersection.element, intersection.uv);
  texcoord = {fmod(texcoord.x, 1), fmod(texcoord.y, 1)};

  // return color
  return {texcoord.x, texcoord.y, 0, 1};
}

static vec4f shade_color(const scene_data& scene, const scene_bvh& bvh,
    const ray3f& ray, int bounce, rng_state& rng,
    const raytrace_params& params) {
  // intersect next point
  auto intersection = intersect_scene_bvh(bvh, scene, ray);
  if (!intersection.hit) return {0, 0, 0, 0};

  // prepare shading point
  auto& instance = scene.instances[intersection.instance];
  auto& material = scene.materials[instance.material];
  auto  color    = material.color;

  // return color
  return {color.x, color.y, color.z, 1};
}

// Trace a single ray from the camera using the given algorithm.
using raytrace_shader_func = vec4f (*)(const scene_data& scene,
    const scene_bvh& bvh, const ray3f& ray, int bounce, rng_state& rng,
    const raytrace_params& params);
static raytrace_shader_func get_shader(const raytrace_params& params) {
  switch (params.shader) {
    case raytrace_shader_type::raytrace: return shade_raytrace;
    case raytrace_shader_type::matte: return shade_matte;
    case raytrace_shader_type::eyelight: return shade_eyelight;
    case raytrace_shader_type::normal: return shade_normal;
    case raytrace_shader_type::texcoord: return shade_texcoord;
    case raytrace_shader_type::color: return shade_color;
    default: {
      throw std::runtime_error("sampler unknown");
      return nullptr;
    }
  }
}

// Trace a block of samples
void raytrace_sample(raytrace_state& state, const scene_data& scene,
    const scene_bvh& bvh, int i, int j, const raytrace_params& params) {
  auto& camera   = scene.cameras[params.camera];
  auto  shader   = get_shader(params);
  auto  idx      = state.width * j + i;
  auto  uv       = params.samples == 1
                       ? vec2f{(i + 0.5f) / state.width, (j + 0.5f) / state.height}
                       : vec2f{(i + rand1f(state.rngs[idx])) / state.width,
                      (j + rand1f(state.rngs[idx])) / state.height};
  auto  ray      = eval_camera(camera, uv);
  auto  radiance = shader(scene, bvh, ray, 0, state.rngs[idx], params);
  if (!isfinite(radiance)) radiance = {0, 0, 0};
  state.image[idx] += radiance;
  state.hits[idx] += 1;
}

// Build the bvh acceleration structure.
scene_bvh make_raytrace_bvh(
    const scene_data& scene, const raytrace_params& params) {
  return make_scene_bvh(scene, false, params.noparallel);
}

// Init a sequence of random number generators.
raytrace_state make_raytrace_state(
    const scene_data& scene, const raytrace_params& params) {
  auto& camera = scene.cameras[params.camera];
  auto  state  = raytrace_state{};
  if (camera.aspect >= 1) {
    state.width  = params.resolution;
    state.height = (int)round(params.resolution / camera.aspect);
  } else {
    state.height = params.resolution;
    state.width  = (int)round(params.resolution * camera.aspect);
  }
  state.samples = 0;
  state.image.assign(state.width * state.height, {0, 0, 0, 0});
  state.hits.assign(state.width * state.height, 0);
  state.rngs.assign(state.width * state.height, {});
  auto rng_ = make_rng(1301081);
  for (auto& rng : state.rngs) {
    rng = make_rng(961748941ull, rand1i(rng_, 1 << 31) / 2 + 1);
  }
  return state;
}

// Progressively compute an image by calling trace_samples multiple times.
void raytrace_samples(raytrace_state& state, const scene_data& scene,
    const scene_bvh& bvh, const raytrace_params& params) {
  if (state.samples >= params.samples) return;
  auto& camera = scene.cameras[params.camera];
  auto  shader = get_shader(params);
  state.samples += 1;
  if (params.samples == 1) {
    for (auto idx = 0; idx < state.width * state.height; idx++) {
      auto i = idx % state.width, j = idx / state.width;
      auto u = (i + 0.5f) / state.width, v = (j + 0.5f) / state.height;
      auto ray      = eval_camera(camera, {u, v});
      auto radiance = shader(scene, bvh, ray, 0, state.rngs[idx], params);
      if (!isfinite(radiance)) radiance = {0, 0, 0};
      state.image[idx] += radiance;
      state.hits[idx] += 1;
    }
  } else if (params.noparallel) {
    for (auto idx = 0; idx < state.width * state.height; idx++) {
      auto i = idx % state.width, j = idx / state.width;
      auto u        = (i + rand1f(state.rngs[idx])) / state.width,
           v        = (j + rand1f(state.rngs[idx])) / state.height;
      auto ray      = eval_camera(camera, {u, v});
      auto radiance = shader(scene, bvh, ray, 0, state.rngs[idx], params);
      if (!isfinite(radiance)) radiance = {0, 0, 0};
      state.image[idx] += radiance;
      state.hits[idx] += 1;
    }
  } else {
    parallel_for(state.width * state.height, [&](int idx) {
      auto i = idx % state.width, j = idx / state.width;
      auto u        = (i + rand1f(state.rngs[idx])) / state.width,
           v        = (j + rand1f(state.rngs[idx])) / state.height;
      auto ray      = eval_camera(camera, {u, v});
      auto radiance = shader(scene, bvh, ray, 0, state.rngs[idx], params);
      if (!isfinite(radiance)) radiance = {0, 0, 0};
      state.image[idx] += radiance;
      state.hits[idx] += 1;
    });
  }
}

// Trace context
raytrace_context make_raytrace_context(const raytrace_params& params) {
  return {{}, false, false};
}

// Async start
void raytrace_start(raytrace_context& context, raytrace_state& state,
    const scene_data& scene, const scene_bvh& bvh,
    const raytrace_params& params) {
  if (state.samples >= params.samples) return;
  context.stop   = false;
  context.done   = false;
  context.worker = std::async(std::launch::async, [&]() {
    if (context.stop) return;
    auto& camera = scene.cameras[params.camera];
    auto  shader = get_shader(params);
    parallel_for(state.width, state.height, [&](int i, int j) {
      if (context.stop) return;
      auto idx      = j * state.width + i;
      auto u        = (i + rand1f(state.rngs[idx])) / state.width,
           v        = (j + rand1f(state.rngs[idx])) / state.height;
      auto ray      = eval_camera(camera, {u, v});
      auto radiance = shader(scene, bvh, ray, 0, state.rngs[idx], params);
      if (!isfinite(radiance)) radiance = {0, 0, 0};
      state.image[idx] += radiance;
      state.hits[idx] += 1;
    });
    state.samples += 1;
    if (context.stop) return;
    context.done = true;
  });
}

// Async cancel
void raytrace_cancel(raytrace_context& context) {
  context.stop = true;
  if (context.worker.valid()) context.worker.get();
}

// Async done
bool raytrace_done(const raytrace_context& context) { return context.done; }

void raytrace_preview(color_image& image, raytrace_context& context,
    raytrace_state& state, const scene_data& scene, const scene_bvh& bvh,
    const raytrace_params& params) {
  // preview
  auto pparams = params;
  pparams.resolution /= params.pratio;
  pparams.samples = 1;
  auto pstate     = make_raytrace_state(scene, pparams);
  raytrace_samples(pstate, scene, bvh, pparams);
  auto preview = get_image(pstate);
  for (auto idx = 0; idx < state.width * state.height; idx++) {
    auto i = idx % image.width, j = idx / image.width;
    auto pi           = clamp(i / params.pratio, 0, preview.width - 1),
         pj           = clamp(j / params.pratio, 0, preview.height - 1);
    image.pixels[idx] = preview.pixels[pj * preview.width + pi];
  }
}

// Check image type
static void check_image(
    const color_image& image, int width, int height, bool linear) {
  if (image.width != width || image.height != height)
    throw std::invalid_argument{"image should have the same size"};
  if (image.linear != linear)
    throw std::invalid_argument{
        linear ? "expected linear image" : "expected srgb image"};
}

// Get resulting render
color_image get_image(const raytrace_state& state) {
  auto image = make_image(state.width, state.height, true);
  get_image(image, state);
  return image;
}
void get_image(color_image& image, const raytrace_state& state) {
  check_image(image, state.width, state.height, true);
  auto scale = 1.0f / (float)state.samples;
  for (auto idx = 0; idx < state.width * state.height; idx++) {
    image.pixels[idx] = state.image[idx] * scale;
  }
}

}  // namespace yocto

#endif
