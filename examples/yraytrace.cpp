//
// LICENSE:
//
// Copyright (c) 2016 -- 2022 Fabio Pellacini
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

#include <yocto/yocto_cli.h>
#include <yocto/yocto_gui.h>
#include <yocto/yocto_math.h>
#include <yocto/yocto_scene.h>
#include <yocto/yocto_sceneio.h>
#include <yocto/yocto_shape.h>
#include <yocto_examples/yocto_raytrace.h>

using namespace yocto;
using namespace std::string_literals;

// main function
void run(const vector<string>& args) {
  // parameters
  auto scenename   = "scene.json"s;
  auto outname     = "out.png"s;
  auto interactive = false;
  auto params      = raytrace_params{};

  // parse command line
  auto cli = make_cli("ytrace", "render with raytracing");
  add_option(cli, "scene", scenename, "scene filename");
  add_option(cli, "output", outname, "output filename");
  add_option(cli, "interactive", interactive, "run interactively");
  add_option(cli, "resolution", params.resolution, "image resolution");
  add_option(
      cli, "shader", params.shader, "shader type", raytrace_shader_labels);
  add_option(cli, "samples", params.samples, "number of samples");
  add_option(cli, "bounces", params.bounces, "number of bounces");
  add_option(cli, "noparallel", params.noparallel, "disable threading");
  parse_cli(cli, args);

  // start rendering
  print_info("rendering {}", scenename);
  auto timer = simple_timer{};

  // scene loading
  timer      = simple_timer{};
  auto scene = load_scene(scenename);
  print_info("load scene: {}", elapsed_formatted(timer));

  // camera
  params.camera = 0;

  // tesselation
  if (!scene.subdivs.empty()) {
    tesselate_subdivs(scene);
  }

  // build bvh
  timer    = simple_timer{};
  auto bvh = make_raytrace_bvh(scene, params);
  print_info("build bvh: {}", elapsed_formatted(timer));

  // state
  auto state = make_raytrace_state(scene, params);

  if (!interactive) {
    // render
    timer = simple_timer{};
    for (auto sample : range(0, params.samples)) {
      auto sample_timer = simple_timer{};
      raytrace_samples(state, scene, bvh, params);
      print_info("render sample {}/{}: {}", state.samples, params.samples,
          elapsed_formatted(sample_timer));
    }
    print_info("render image: {}", elapsed_formatted(timer));

    // save image
    timer      = simple_timer{};
    auto image = get_image(state);
    save_image(outname, image);
    print_info("save image: {}", elapsed_formatted(timer));
  } else {
#ifdef YOCTO_OPENGL
    // rendering context
    auto context = make_raytrace_context(params);

    // init image
    auto image = make_image(state.width, state.height, true);

    // opengl image
    auto glimage     = glimage_state{};
    auto glparams    = glimage_params{};
    glparams.tonemap = true;

    // camera names
    auto camera_names = scene.camera_names;
    if (camera_names.empty()) {
      for (auto idx : range(scene.cameras.size())) {
        camera_names.push_back("camera" + std::to_string(idx + 1));
      }
    }

    // start rendering batch
    auto render_next = [&]() {
      raytrace_cancel(context);
      raytrace_start(context, state, scene, bvh, params);
    };

    // restart renderer
    auto render_restart = [&]() {
      // make sure we can start
      raytrace_cancel(context);
      state = make_raytrace_state(scene, params);
      if (image.width != state.width || image.height != state.height)
        image = make_image(state.width, state.height, true);

      // render preview
      raytrace_preview(image, context, state, scene, bvh, params);

      // update image
      set_image(glimage, image);

      // start
      raytrace_start(context, state, scene, bvh, params);
    };

    // render cancel
    auto render_cancel = [&]() { raytrace_cancel(context); };

    // render update
    auto render_update = [&]() {
      if (context.done) {
        get_image(image, state);
        set_image(glimage, image);
        raytrace_start(context, state, scene, bvh, params);
      }
    };

    // prepare selection
    auto selection = scene_selection{};

    // callbacks
    auto callbacks = gui_callbacks{};
    callbacks.init = [&](const gui_input& input) {
      init_image(glimage);
      render_restart();
    };
    callbacks.clear = [&](const gui_input& input) { clear_image(glimage); };
    callbacks.draw  = [&](const gui_input& input) {
      render_update();
      update_image_params(input, image, glparams);
      draw_image(glimage, glparams);
    };
    callbacks.widgets = [&](const gui_input& input) {
      auto tparams = params;
      // if (draw_raytrace_widgets(input, state.samples, tparams, camera_names))
      // {
      //   render_cancel();
      //   params = tparams;
      //   render_restart();
      // }
      draw_tonemap_widgets(input, glparams.exposure, glparams.filmic);
      draw_image_widgets(input, image, glparams);
    };
    callbacks.uiupdate = [&](const gui_input& input) {
      auto camera = scene.cameras[params.camera];
      if (uiupdate_camera_params(input, camera)) {
        render_cancel();
        scene.cameras[params.camera] = camera;
        render_restart();
      }
    };

    // run ui
    show_gui_window({1280 + 320, 720}, "ytrace - " + scenename, callbacks);

    // done
    render_cancel();
#else
    throw io_error{"Interactive requires OpenGL"};
#endif
  }
}

// Run
int main(int argc, const char* argv[]) {
  try {
    run({argv, argv + argc});
    return 0;
  } catch (const std::exception& error) {
    print_error(error.what());
    return 1;
  }
}
