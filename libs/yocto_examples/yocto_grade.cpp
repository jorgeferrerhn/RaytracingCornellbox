//
// Implementation for Yocto/Grade.
//

//
// LICENSE:
//
// Copyright (c) 2020 -- 2022 Fabio Pellacini
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

#include "yocto_grade.h"

#include <yocto/yocto_color.h>
#include <yocto/yocto_sampling.h>

// -----------------------------------------------------------------------------
// COLOR GRADING FUNCTIONS
// -----------------------------------------------------------------------------
namespace yocto {

color_image box_blur(const color_image& image, int radius) {
  // make a copy
  auto filtered = make_image(image.width, image.height, image.linear);

  // choose implementation
  if (radius <= 2) {  // small kernels
    // loop over result pixels
    for (auto j : range(image.height)) {
      for (auto i : range(image.width)) {
        // init to zero
        filtered[{i, j}] = {0, 0, 0, 0};
        auto count       = 0.0f;
        // loop over kernel pixel
        for (auto jj = -radius; jj <= radius; jj++) {
          for (auto ii = -radius; ii <= radius; ii++) {
            if (i + ii < 0 || i + ii >= image.width) continue;
            if (j + jj < 0 || j + jj >= image.height) continue;
            filtered[{i, j}] += image[{i + ii, j + jj}];
            count += 1;
          }
        }
        // normalize
        filtered[{i, j}] /= count;
      }
    }
  } else {  // large kernel
    auto buffer = make_image(image.width, image.height, image.linear);
    // filter w.r.t. x
    for (auto j : range(image.height)) {
      for (auto i : range(image.width)) {
        buffer[{i, j}] = {0, 0, 0, 0};
        auto count     = 0.0f;
        for (auto ii = -radius; ii <= radius; ii++) {
          if (i + ii < 0 || i + ii >= image.width) continue;
          buffer[{i, j}] += image[{i + ii, j}];
          count += 1;
        }
        buffer[{i, j}] /= count;
      }
    }
    // filter w.r.t. y
    for (auto j : range(image.height)) {
      for (auto i : range(image.width)) {
        filtered[{i, j}] = {0, 0, 0, 0};
        auto count       = 0.0f;
        for (auto jj = -radius; jj <= radius; jj++) {
          if (j + jj < 0 || j + jj >= image.height) continue;
          filtered[{i, j}] += buffer[{i, j + jj}];
          count += 1;
        }
        filtered[{i, j}] /= count;
      }
    }
  }

  // done
  return filtered;
}

color_image gaussian_blur(const color_image& image, int radius) {
  // compute kernel
  auto kernel = vector<float>(radius * 2 + 1);   // gaussian kernel
  auto sigma  = max((float)radius / 2, 1.0f);    // gaussian sigma
  for (auto x = -radius; x <= radius; x += 1) {  // compute kernel
    kernel[x + radius] = exp(-x * x / (2 * sigma * sigma));
  }

  // make a copy
  auto filtered = make_image(image.width, image.height, image.linear);

  // choose implementation
  if (radius <= 2) {  // small kernels
    // loop over result pixels
    for (auto j : range(image.height)) {
      for (auto i : range(image.width)) {
        // init to zero
        filtered[{i, j}] = {0, 0, 0, 0};
        auto weight      = 0.0f;
        // loop over kernel pixel
        for (auto jj = -radius; jj <= radius; jj++) {
          for (auto ii = -radius; ii <= radius; ii++) {
            if (i + ii < 0 || i + ii >= image.width) continue;
            if (j + jj < 0 || j + jj >= image.height) continue;
            filtered[{i, j}] += image[{i + ii, j + jj}] * kernel[ii + radius] *
                                kernel[jj + radius];
            weight += kernel[ii + radius] * kernel[jj + radius];
          }
        }
        // normalize
        filtered[{i, j}] /= weight;
      }
    }
  } else {  // large kernel
    auto buffer = make_image(image.width, image.height, image.linear);
    // filter w.r.t. x
    for (auto j : range(image.height)) {
      for (auto i : range(image.width)) {
        buffer[{i, j}] = {0, 0, 0, 0};
        auto weight    = 0.0f;
        for (auto ii = -radius; ii <= radius; ii++) {
          if (i + ii < 0 || i + ii >= image.width) continue;
          buffer[{i, j}] += image[{i + ii, j}] * kernel[ii + radius];
          weight += kernel[ii + radius];
        }
        buffer[{i, j}] /= weight;
      }
    }
    // filter w.r.t. y
    for (auto j : range(image.height)) {
      for (auto i : range(image.width)) {
        filtered[{i, j}] = {0, 0, 0, 0};
        auto weight      = 0.0f;
        for (auto jj = -radius; jj <= radius; jj++) {
          if (j + jj < 0 || j + jj >= image.height) continue;
          filtered[{i, j}] += buffer[{i, j + jj}] * kernel[jj + radius];
          weight += kernel[jj + radius];
        }
        filtered[{i, j}] /= weight;
      }
    }
  }

  // done
  return filtered;
}

// helper functions
inline float clamp01(float c) { return max(0.0f, min(1.0f, c)); }
inline vec3f clamp01(vec3f c) {
  return {clamp01(c.x), clamp01(c.y), clamp01(c.z)};
}
inline vec4f clamp01(vec4f c) {
  return {clamp01(c.x), clamp01(c.y), clamp01(c.z), clamp01(c.w)};
}

// contrast functions
inline float bias_(float a, float b) { return a / ((1 / b - 2) * (1 - a) + 1); }
inline float gain_(float a, float b) {
  return (a < 0.5f) ? bias_(a * 2, b) / 2 : bias_(a * 2 - 1, 1 - b) / 2 + 0.5f;
}
inline vec3f gain_(vec3f a, float b) {
  return {gain_(a.x, b), gain_(a.y, b), gain_(a.z, b)};
}

color_image grade_image(const color_image& image, const grade_params& params) {
  // Init result image
  auto graded = image;

  // Loop over pixels
  for (auto& pixel : graded.pixels) {
    // separate color and alpha
    auto c = xyz(pixel);
    auto a = pixel.w;
    // linear -----------------------------------------------------
    c *= pow(2, params.exposure);  // exposure
    if (params.filmic) {           // filmic correction
      c *= 0.6;
      c = (c * c * 2.51f + c * 0.03f) / (c * c * 2.43f + c * 0.59f + 0.14f);
    }
    if (params.srgb) c = pow(c, 1 / 2.2f);  // linear to srgb
    c = clamp01(c);                         // clamp to [0,1]
    // gamma-corrected [0,1] --------------------------------------
    c *= params.tint;                                       // color tinting
    c = mean(c) + (c - mean(c)) * (params.saturation * 2);  // saturation
    c = clamp01(c);                                         // clamp to [0,1]
    c = gain_(c, 1 - params.contrast);                      // contrast
    c = clamp01(c);                                         // clamp to [0,1]
    // save back
    pixel = {c.x, c.y, c.z, a};
  }

  // vignette
  if (params.vignette > 0) {
    auto smoothstep = [](float a, float b, float u) {
      auto t = clamp((u - a) / (b - a), 0.0f, 1.0f);
      return t * t * (3 - 2 * t);
    };
    for (auto j : range(image.height)) {
      for (auto i : range(image.width)) {
        // separate color and alpha
        auto c = xyz(graded[{i, j}]);
        auto a = graded[{i, j}].w;
        // apply vignette
        auto vr     = 1 - params.vignette;
        auto size   = vec2f{(float)image.width, (float)image.height};
        auto radius = length(vec2f{(float)i, (float)j} - size / 2) /
                      length(size / 2);
        c = c * (1 - smoothstep(vr, 2 * vr, radius));
        c = clamp01(c);  // clamp to [0,1]
        // save back
        graded[{i, j}] = {c.x, c.y, c.z, a};
      }
    }
  }

  // mosaic
  if (params.mosaic > 0) {
    for (auto j : range(image.height)) {
      for (auto i : range(image.width)) {
        graded[{i, j}] = graded[{i - i % params.mosaic, j - j % params.mosaic}];
      }
    }
  }

  // grid
  if (params.grid > 0) {
    for (auto j : range(image.height)) {
      for (auto i : range(image.width)) {
        graded[{i, j}] = (0 == i % params.grid || 0 == j % params.grid)
                             ? 0.5 * graded[{i, j}]
                             : graded[{i, j}];
      }
    }
  }

  // grain
  if (params.grain > 0) {
    auto rng = make_rng(827981729871llu);  // seed generator for consistency
    for (auto& pixel : graded.pixels) {
      // separate color and alpha
      auto c = xyz(pixel);
      auto a = pixel.w;
      // apply grain
      c = c + (rand1f(rng) - 0.5f) * params.grain;
      // save back
      pixel = {c.x, c.y, c.z, a};
    }
  }

  // blur
  if (params.blur && params.sharpness == 0) {
    graded = params.box ? box_blur(graded, params.blur)
                        : gaussian_blur(graded, params.blur);
  }

  // sharpening
  if (params.blur && params.sharpness > 0) {
    auto blurred = params.box ? box_blur(graded, params.blur)
                              : gaussian_blur(graded, params.blur);
    for (auto idx : range(image.width * image.height)) {
      graded.pixels[idx] = blurred.pixels[idx] +
                           (graded.pixels[idx] - blurred.pixels[idx]) *
                               params.sharpness;
      graded.pixels[idx]   = clamp01(graded.pixels[idx]);  // clamp to [0,1]
      graded.pixels[idx].w = 1;
    }
  }

  // scattering
  if (params.scatter > 0) {
    auto rng      = make_rng(81721);
    auto original = graded;
    for (auto j : range(image.height)) {
      for (auto i : range(image.width)) {
        auto uv        = (rand2f(rng) * 2 - 1) * params.scatter;
        graded[{i, j}] = original[{clamp(int(i + uv.x + 0.5f), 0, image.width-1),
            clamp(int(j + uv.y + 0.5f), 0, image.height - 1)}];
      }
    }
  }

  // scattering
  if (params.anaglyph > 0) {
    auto original = graded;
    for (auto j : range(image.height)) {
      for (auto i : range(image.width)) {
        graded[{i, j}].x = original[{i, j}].x;
        graded[{i, j}].y = original[{(i + int(params.anaglyph + 0.5f)) % image.width, j}].y;
        graded[{i, j}].z = original[{i, (j + int(params.anaglyph + 0.5f)) % image.height}].z;
      }
    }
  }

  // Done
  return graded;
}

}  // namespace yocto