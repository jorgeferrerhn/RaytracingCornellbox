//
// Yocto/Grade: Tiny library of color manipulation examples.
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

#ifndef _YOCTO_IMAGING_
#define _YOCTO_IMAGING_

#include <yocto/yocto_image.h>
#include <yocto/yocto_math.h>

// -----------------------------------------------------------------------------
// COLOR GRADING FUNCTIONS
// -----------------------------------------------------------------------------
namespace yocto {

// Grade parameters
struct grade_params {
  float exposure   = 0.0f;
  bool  filmic     = false;
  bool  srgb       = true;
  vec3f tint       = vec3f{1, 1, 1};
  float saturation = 0.5f;
  float contrast   = 0.5f;
  float vignette   = 0.0f;
  float grain      = 0.0f;
  int   mosaic     = 0;
  int   grid       = 0;
  int   blur       = 0;
  bool  box        = false;
  float sharpness  = 0;
  float scatter    = 0;
  float anaglyph   = 0;
};

// Grading functions
color_image grade_image(const color_image& image, const grade_params& params);

};  // namespace yocto

#endif
