//
// # Yocto/Extension: Tiny Yocto/GL extension
//
//

//
// LICENSE:
//
// Copyright (c) 2020 -- 2020 Fabio Pellacini
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
//

#ifndef _YOCTO_EXTENSION_H_
#define _YOCTO_EXTENSION_H_

// -----------------------------------------------------------------------------
// INCLUDES
// -----------------------------------------------------------------------------

#include <yocto/yocto_image.h>
#include <yocto/yocto_math.h>

#include <atomic>
#include <future>
#include <memory>

namespace yocto::pathtrace {
struct material;
}

// -----------------------------------------------------------------------------
// ALIASES
// -----------------------------------------------------------------------------
namespace yocto::extension {

// Namespace aliases
namespace ext = yocto::extension;
namespace img = yocto::image;

// Math defitions
using math::bbox3f;
using math::byte;
using math::clamp;
using math::frame3f;
using math::identity3x4f;
using math::max;
using math::ray3f;
using math::rng_state;
using math::vec2f;
using math::vec2i;
using math::vec3b;
using math::vec3f;
using math::vec3i;
using math::vec4f;
using math::vec4i;
using math::zero2f;
using math::zero3f;

}  // namespace yocto::extension

// -----------------------------------------------------------------------------
// HIGH LEVEL API
// -----------------------------------------------------------------------------
namespace yocto::extension {

static const int p_max = 3;

struct hair_material {
  vec3f sigma_a     = zero3f;
  float beta_m      = 0.3;
  float beta_n      = 0.3;
  float alpha       = 2;
  float eta         = 1.55;
  vec3f color       = zero3f;
  float eumelanin   = 0;
  float pheomelanin = 0;
};

struct hair_brdf {
  vec3f sigma_a = zero3f;
  float alpha   = 2;
  float eta     = 1.55;
  float h       = 0;

  // computed properties
  std::array<float, p_max + 1> v;
  float                        s = 0;
  vec3f                        sin_2k_alpha;
  vec3f                        cos_2k_alpha;
  float                        gamma_o = 0;

  // Allow to convert outgoing and incoming directions to BRDF coordinate
  // system
  frame3f world_to_brdf = identity3x4f;
};

hair_brdf eval_hair_brdf(const hair_material& material, float v,
    const vec3f& normal, const vec3f& tangent);

vec3f eval_hair_scattering(
    const hair_brdf& brdf, const vec3f& outgoing, const vec3f& incoming);

vec3f sample_hair_scattering(
    const hair_brdf& brdf, const vec3f& outgoing, const vec2f& rn);

float sample_hair_scattering_pdf(
    const hair_brdf& brdf, const vec3f& outgoing, const vec3f& incoming);

void white_furnace_test();
void sampling_weights_test();
void white_furnace_sampled_test();
void sampling_consistency_test();

}  // namespace yocto::extension

#endif
