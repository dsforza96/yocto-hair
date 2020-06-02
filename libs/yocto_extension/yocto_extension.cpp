//
// Implementation for Yocto/Extension.
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

#include "yocto_extension.h"

#include <atomic>
#include <deque>
#include <future>
#include <memory>
#include <mutex>
#include <stdexcept>

using namespace std::string_literals;

// -----------------------------------------------------------------------------
// MATH FUNCTIONS
// -----------------------------------------------------------------------------
namespace yocto::extension {

// import math symbols for use
using math::abs;
using math::acos;
using math::atan2;
using math::clamp;
using math::cos;
using math::exp;
using math::flt_eps;
using math::flt_max;
using math::fmod;
using math::fresnel_conductor;
using math::fresnel_dielectric;
using math::identity3x3f;
using math::invalidb3f;
using math::log;
using math::luminance;
using math::make_rng;
using math::max;
using math::min;
using math::pif;
using math::pow;
using math::rand1f;
using math::rand2f;
using math::sample_discrete_cdf;
using math::sample_discrete_cdf_pdf;
using math::sample_sphere;
using math::sample_sphere_pdf;
using math::sample_uniform;
using math::sample_uniform_pdf;
using math::sin;
using math::sqrt;
using math::zero2f;
using math::zero2i;
using math::zero3f;
using math::zero3i;
using math::zero4f;
using math::zero4i;

}  // namespace yocto::extension

// -----------------------------------------------------------------------------
// IMPLEMENTATION FOR EXTENSION
// -----------------------------------------------------------------------------
namespace yocto::extension {

static const float sqrt_pi_over_8f = 0.626657069f;

inline float sqr(float v) { return v * v; }
inline vec3f sqr(vec3f v) { return v * v; }

template <int n>
static float pow(float v) {
  auto n2 = pow<n / 2>(v);
  return n2 * n2 * pow<n & 1>(v);
}

template <>
inline float pow<1>(float v) {
  return v;
}

template <>
inline float pow<0>(float v) {
  return 1;
}

inline float safe_asin(float x) { return asin(clamp(x, -1.0f, 1.0f)); }

inline float safe_sqrt(float x) { return sqrt(max(0.0f, x)); }

static vec3f sigma_a_from_concentration(float ce, float cp) {
  auto eumelanin_sigma_a   = vec3f{0.419f, 0.697f, 1.37f};
  auto pheomelanin_sigma_a = vec3f{0.187f, 0.4f, 1.05f};
  return ce * eumelanin_sigma_a + cp * pheomelanin_sigma_a;
}

static vec3f sigma_a_from_reflectance(const vec3f& c, float beta_n) {
  return sqr(log(c) / (5.969f - 0.215f * beta_n + 2.532f * sqr(beta_n) -
                          10.73f * pow<3>(beta_n) + 5.574f * pow<4>(beta_n) +
                          0.245f * pow<5>(beta_n)));
}

hair_brdf eval_hair_brdf(const hair_material& material, float v,
    const vec3f& normal, const vec3f& tangent) {
  auto brdf = hair_brdf{};

  if (material.sigma_a) {
    brdf.sigma_a = material.sigma_a;
  } else if (material.color) {
    brdf.sigma_a = sigma_a_from_reflectance(material.color, material.beta_n);
  } else if (material.eumelanin || material.pheomelanin) {
    brdf.sigma_a = sigma_a_from_concentration(
        material.eumelanin, material.pheomelanin);
  }

  auto beta_m = material.beta_m;
  auto beta_n = material.beta_n;
  brdf.alpha  = material.alpha;
  brdf.eta    = material.eta;

#ifdef YOCTO_EMBREE
  brdf.h = v;
#else
  brdf.h = -1 + 2 * v;
#endif

  brdf.gamma_o = safe_asin(brdf.h);

  // Compute longitudinal variance from $\beta_m$
  brdf.v[0] = sqr(
      0.726f * beta_m + 0.812f * sqr(beta_m) + 3.7f * pow<20>(beta_m));
  brdf.v[1] = 0.25f * brdf.v[0];
  brdf.v[2] = 4 * brdf.v[0];
  for (auto p = 3; p <= p_max; ++p) brdf.v[p] = brdf.v[2];

  // Compute azimuthal logistic scale factor from $\beta_n$
  brdf.s = sqrt_pi_over_8f *
           (0.265f * beta_n + 1.194f * sqr(beta_n) + 5.372f * pow<22>(beta_n));

  // Compute $\alpha$ terms for hair scales
  brdf.sin_2k_alpha[0] = sin(pif / 180 * brdf.alpha);
  brdf.cos_2k_alpha[0] = safe_sqrt(1 - sqr(brdf.sin_2k_alpha[0]));
  for (auto i = 1; i < 3; ++i) {
    brdf.sin_2k_alpha[i] = 2 * brdf.cos_2k_alpha[i - 1] *
                           brdf.sin_2k_alpha[i - 1];
    brdf.cos_2k_alpha[i] = sqr(brdf.cos_2k_alpha[i - 1]) -
                           sqr(brdf.sin_2k_alpha[i - 1]);
  }

  brdf.world_to_brdf = inverse(frame_fromzx(zero3f, normal, tangent));

  return brdf;
}

inline float i0(float x) {
  float   val   = 0;
  float   x2i   = 1;
  int64_t ifact = 1;
  int     i4    = 1;
  // I0(x) \approx Sum_i x^(2i) / (4^i (i!)^2)
  for (int i = 0; i < 10; ++i) {
    if (i > 1) ifact *= i;
    val += x2i / (i4 * ifact * ifact);
    x2i *= x * x;
    i4 *= 4;
  }
  return val;
}

inline float log_i0(float x) {
  if (x > 12)
    return x + 0.5f * (-log(2 * pif) + log(1 / x) + 1 / (8 * x));
  else
    return log(i0(x));
}

static float mp(float cos_theta_i, float cos_theta_o, float sin_theta_i,
    float sin_theta_o, float v) {
  auto a = cos_theta_i * cos_theta_o / v;
  auto b = sin_theta_i * sin_theta_o / v;
  return (v <= 0.1f) ? (exp(log_i0(a) - b - 1 / v + 0.6931f + log(1 / (2 * v))))
                     : (exp(-b) * i0(a)) / (sinh(1 / v) * 2 * v);
}

static std::array<vec3f, p_max + 1> ap(
    float cos_theta_o, float eta, float h, const vec3f& T) {
  std::array<vec3f, p_max + 1> ap;
  // Compute $p=0$ attenuation at initial cylinder intersection
  auto cos_gamma_o = safe_sqrt(1 - h * h);
  auto cos_theta   = cos_theta_o * cos_gamma_o;

  // fresnel_dielectric(eta, cos_theta): we hack function's parameters bulding
  // two vector s.t. their dot product gives exactly cos_theta
  auto f = fresnel_dielectric(eta, {0, 0, 1}, {0, 0, cos_theta});
  ap[0]  = vec3f(f);

  // Compute $p=1$ attenuation term
  ap[1] = sqr(1 - f) * T;

  // Compute attenuation terms up to $p=_pMax_$
  for (auto p = 2; p < p_max; ++p) ap[p] = ap[p - 1] * T * f;

  // Compute attenuation term accounting for remaining orders of scattering
  ap[p_max] = ap[p_max - 1] * f * T / (vec3f(1.f) - T * f);
  return ap;
}

inline float phi(int p, float gamma_o, float gamma_t) {
  return 2 * p * gamma_t - 2 * gamma_o + p * pif;
}

inline float logistic(float x, float s) {
  x = abs(x);
  return exp(-x / s) / (s * sqr(1 + exp(-x / s)));
}

inline float logistic_cdf(float x, float s) { return 1 / (1 + exp(-x / s)); }

inline float trimmed_logistic(float x, float s, float a, float b) {
  return logistic(x, s) / (logistic_cdf(b, s) - logistic_cdf(a, s));
}

inline float np(float phi, int p, float s, float gamma_o, float gamma_t) {
  auto dphi = phi - ext::phi(p, gamma_o, gamma_t);
  // Remap _dphi_ to $[-\pi,\pi]$
  while (dphi > pif) dphi -= 2 * pif;
  while (dphi < -pif) dphi += 2 * pif;
  return trimmed_logistic(dphi, s, -pif, pif);
}

vec3f eval_hair_scattering(
    const hair_brdf& brdf, const vec3f& outgoing_, const vec3f& incoming_) {
  auto sigma_a       = brdf.sigma_a;
  auto eta           = brdf.eta;
  auto h             = brdf.h;
  auto gamma_o       = brdf.gamma_o;
  auto v             = brdf.v;
  auto s             = brdf.s;
  auto sin_2k_alpha  = brdf.sin_2k_alpha;
  auto cos_2k_alpha  = brdf.cos_2k_alpha;
  auto world_to_brdf = brdf.world_to_brdf;

  auto outgoing = transform_direction(world_to_brdf, outgoing_);
  auto incoming = transform_direction(world_to_brdf, incoming_);

  // Compute hair coordinate system terms related to _wo_
  auto sin_theta_o = outgoing.x;
  auto cos_theta_o = safe_sqrt(1 - sqr(sin_theta_o));
  auto phi_o       = atan2(outgoing.z, outgoing.y);

  // Compute hair coordinate system terms related to _wi_
  auto sin_theta_i = incoming.x;
  auto cos_theta_i = safe_sqrt(1 - sqr(sin_theta_i));
  auto phi_i       = atan2(incoming.z, incoming.y);

  // Compute $\cos \thetat$ for refracted ray
  auto sin_theta_t = sin_theta_o / eta;
  auto cos_theta_t = safe_sqrt(1 - sqr(sin_theta_t));

  // Compute $\gammat$ for refracted ray
  auto etap        = sqrt(eta * eta - sqr(sin_theta_o)) / cos_theta_o;
  auto sin_gamma_t = h / etap;
  auto cos_gamma_t = safe_sqrt(1 - sqr(sin_gamma_t));
  auto gamma_t     = safe_asin(sin_gamma_t);

  // Compute the transmittance _T_ of a single path through the cylinder
  auto T = exp(-sigma_a * (2 * cos_gamma_t / cos_theta_t));

  // Evaluate hair BSDF
  auto phi  = phi_i - phi_o;
  auto ap   = ext::ap(cos_theta_o, eta, h, T);
  auto fsum = zero3f;
  for (auto p = 0; p < p_max; ++p) {
    // Compute $\sin \thetao$ and $\cos \thetao$ terms accounting for scales
    auto sin_theta_op = 0.0f;
    auto cos_theta_op = 0.0f;
    if (p == 0) {
      sin_theta_op = sin_theta_o * cos_2k_alpha[1] -
                     cos_theta_o * sin_2k_alpha[1];
      cos_theta_op = cos_theta_o * cos_2k_alpha[1] +
                     sin_theta_o * sin_2k_alpha[1];
    }

    // Handle remainder of $p$ values for hair scale tilt
    else if (p == 1) {
      sin_theta_op = sin_theta_o * cos_2k_alpha[0] +
                     cos_theta_o * sin_2k_alpha[0];
      cos_theta_op = cos_theta_o * cos_2k_alpha[0] -
                     sin_theta_o * sin_2k_alpha[0];
    } else if (p == 2) {
      sin_theta_op = sin_theta_o * cos_2k_alpha[2] +
                     cos_theta_o * sin_2k_alpha[2];
      cos_theta_op = cos_theta_o * cos_2k_alpha[2] -
                     sin_theta_o * sin_2k_alpha[2];
    } else {
      sin_theta_op = sin_theta_o;
      cos_theta_op = cos_theta_o;
    }

    // Handle out-of-range $\cos \thetao$ from scale adjustment
    cos_theta_op = abs(cos_theta_op);
    fsum += mp(cos_theta_i, cos_theta_op, sin_theta_i, sin_theta_op, v[p]) *
            ap[p] * np(phi, p, s, gamma_o, gamma_t);
  }

  // Compute contribution of remaining terms after _pMax_
  fsum += mp(cos_theta_i, cos_theta_o, sin_theta_i, sin_theta_o, v[p_max]) *
          ap[p_max] / (2 * pif);

  // if (abs(incoming.z) > 0)
  //   fsum /= abs(incoming.z);
  // return fsum * abs(incoming.z);

  return fsum;
}

// https://fgiesen.wordpress.com/2009/12/13/decoding-morton-codes/
static uint32_t compact1by1(uint32_t x) {
  // x = -f-e -d-c -b-a -9-8 -7-6 -5-4 -3-2 -1-0
  x &= 0x55555555;
  // x = --fe --dc --ba --98 --76 --54 --32 --10
  x = (x ^ (x >> 1)) & 0x33333333;
  // x = ---- fedc ---- ba98 ---- 7654 ---- 3210
  x = (x ^ (x >> 2)) & 0x0f0f0f0f;
  // x = ---- ---- fedc ba98 ---- ---- 7654 3210
  x = (x ^ (x >> 4)) & 0x00ff00ff;
  // x = ---- ---- ---- ---- fedc ba98 7654 3210
  x = (x ^ (x >> 8)) & 0x0000ffff;
  return x;
}

static vec2f demux_float(float f) {
  uint64_t v       = f * (1ull << 32);
  uint32_t bits[2] = {compact1by1(v), compact1by1(v >> 1)};
  return {bits[0] / float(1 << 16), bits[1] / float(1 << 16)};
}

static float sample_trimmed_logistic(float u, float s, float a, float b) {
  auto k = logistic_cdf(b, s) - logistic_cdf(a, s);
  auto x = -s * log(1 / (u * k + logistic_cdf(a, s)) - 1);
  return clamp(x, a, b);
}

static std::array<float, p_max + 1> compute_ap_pdf(
    const hair_brdf& brdf, float cos_theta_o) {
  auto sigma_a = brdf.sigma_a;
  auto eta     = brdf.eta;
  auto h       = brdf.h;

  // Compute array of $A_p$ values for _cosThetaO_
  auto sin_theta_o = safe_sqrt(1 - cos_theta_o * cos_theta_o);

  // Compute $\cos \thetat$ for refracted ray
  auto sin_theta_t = sin_theta_o / eta;
  auto cos_theta_t = safe_sqrt(1 - sqr(sin_theta_t));

  // Compute $\gammat$ for refracted ray
  auto etap        = sqrt(eta * eta - sqr(sin_theta_o)) / cos_theta_o;
  auto sin_gamma_t = h / etap;
  auto cos_gamma_t = safe_sqrt(1 - sqr(sin_gamma_t));

  // Compute the transmittance _T_ of a single path through the cylinder
  auto T  = exp(-sigma_a * (2 * cos_gamma_t / cos_theta_t));
  auto ap = ext::ap(cos_theta_o, eta, h, T);

  // Compute $A_p$ PDF from individual $A_p$ terms
  auto ap_pdf = std::array<float, p_max + 1>{};
  auto sum_y  = 0.0f;
  for (auto i = 0; i <= p_max; ++i) {
    sum_y += luminance(ap[i]);
  }
  for (auto i = 0; i <= p_max; ++i) {
    ap_pdf[i] = luminance(ap[i]) / sum_y;
  }
  return ap_pdf;
}

vec3f sample_hair_scattering(
    const hair_brdf& brdf, const vec3f& outgoing_, const vec2f& rn) {
  auto eta           = brdf.eta;
  auto h             = brdf.h;
  auto gamma_o       = brdf.gamma_o;
  auto v             = brdf.v;
  auto s             = brdf.s;
  auto sin_2k_alpha  = brdf.sin_2k_alpha;
  auto cos_2k_alpha  = brdf.cos_2k_alpha;
  auto world_to_brdf = brdf.world_to_brdf;

  auto outgoing = transform_direction(world_to_brdf, outgoing_);

  // Compute hair coordinate system terms related to _wo_
  auto sin_theta_o = outgoing.x;
  auto cos_theta_o = safe_sqrt(1 - sqr(sin_theta_o));
  auto phi_o       = atan2(outgoing.z, outgoing.y);

  // Derive four random samples from _u2_
  auto u = std::array{demux_float(rn.x), demux_float(rn.y)};

  // Determine which term $p$ to sample for hair scattering
  auto ap_pdf = compute_ap_pdf(brdf, cos_theta_o);
  auto p      = 0;
  for (p = 0; p < p_max; ++p) {
    if (u[0][0] < ap_pdf[p]) break;
    u[0][0] -= ap_pdf[p];
  }

  // Rotate $\sin \thetao$ and $\cos \thetao$ to account for hair scale tilt
  auto sin_theta_op = 0.0f;
  auto cos_theta_op = 0.0f;
  if (p == 0) {
    sin_theta_op = sin_theta_o * cos_2k_alpha[1] -
                   cos_theta_o * sin_2k_alpha[1];
    cos_theta_op = cos_theta_o * cos_2k_alpha[1] +
                   sin_theta_o * sin_2k_alpha[1];
  } else if (p == 1) {
    sin_theta_op = sin_theta_o * cos_2k_alpha[0] +
                   cos_theta_o * sin_2k_alpha[0];
    cos_theta_op = cos_theta_o * cos_2k_alpha[0] -
                   sin_theta_o * sin_2k_alpha[0];
  } else if (p == 2) {
    sin_theta_op = sin_theta_o * cos_2k_alpha[2] +
                   cos_theta_o * sin_2k_alpha[2];
    cos_theta_op = cos_theta_o * cos_2k_alpha[2] -
                   sin_theta_o * sin_2k_alpha[2];
  } else {
    sin_theta_op = sin_theta_o;
    cos_theta_op = cos_theta_o;
  }

  // Sample $M_p$ to compute $\thetai$
  u[1][0]          = max(u[1][0], 1e-5f);
  auto cos_theta   = 1 + v[p] * log(u[1][0] + (1 - u[1][0]) * exp(-2 / v[p]));
  auto sin_theta   = safe_sqrt(1 - sqr(cos_theta));
  auto cos_phi     = cos(2 * pif * u[1][1]);
  auto sin_theta_i = -cos_theta * sin_theta_op +
                     sin_theta * cos_phi * cos_theta_op;
  auto cos_theta_i = safe_sqrt(1 - sqr(sin_theta_i));

  // Sample $N_p$ to compute $\Delta\phi$

  // Compute $\gammat$ for refracted ray
  auto etap        = sqrt(eta * eta - sqr(sin_theta_o)) / cos_theta_o;
  auto sin_gamma_t = h / etap;
  auto gamma_t     = safe_asin(sin_gamma_t);
  auto dphi        = 0.0f;
  if (p < p_max)
    dphi = phi(p, gamma_o, gamma_t) +
           sample_trimmed_logistic(u[0][1], s, -pif, pif);
  else
    dphi = 2 * pif * u[0][1];

  // Compute _wi_ from sampled hair scattering angles
  auto phi_i = phi_o + dphi;

  auto incoming = vec3f{
      sin_theta_i, cos_theta_i * cos(phi_i), cos_theta_i * sin(phi_i)};
  return transform_direction(inverse(world_to_brdf), incoming);
}

float sample_hair_scattering_pdf(
    const hair_brdf& brdf, const vec3f& outgoing_, const vec3f& incoming_) {
  auto eta           = brdf.eta;
  auto h             = brdf.h;
  auto gamma_o       = brdf.gamma_o;
  auto v             = brdf.v;
  auto s             = brdf.s;
  auto sin_2k_alpha  = brdf.sin_2k_alpha;
  auto cos_2k_alpha  = brdf.cos_2k_alpha;
  auto world_to_brdf = brdf.world_to_brdf;

  auto outgoing = transform_direction(world_to_brdf, outgoing_);
  auto incoming = transform_direction(world_to_brdf, incoming_);

  // Compute hair coordinate system terms related to _wo_
  auto sin_theta_o = outgoing.x;
  auto cos_theta_o = safe_sqrt(1 - sqr(sin_theta_o));
  auto phi_o       = atan2(outgoing.z, outgoing.y);

  // Compute hair coordinate system terms related to _wi_
  auto sin_theta_i = incoming.x;
  auto cos_theta_i = safe_sqrt(1 - sqr(sin_theta_i));
  auto phi_i       = atan2(incoming.z, incoming.y);

  // Compute $\gammat$ for refracted ray
  auto etap        = sqrt(eta * eta - sqr(sin_theta_o)) / cos_theta_o;
  auto sin_gamma_t = h / etap;
  auto gamma_t     = safe_asin(sin_gamma_t);

  // Compute PDF for $A_p$ terms
  auto ap_pdf = compute_ap_pdf(brdf, cos_theta_o);

  // Compute PDF sum for hair scattering events
  auto phi = phi_i - phi_o;
  auto pdf = 0.0f;
  for (auto p = 0; p < p_max; ++p) {
    // Compute $\sin \thetao$ and $\cos \thetao$ terms accounting for scales
    auto sin_theta_op = 0.0f;
    auto cos_theta_op = 0.0f;
    if (p == 0) {
      sin_theta_op = sin_theta_o * cos_2k_alpha[1] -
                     cos_theta_o * sin_2k_alpha[1];
      cos_theta_op = cos_theta_o * cos_2k_alpha[1] +
                     sin_theta_o * sin_2k_alpha[1];
    }

    // Handle remainder of $p$ values for hair scale tilt
    else if (p == 1) {
      sin_theta_op = sin_theta_o * cos_2k_alpha[0] +
                     cos_theta_o * sin_2k_alpha[0];
      cos_theta_op = cos_theta_o * cos_2k_alpha[0] -
                     sin_theta_o * sin_2k_alpha[0];
    } else if (p == 2) {
      sin_theta_op = sin_theta_o * cos_2k_alpha[2] +
                     cos_theta_o * sin_2k_alpha[2];
      cos_theta_op = cos_theta_o * cos_2k_alpha[2] -
                     sin_theta_o * sin_2k_alpha[2];
    } else {
      sin_theta_op = sin_theta_o;
      cos_theta_op = cos_theta_o;
    }

    // Handle out-of-range $\cos \thetao$ from scale adjustment
    cos_theta_op = abs(cos_theta_op);
    pdf += mp(cos_theta_i, cos_theta_op, sin_theta_i, sin_theta_op, v[p]) *
           ap_pdf[p] * np(phi, p, s, gamma_o, gamma_t);
  }
  pdf += mp(cos_theta_i, cos_theta_o, sin_theta_i, sin_theta_o, v[p_max]) *
         ap_pdf[p_max] * (1 / (2 * pif));
  return pdf;
}

// HAIR SCATTERING TESTS

void white_furnace_test() {
  auto rng = make_rng(199382389514);
  auto wo  = sample_sphere(rand2f(rng));
  for (auto beta_m = 0.1f; beta_m < 1.0f; beta_m += 0.2f) {
    for (auto beta_n = 0.1f; beta_n < 1.0f; beta_n += 0.2f) {
      // Estimate reflected uniform incident radiance from hair
      auto sum   = zero3f;
      auto count = 300000;
      for (auto i = 0; i < count; ++i) {
#ifdef YOCTO_EMBREE
        auto h = -1 + 2 * rand1f(rng);
#else
        auto h = rand1f(rng);
#endif
        // the original pbrt test fails with h = 0
        if (h == 0) h += flt_eps;

        auto mat   = hair_material{};
        mat.beta_m = beta_m;
        mat.beta_n = beta_n;
        mat.alpha  = 0;
        auto brdf  = eval_hair_brdf(mat, h, {0, 0, 1}, {1, 0, 0});
        auto wi    = sample_sphere(rand2f(rng));
        sum += eval_hair_scattering(brdf, wo, wi);
      }
      auto avg = luminance(sum) / (count * sample_sphere_pdf(wo));
      if (!(avg >= 0.95f && avg <= 1.05f))
        throw std::runtime_error("TEST FAILED!");
    }
  }
  printf("OK!\n");
  fflush(stdout);
}

void white_furnace_sampled_test() {
  auto rng = make_rng(199382389514);
  auto wo  = sample_sphere(rand2f(rng));
  for (auto beta_m = 0.1f; beta_m < 1.0f; beta_m += 0.2f) {
    for (auto beta_n = 0.1f; beta_n < 1.0f; beta_n += 0.2f) {
      auto sum   = zero3f;
      auto count = 300000;
      for (auto i = 0; i < count; ++i) {
#ifdef YOCTO_EMBREE
        auto h = -1 + 2 * rand1f(rng);
#else
        auto h = rand1f(rng);
#endif
        auto mat   = hair_material{};
        mat.beta_m = beta_m;
        mat.beta_n = beta_n;
        mat.alpha  = 0;
        auto brdf  = eval_hair_brdf(mat, h, {0, 0, 1}, {1, 0, 0});
        auto wi    = sample_hair_scattering(brdf, wo, rand2f(rng));
        auto pdf   = sample_hair_scattering_pdf(brdf, wo, wi);
        auto f     = eval_hair_scattering(brdf, wo, wi);
        if (pdf > 0) sum += f / pdf;
      }
      auto avg = luminance(sum) / (count);
      if (!(avg >= 0.99f && avg <= 1.01f))
        throw std::runtime_error("TEST FAILED!");
    }
  }
  printf("OK!\n");
  fflush(stdout);
}

void sampling_weights_test() {
  auto rng = make_rng(199382389514);
  for (auto beta_m = 0.1f; beta_m < 1.0f; beta_m += 0.2f) {
    for (auto beta_n = 0.4f; beta_n < 1.0f; beta_n += 0.2f) {
      auto count = 10000;
      for (auto i = 0; i < count; ++i) {
        // Check _HairBSDF::Sample\_f()_ sample weight
#ifdef YOCTO_EMBREE
        auto h = -1 + 2 * rand1f(rng);
#else
        auto h = rand1f(rng);
#endif
        auto mat   = hair_material{};
        mat.beta_m = beta_m;
        mat.beta_n = beta_n;
        mat.alpha  = 0;
        auto brdf  = eval_hair_brdf(mat, h, {0, 0, 1}, {1, 0, 0});
        auto wo    = sample_sphere(rand2f(rng));
        auto wi    = sample_hair_scattering(brdf, wo, rand2f(rng));
        auto pdf   = sample_hair_scattering_pdf(brdf, wo, wi);
        auto f     = eval_hair_scattering(brdf, wo, wi);
        if (pdf > 0) {
          // Verify that hair BSDF sample weight is close to 1 for _wi_
          if (!(luminance(f) / pdf >= 0.999f && luminance(f) / pdf <= 1.001f))
            throw std::runtime_error("TEST FAILED!");
        }
      }
    }
  }
  printf("OK!\n");
  fflush(stdout);
}

void sampling_consistency_test() {
  auto rng = make_rng(199382389514);
  for (auto beta_m = 0.2f; beta_m < 1.0f; beta_m += 0.2f)
    for (auto beta_n = 0.4f; beta_n < 1.0f; beta_n += 0.2f) {
      // Declare variables for hair sampling test
      const auto count   = 64 * 1024;
      auto       sigma_a = vec3f(0.25f);
      auto       wo      = sample_sphere(rand2f(rng));
      auto       li = [](const vec3f& w) -> vec3f { return vec3f(w.z * w.z); };
      auto       f_importance = vec3f(0), f_uniform = vec3f(0);
      for (auto i = 0; i < count; ++i) {
        // Compute estimates of scattered radiance for hair sampling test
#ifdef YOCTO_EMBREE
        auto h = -1 + 2 * rand1f(rng);
#else
        auto h = rand1f(rng);
#endif
        auto mat   = hair_material{};
        mat.beta_m = beta_m;
        mat.beta_n = beta_n;
        mat.alpha  = 0;
        auto brdf  = eval_hair_brdf(mat, h, {0, 0, 1}, {1, 0, 0});
        auto u     = rand2f(rng);
        auto wi    = sample_hair_scattering(brdf, wo, u);
        auto pdf   = sample_hair_scattering_pdf(brdf, wo, wi);
        auto f     = eval_hair_scattering(brdf, wo, wi);
        if (pdf > 0) f_importance += f * li(wi) / (count * pdf);
        wi = sample_sphere(u);
        f_uniform += eval_hair_scattering(brdf, wo, wi) * li(wi) /
                     (count * sample_sphere_pdf(wi));
      }
      // Verify consistency of estimated hair reflected radiance values
      auto err = abs(luminance(f_importance) - luminance(f_uniform)) /
                 luminance(f_uniform);
      if (err >= 0.05f) throw std::runtime_error("TEST FAILED!");
    }
  printf("OK!\n");
  fflush(stdout);
}

}  // namespace yocto::extension
