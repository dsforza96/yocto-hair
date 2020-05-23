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
using math::flt_max;
using math::fmod;
using math::fresnel_conductor;
using math::fresnel_dielectric;
using math::identity3x3f;
using math::invalidb3f;
using math::log;
using math::make_rng;
using math::max;
using math::min;
using math::pif;
using math::pow;
using math::sample_discrete_cdf;
using math::sample_discrete_cdf_pdf;
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
inline float fresnel_dielectric(float eta, float cosw) {
  // Implementation from
  // https://seblagarde.wordpress.com/2013/04/29/memo-on-fresnel-equations/
  auto sin2 = 1 - cosw * cosw;
  auto eta2 = eta * eta;

  auto cos2t = 1 - sin2 / eta2;
  if (cos2t < 0) return 1;  // tir

  auto t0 = sqrt(cos2t);
  auto t1 = eta * t0;
  auto t2 = eta * cosw;

  auto rs = (cosw - t1) / (cosw + t1);
  auto rp = (t0 - t2) / (t0 + t2);

  return (rs * rs + rp * rp) / 2;
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
/*
static hair_brdf eval_hair_brdf(const pathtrace::material* material, float h) {
  auto brdf    = hair_brdf{};
  brdf.sigma_a = material->sigma_a;
  brdf.beta_m  = material->beta_m;
  brdf.beta_n  = material->beta_n;
  brdf.alpha   = material->alpha;
  brdf.eta     = material->eta;

  brdf.h       = h;
  brdf.gamma_o = safe_asin(h);
  brdf.s = sqrt_pi_over_8f * (0.265f * brdf.beta_n + 1.194f * sqr(brdf.beta_n) +
                                 5.372f * pow<22>(brdf.beta_n));

  brdf.v[0] = sqr(0.726f * brdf.beta_m + 0.812f * sqr(brdf.beta_m) +
                  3.7f * pow<20>(brdf.beta_m));
  brdf.v[1] = 0.25 * brdf.v[0];
  brdf.v[2] = 4 * brdf.v[0];
  for (int p = 3; p <= p_max; ++p) brdf.v[p] = brdf.v[2];

  brdf.sin_2k_alpha[0] = sin(pif / 180 * brdf.alpha);
  brdf.cos_2k_alpha[0] = safe_sqrt(1 - sqr(brdf.sin_2k_alpha[0]));
  for (int i = 1; i < p_max; ++i) {
    brdf.sin_2k_alpha[i] = 2 * brdf.cos_2k_alpha[i - 1] *
                           brdf.sin_2k_alpha[i - 1];
    brdf.cos_2k_alpha[i] = sqr(brdf.cos_2k_alpha[i - 1]) -
                           sqr(brdf.sin_2k_alpha[i - 1]);
  }

  return brdf;
}
*/
inline float log_i0(float x) {
  if (x > 12)
    return x + 0.5 * (-log(2 * pif) + log(1 / x) + 1 / (8 * x));
  else
    return log(i0(x));
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

inline float mp(float cos_theta_i, float cos_theta_o, float sin_theta_i,
    float sinTheta_o, float v) {
  float a  = cos_theta_i * cos_theta_o / v;
  float b  = sin_theta_i * sinTheta_o / v;
  float mp = (v <= .1)
                 ? (exp(log_i0(a) - b - 1 / v + 0.6931f + log(1 / (2 * v))))
                 : (exp(-b) * i0(a)) / (sinh(1 / v) * 2 * v);
  return mp;
}

static std::array<vec3f, p_max + 1> ap(
    float cos_theta_o, float eta, float h, const vec3f& T) {
  std::array<vec3f, p_max + 1> ap;
  // Compute $p=0$ attenuation at initial cylinder intersection
  float cos_gamma_o = safe_sqrt(1 - h * h);
  float cos_theta   = cos_theta_o * cos_gamma_o;
  float f = fresnel_dielectric(eta, cos_theta);  // TODO: Ci manca un eta
  ap[0]   = vec3f(f);

  // Compute $p=1$ attenuation term
  ap[1] = sqr(1 - f) * T;

  // Compute attenuation terms up to $p=_pMax_$
  for (int p = 2; p < p_max; ++p) ap[p] = ap[p - 1] * T * f;

  // Compute attenuation term accounting for remaining orders of scattering
  ap[p_max] = ap[p_max - 1] * f * T / (vec3f(1.f) - T * f);
  return ap;
}

inline float np(float phi, int p, float s, float gamma_o, float gamma_t) {
  float dphi = phi - extension::phi(p, gamma_o, gamma_t);
  // Remap _dphi_ to $[-\pi,\pi]$
  while (dphi > pif) dphi -= 2 * pif;
  while (dphi < -pif) dphi += 2 * pif;
  return trimmed_logistic(dphi, s, -pif, pif);
}

static vec3f eval_hair_scattering(const hair_brdf& brdf, const vec3f& normal,
    const vec3f& outgoing, const vec3f& incoming) {
  auto h            = brdf.h;
  auto sigma_a      = brdf.sigma_a;
  auto beta_m       = brdf.beta_n;
  auto beta_n       = brdf.beta_m;
  auto eta          = brdf.eta;
  auto gamma_o      = brdf.gamma_o;
  auto s            = brdf.s;
  auto v            = brdf.v;
  auto sin_2k_alpha = brdf.sin_2k_alpha;
  auto cos_2k_alpha = brdf.cos_2k_alpha;

  // Compute hair coordinate system terms related to _wo_
  /*
  float cos_theta_o = dot(normal, outgoing);
  float sin_theta_o = safe_sqrt(1.0f - sqr(cos_theta_o));
  auto  ortho_o     = orthonormalize(normal, outgoing);  // TODO: test
  float phi_o       = atan2(ortho_o.z, ortho_o.y);

  // Compute hair coordinate system terms related to _wi_
  float cos_theta_i = dot(normal, incoming);
  float sin_theta_i = safe_sqrt(1.0f - sqr(cos_theta_i));
  auto  ortho_i     = orthonormalize(normal, incoming);  // TODO: test
  float phi_i       = atan2(ortho_i.z, ortho_i.y);
  */

  // Compute hair coordinate system terms related to _wo_
  float sin_theta_o = outgoing.x;
  float cos_theta_o = safe_sqrt(1 - sqr(sin_theta_o));
  float phi_o       = atan2(outgoing.z, outgoing.y);

  // Compute hair coordinate system terms related to _wi_
  float sin_theta_i = incoming.x;
  float cos_theta_i = safe_sqrt(1 - sqr(sin_theta_i));
  float phi_i       = atan2(incoming.z, incoming.y);

  // Compute $\cos \thetat$ for refracted ray
  float sin_theta_t = sin_theta_o / eta;
  float cos_theta_t = safe_sqrt(1 - sqr(sin_theta_t));

  float etap        = sqrt(eta * eta - sqr(sin_theta_o)) / cos_theta_o;
  float sin_gamma_t = h / etap;
  float cos_gamma_t = safe_sqrt(1 - sqr(sin_gamma_t));
  float gamma_t     = safe_asin(cos_gamma_t);

  // Compute the transmittance _T_ of a single path through the cylinder
  vec3f t = exp(-sigma_a * (2 * cos_gamma_t / cos_theta_t));

  // Evaluate hair BSDF
  float                        phi = phi_i - phi_o;
  std::array<vec3f, p_max + 1> ap  = extension::ap(cos_theta_o, eta, h, t);

  vec3f fsum = zero3f;
  for (int p = 0; p < p_max; ++p) {
    // Compute $\sin \thetao$ and $\cos \thetao$ terms accounting for scales
    float sin_theta_op, cos_theta_op;
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
          ap[p_max] / (2.f * pif);

  if (abs(incoming.z) > 0)
    fsum /= abs(incoming.z);  // TODO: DA SISTEMARE AbsCosTheta

  return fsum;
}

static std::array<float, p_max + 1> compute_ap_pdf(
    const hair_brdf& brdf, float cos_theta_o) {
  auto sigma_a = brdf.sigma_a;
  auto eta     = brdf.eta;
  auto h       = brdf.h;

  // Compute array of $A_p$ values for _cosThetaO_
  float sin_theta_o = safe_sqrt(1 - cos_theta_o * cos_theta_o);

  // Compute $\cos \thetat$ for refracted ray
  float sin_theta_t = sin_theta_o / eta;
  float cos_theta_t = safe_sqrt(1 - sqr(sin_theta_t));

  // Compute $\gammat$ for refracted ray
  float etap        = sqrt(eta * eta - sqr(sin_theta_o)) / cos_theta_o;
  float sin_gamma_t = h / etap;
  float cos_gamma_t = safe_sqrt(1 - sqr(sin_gamma_t));

  // Compute the transmittance _T_ of a single path through the cylinder
  vec3f t = exp(-sigma_a * (2 * cos_gamma_t / cos_theta_t));
  std::array<vec3f, p_max + 1> ap = extension::ap(cos_theta_o, eta, h, t);

  // Compute $A_p$ PDF from individual $A_p$ terms
  std::array<float, p_max + 1> ap_pdf;
  float                        sum_y = 0;
  for (auto i = 0; i <= p_max; ++i) {
    sum_y += math::luminance(ap[i]);
  }
  for (auto i = 0; i <= p_max; ++i) {
    ap_pdf[i] = math::luminance(ap[i]) / sum_y;
  }

  return ap_pdf;
}

static float sample_hair_scattering_pdf(
    const hair_brdf& brdf, const vec3f& outgoing, const vec3f& incoming) {
  auto h            = brdf.h;
  auto sigma_a      = brdf.sigma_a;
  auto beta_m       = brdf.beta_n;
  auto beta_n       = brdf.beta_m;
  auto eta          = brdf.eta;
  auto gamma_o      = brdf.gamma_o;
  auto s            = brdf.s;
  auto v            = brdf.v;
  auto sin_2k_alpha = brdf.sin_2k_alpha;
  auto cos_2k_alpha = brdf.cos_2k_alpha;

  // Compute hair coordinate system terms related to _wo_
  float sin_theta_o = outgoing.x;
  float cos_theta_o = safe_sqrt(1 - sqr(sin_theta_o));
  float phi_o       = atan2(outgoing.z, outgoing.y);

  // Compute hair coordinate system terms related to _wi_
  float sin_theta_i = incoming.x;
  float cos_theta_i = safe_sqrt(1 - sqr(sin_theta_i));
  float phi_i       = atan2(incoming.z, incoming.y);

  float etap        = sqrt(eta * eta - sqr(sin_theta_o)) / cos_theta_o;
  float sin_gamma_t = h / etap;
  float gamma_t     = safe_asin(sin_gamma_t);

  // Compute PDF for $A_p$ terms
  std::array<float, p_max + 1> ap_pdf = compute_ap_pdf(brdf, cos_theta_o);

  // Compute PDF sum for hair scattering events
  float phi = phi_i - phi_o;
  float pdf = 0;
  for (int p = 0; p < p_max; ++p) {
    // Compute $\sin \thetao$ and $\cos \thetao$ terms accounting for scales
    float sin_theta_op, cos_theta_op;
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
  float k = logistic_cdf(b, s) - logistic_cdf(a, s);
  float x = -s * log(1 / (u * k + logistic_cdf(a, s)) - 1);
  return clamp(x, a, b);
}

static vec3f sample_hair_scattering(const hair_brdf& brdf, vec3f normal,
    const vec3f& outgoing, vec3f* incoming, const vec2f& u2, float* pdf) {
  auto h            = brdf.h;
  auto sigma_a      = brdf.sigma_a;
  auto beta_m       = brdf.beta_n;
  auto beta_n       = brdf.beta_m;
  auto eta          = brdf.eta;
  auto gamma_o      = brdf.gamma_o;
  auto s            = brdf.s;
  auto v            = brdf.v;
  auto sin_2k_alpha = brdf.sin_2k_alpha;
  auto cos_2k_alpha = brdf.cos_2k_alpha;

  // Compute hair coordinate system terms related to _wo_
  float sin_theta_o = outgoing.x;
  float cos_theta_o = safe_sqrt(1 - sqr(sin_theta_o));
  float phi_o       = atan2(outgoing.z, outgoing.y);

  // Derive four random samples from _u2_
  vec2f u[2] = {demux_float(u2[0]), demux_float(u2[1])};

  // Determine which term $p$ to sample for hair scattering
  std::array<float, p_max + 1> ap_pdf = compute_ap_pdf(brdf, cos_theta_o);
  int                          p;
  for (p = 0; p < p_max; ++p) {
    if (u[0][0] < ap_pdf[p]) break;
    u[0][0] -= ap_pdf[p];
  }

  // Rotate $\sin \thetao$ and $\cos \thetao$ to account for hair scale tilt
  float sin_theta_op, cos_theta_op;
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
  u[1][0]           = max(u[1][0], float(1e-5));
  float cos_theta   = 1 + v[p] * log(u[1][0] + (1 - u[1][0]) * exp(-2 / v[p]));
  float sin_theta   = safe_sqrt(1 - sqr(cos_theta));
  float cos_phi     = cos(2 * pif * u[1][1]);
  float sin_theta_i = -cos_theta * cos_theta_op +
                      sin_theta * cos_phi * cos_theta_op;
  float cos_theta_i = safe_sqrt(1 - sqr(sin_theta_i));

  // Sample $N_p$ to compute $\Delta\phi$

  // Compute $\gammat$ for refracted ray
  float etap        = sqrt(eta * eta - sqr(sin_theta_o)) / cos_theta_o;
  float sin_gamma_t = h / etap;
  float gamma_t     = safe_asin(sin_gamma_t);
  float dphi;
  if (p < p_max)
    dphi = phi(p, gamma_o, gamma_t) +
           sample_trimmed_logistic(u[0][1], s, -pif, pif);
  else
    dphi = 2 * pif * u[0][1];

  // Compute _wi_ from sampled hair scattering angles
  float phi_i = phi_o + dphi;
  *incoming   = vec3f{
      sin_theta_i, cos_theta_i * cos(phi_i), cos_theta_i * sin(phi_i)};

  // Compute PDF for sampled hair scattering direction _wi_
  *pdf = 0;
  for (int p = 0; p < p_max; ++p) {
    // Compute $\sin \thetao$ and $\cos \thetao$ terms accounting for scales
    float sin_theta_op, cos_theta_op;
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
    *pdf += mp(cos_theta_o, cos_theta_op, sin_theta_i, sin_theta_op, v[p]) *
            ap_pdf[p] * np(dphi, p, s, gamma_o, gamma_t);
  }
  *pdf += mp(cos_theta_i, cos_theta_o, sin_theta_i, sin_theta_o, v[p_max]) *
          ap_pdf[p_max] * (1 / (2 * pif));
  // if (std::abs(wi->x) < .9999) CHECK_NEAR(*pdf, Pdf(wo, *wi), .01);
  return eval_hair_scattering(brdf, normal, outgoing, *incoming);
}

}  // namespace yocto::extension
