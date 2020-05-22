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

static const int   MAX_P           = 3;
static const float sqrt_pi_over_8f = 0.626657069f;

inline float sqr(float x) { return x * x; }

template <int n>
static float pow(float v) {
  float n2 = pow<n / 2>(v);
  return n2 * n2 * pow<n & 1>(v);
}

template <>
float pow<1>(float v) {
  return v;
}

template <>
float pow<0>(float v) {
  return 1;
}

inline float safe_asin(float x) { return asin(clamp(x, -1.0f, 1.0f)); }

inline float safe_sqrt(float x) { return sqrt(max(0.0f, x)); }

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

static std::array<vec3f, MAX_P + 1> ap(
    float cos_theta_o, float eta, float h, const vec3f& T) {
  std::array<vec3f, MAX_P + 1> ap;
  // Compute $p=0$ attenuation at initial cylinder intersection
  float cos_gamma_o = safe_sqrt(1 - h * h);
  float cos_theta   = cos_theta_o * cos_gamma_o;
  float f = fresnel_dielectric(eta, cos_theta);  // TODO: Ci manca un eta
  ap[0]   = vec3f(f);

  // Compute $p=1$ attenuation term
  ap[1] = sqr(1 - f) * T;

  // Compute attenuation terms up to $p=_pMax_$
  for (int p = 2; p < MAX_P; ++p) ap[p] = ap[p - 1] * T * f;

  // Compute attenuation term accounting for remaining orders of scattering
  ap[MAX_P] = ap[MAX_P - 1] * f * T / (vec3f(1.f) - T * f);
  return ap;
}

inline float np(float phi, int p, float s, float gamma_o, float gamma_t) {
  float dphi = phi - extension::phi(p, gamma_o, gamma_t);
  // Remap _dphi_ to $[-\pi,\pi]$
  while (dphi > pif) dphi -= 2 * pif;
  while (dphi < -pif) dphi += 2 * pif;
  return trimmed_logistic(dphi, s, -pif, pif);
}

inline vec3f eval_hair_brdf(float h, float eta, const vec3f& sigma_a,
    float beta_m, float beta_n, float alpha, const vec3f& normal,
    const vec3f& outgoing, const vec3f& incoming) {
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
  float phi_o      = atan2(outgoing.z, outgoing.y);

  // Compute hair coordinate system terms related to _wi_
  float sin_theta_i = incoming.x;
  float cos_theta_i = safe_sqrt(1 - sqr(sin_theta_i));
  float phi_i      = atan2(incoming.z, incoming.y);

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
  std::array<vec3f, MAX_P + 1> ap  = extension::ap(cos_theta_o, eta, h, t);

  vec3f fsum = zero3f;

  // TODO: spostare
  std::array<float, MAX_P + 1> v;
  v[0] = sqr(0.726f * beta_m + 0.812f * sqr(beta_m) + 3.7f * pow<20>(beta_m));
  v[1] = .25 * v[0];
  v[2] = 4 * v[0];
  for (int p = 3; p <= MAX_P; ++p)
    // TODO: is there anything better here?
    v[p] = v[2];

  std::array<float, MAX_P> sin_2k_alpha;
  std::array<float, MAX_P> cos2kAlpha;
  sin_2k_alpha[0] = sin(pif / 180 * alpha);
  cos2kAlpha[0]   = safe_sqrt(1 - sqr(sin_2k_alpha[0]));
  for (int i = 1; i < 3; ++i) {
    sin_2k_alpha[i] = 2 * cos2kAlpha[i - 1] * sin_2k_alpha[i - 1];
    cos2kAlpha[i]   = sqr(cos2kAlpha[i - 1]) - sqr(sin_2k_alpha[i - 1]);
  }
  float gamma_o = safe_asin(h);
  float s       = sqrt_pi_over_8f *
            (0.265f * beta_n + 1.194f * sqr(beta_n) + 5.372f * pow<22>(beta_n));
  // END TODO: spostare

  for (int p = 0; p < MAX_P; ++p) {
    // Compute $\sin \thetao$ and $\cos \thetao$ terms accounting for scales
    float sin_theta_op, cos_theta_op;
    if (p == 0) {
      sin_theta_op = sin_theta_o * cos2kAlpha[1] -
                     cos_theta_o * sin_2k_alpha[1];
      cos_theta_op = cos_theta_o * cos2kAlpha[1] +
                     sin_theta_o * sin_2k_alpha[1];
    }

    // Handle remainder of $p$ values for hair scale tilt
    else if (p == 1) {
      sin_theta_op = sin_theta_o * cos2kAlpha[0] +
                     cos_theta_o * sin_2k_alpha[0];
      cos_theta_op = cos_theta_o * cos2kAlpha[0] -
                     sin_theta_o * sin_2k_alpha[0];
    } else if (p == 2) {
      sin_theta_op = sin_theta_o * cos2kAlpha[2] +
                     cos_theta_o * sin_2k_alpha[2];
      cos_theta_op = cos_theta_o * cos2kAlpha[2] -
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
  fsum += mp(cos_theta_i, cos_theta_o, sin_theta_i, sin_theta_o, v[MAX_P]) *
          ap[MAX_P] / (2.f * pif);

  if (abs(incoming.z) > 0)
    fsum /= abs(incoming.z);  // TODO: DA SISTEMARE AbsCosTheta

  return fsum;
}

inline float sample_hair_brdf_pdf(const vec3f& outgoing, const vec3f& incoming) { // TODO: Capire perchè dopo i parametri c'era CONST
  // Compute hair coordinate system terms related to _wo_
  float sinThetaO = wo.x;
  float cosThetaO = SafeSqrt(1 - Sqr(sinThetaO));
  float phiO      = std::atan2(wo.z, wo.y);

  // Compute hair coordinate system terms related to _wi_
  float sinThetaI = wi.x;
  float cosThetaI = SafeSqrt(1 - Sqr(sinThetaI));
  float phiI      = std::atan2(wi.z, wi.y);

  // Compute $\gammat$ for refracted ray
  float etap      = std::sqrt(eta * eta - Sqr(sinThetaO)) / cosThetaO;
  float sinGammaT = h / etap;
  float gammaT    = SafeASin(sinGammaT);

  // Compute PDF for $A_p$ terms
  std::array<float, pMax + 1> apPdf = ComputeApPdf(cosThetaO);

  // Compute PDF sum for hair scattering events
  float phi = phiI - phiO;
  float pdf = 0;
  for (int p = 0; p < pMax; ++p) {
    // Compute $\sin \thetao$ and $\cos \thetao$ terms accounting for scales
    float sinThetaOp, cosThetaOp;
    if (p == 0) {
      sinThetaOp = sinThetaO * cos2kAlpha[1] - cosThetaO * sin2kAlpha[1];
      cosThetaOp = cosThetaO * cos2kAlpha[1] + sinThetaO * sin2kAlpha[1];
    }

    // Handle remainder of $p$ values for hair scale tilt
    else if (p == 1) {
      sinThetaOp = sinThetaO * cos2kAlpha[0] + cosThetaO * sin2kAlpha[0];
      cosThetaOp = cosThetaO * cos2kAlpha[0] - sinThetaO * sin2kAlpha[0];
    } else if (p == 2) {
      sinThetaOp = sinThetaO * cos2kAlpha[2] + cosThetaO * sin2kAlpha[2];
      cosThetaOp = cosThetaO * cos2kAlpha[2] - sinThetaO * sin2kAlpha[2];
    } else {
      sinThetaOp = sinThetaO;
      cosThetaOp = cosThetaO;
    }

    // Handle out-of-range $\cos \thetao$ from scale adjustment
    cosThetaOp = std::abs(cosThetaOp);
    pdf += Mp(cosThetaI, cosThetaOp, sinThetaI, sinThetaOp, v[p]) * apPdf[p] *
           Np(phi, p, s, gammaO, gammaT);
  }
  pdf += Mp(cosThetaI, cosThetaO, sinThetaI, sinThetaO, v[pMax]) * apPdf[pMax] *
         (1 / (2 * Pi));
  return pdf;
}

}  // namespace yocto::extension
