//
// # Yocto/Math: Tiny library for math support in graphics applications.
//
// Yocto/Math provides the basic math primitives used in grahics, including
// small-sized vectors and matrixes, frames, bounding boxes, transforms,
// color and geometry functions, random number generation, noise.
//
//
// ## Small vectors, matrices and frames
//
// We provide common operations for small vectors and matrices typically used
// in graphics. In particular, we support 1-4 dimensional vectors
// coordinates in float and int coordinates (`vec1f`, `vec2f`, `vec3f`, `vec4f`,
// `vec1i`, `vec2i`, `vec3i`, `vec4i`).
//
// We support 2-4 dimensional matrices (`mat2f`, `mat3f`, `mat4f`) with
// matrix-matrix and matrix-std::vector products, transposes and inverses.
// Matrices are stored in column-major order and are accessed and
// constructed by column. The one dimensional version is for completeness only.
//
// To represent transformations, most of the library facilities prefer the use
// coordinate frames, aka rigid transforms, represented as `frame2f` and
// `frame3f`. The structure store three coordinate axes and the origin.
// This is equivalent to a rigid transform written as a column-major affine
// matrix. Transform operations are fater with this representation.
//
//
// ## Rays and bounding boxes
//
// We represent rays in 2-3 dimensions with `ray2f`, `ray3f`.
// Each ray support initialization and evaluation.
//
// We represent bounding boxes in 2-3 dimensions with `bbox2f`, `bbox3f`.
// Each bounding box support construction from points and other bounding box.
// We provide operations to compute bounds for points, lines, triangles and
// quads.
//
//
// ## Transforms
//
// For both matrices and frames we support transform operations for points,
// vectors and directions (`transform_point()`, `transform_vector()`,
// `transform_direction()`). Transform matrices and frames can be
// constructed from basic translation, rotation and scaling, e.g. with
// `translation_mat()` or `translation_frame()` respectively, etc.
// For rotation we support axis-angle and quaternions, with slerp.
//
// TODO: better documentation
//
//
// ## Geometry functions
//
// The library supports basic geomtry functions such as computing
// line/triangle/quad normals and areas, picking points on triangles
// and the like. In these functions, triangles are parameterized with uv written
// w.r.t the (p1-p0) and (p2-p0) axis respectively. Quads are internally handled
// as pairs of two triangles (p0,p1,p3) and (p2,p3,p1), with the uv coordinates
// of the second triangle corrected as 1-u and 1-v to produce a quad
// parametrization where u and v go from 0 to 1. Degenerate quads with p2==p3
// represent triangles correctly, and this convention is used throught the
// library. This is equivalent to Intel's Embree.
//
// TODO: better documentation
//
//
// ## Color funtions
//
// This library support a small number of color operations helpful in writing
// graphics applications. In particular, we support color conversion to/from
// linear rgb, srgb, hsv, xyz, byte to flot color conversions and a few color
// manipulations like contrast and saturation.
//
// TODO: better documentation
//
//
// ## Random Number Generation
//
// This library supports generting random numbers using the PCG32 algorithm,
// that is a portable generator well suited for graphics applications.
//
// 1. initialize the random number generator with `make_rng()`
// 2. if necessary, you can reseed the rng with `seed_rng()`
// 3. generate random integers in an interval with `rand1i()`
// 4. generate random floats and double in the [0,1) range with `rand1f()`,
//    `rand2f()`, `rand3f()`, `rand1d()`
//
//
// ## Noise Functions
//
// We support generation of Perlin noise based on the stb libraries.
//
// 1. use `perlin_noise()` to generate Perlin noise with optional wrapping
// 2. use `perlin_ridge()`, `perlin_fbm()` and `perlin_turbulence()` for fractal
//    noises
//
//
// ## Shading functions
//
// We include a few functions to help writing shaders for path tracing.
//
// 1. use `fresnel_dielectric()` or `fresnel_conductor()` to evaluate the
//    fresnel term for dielectrics or conductors; use `fresnel_schlick()` for
//    the Schlick fresnel approximation
// 2. use `eta_to_reflectivity()` and `reflective_to_eta()` to convert eta to
//    reflectivity and vice-versa; use `eta_to_edgetint()` and
//    `edgetint_to_eta()`
// 3. use `microfacet_distribution()` and `microfacet_shadowing()` to evaluate
//    a microfacet distribution and its associated shadowing term
// 4. evaluate BRDF lobes with
//    - `eval_diffuse_reflection()`: diffuse brdf
//    - `eval_microfacet_reflection()`: specular brdf for dielectrics and metals
//    - `eval_microfacet_transmission()`: transmission brdf for thin dielectrics
//    - `eval_microfacet_refraction()`: refraction brdf for dielectrics
// 5. sample BRDF lobes with `sample_XXX()` using the above lobe names
// 6. compute the PDF for BRDF lobe sampling with `sample_XXX_pdf()` using the
//    above lobe names
//
//
// ## Monte Carlo helpers
//
// We include many method to generate random points and directions. These may be
// used in path tracing or procedural generation.
//
// 1. use `sample_XXX()` to warp random numbers in [0,1)^k domains to the
//   desired domain; in particular we support `sample_hemisphere()`,
//   `sample_sphere()`, `sample_hemisphere_cos()`,
//   `sample_hemisphere_cospower()`. `sample_disk()`. `sample_cylinder()`.
//   `sample_triangle()`, `sample_quad()`
// 2. use `sample_discrete()` to sample from a descreet distribution
// 3. use `sample_XXX_pdf()` to compute the PDF of the sampling functions
//
//

//
// LICENSE:
//
// Copyright (c) 2016 -- 2020 Fabio Pellacini
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
// LICENSE OF INCLUDED SOFTWARE for Pcg random number generator
//
// This code also includes a small exerpt from http://www.pcg-random.org/
// licensed as follows
// *Really* minimal PCG32 code / (c) 2014 M.E. O'Neill / pcg-random.org
// Licensed under Apache License 2.0 (NO WARRANTY, etc. see website)
//
//
// LICENCE OF INCLUDED SOFTWARE FOR PERLIN NOISE
// https://github.com/nothings/stb/blob/master/stb_perlin.h
//
// -----------------------------------------------------------------------------
// ALTERNATIVE B - Public Domain (www.unlicense.org)
// This is free and unencumbered software released into the public domain.
// Anyone is free to copy, modify, publish, use, compile, sell, or distribute
// this software, either in source code form or as a compiled binary, for any
// purpose, commercial or non-commercial, and by any means. In jurisdictions
// that recognize copyright laws, the author or authors of this software
// dedicate any and all copyright interest in the software to the public domain.
// We make this dedication for the benefit of the public at large and to the
// detriment of our heirs and successors. We intend this dedication to be an
// overt act of relinquishment in perpetuity of all present and future rights to
// this software under copyright law.
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
// ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
// WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
// -----------------------------------------------------------------------------
//
//

#ifndef _YOCTO_MATH_H_
#define _YOCTO_MATH_H_

// -----------------------------------------------------------------------------
// INCLUDES
// -----------------------------------------------------------------------------

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <vector>

// -----------------------------------------------------------------------------
// MATH CONSTANTS AND FUNCTIONS
// -----------------------------------------------------------------------------
namespace yocto::math {

using byte   = unsigned char;
using uint   = unsigned int;
using ushort = unsigned short;

inline const double pi  = 3.14159265358979323846;
inline const float  pif = (float)pi;

inline const auto int_max = std::numeric_limits<int>::max();
inline const auto int_min = std::numeric_limits<int>::lowest();
inline const auto flt_max = std::numeric_limits<float>::max();
inline const auto flt_min = std::numeric_limits<float>::lowest();
inline const auto flt_eps = std::numeric_limits<float>::epsilon();

inline float abs(float a);
inline float min(float a, float b);
inline float max(float a, float b);
inline float clamp(float a, float min, float max);
inline float sign(float a);
inline float sqrt(float a);
inline float sin(float a);
inline float cos(float a);
inline float tan(float a);
inline float asin(float a);
inline float acos(float a);
inline float atan(float a);
inline float log(float a);
inline float exp(float a);
inline float log2(float a);
inline float exp2(float a);
inline float pow(float a, float b);
inline float isfinite(float a);
inline float atan2(float a, float b);
inline float fmod(float a, float b);
inline float radians(float a);
inline float degrees(float a);
inline float lerp(float a, float b, float u);
inline void  swap(float& a, float& b);
inline float smoothstep(float a, float b, float u);
inline float bias(float a, float bias);
inline float gain(float a, float gain);

inline int  abs(int a);
inline int  min(int a, int b);
inline int  max(int a, int b);
inline int  clamp(int a, int min, int max);
inline int  sign(int a);
inline int  pow2(int a);
inline void swap(int& a, int& b);

}  // namespace yocto::math

// -----------------------------------------------------------------------------
// VECTORS
// -----------------------------------------------------------------------------
namespace yocto::math {

struct vec2f {
  float x = 0;
  float y = 0;

  vec2f();
  vec2f(float x, float y);
  explicit vec2f(float v);
  explicit operator bool() const;

  float&       operator[](int i);
  const float& operator[](int i) const;
};

struct vec3f {
  float x = 0;
  float y = 0;
  float z = 0;

  vec3f();
  vec3f(float x, float y, float z);
  vec3f(const vec2f& v, float z);
  explicit vec3f(float v);
  explicit operator bool() const;

  float&       operator[](int i);
  const float& operator[](int i) const;
};

struct vec4f {
  float x = 0;
  float y = 0;
  float z = 0;
  float w = 0;

  vec4f();
  vec4f(float x, float y, float z, float w);
  vec4f(const vec3f& v, float w);
  explicit vec4f(float v);
  explicit operator bool() const;

  float&       operator[](int i);
  const float& operator[](int i) const;
};

// Zero std::vector constants.
inline const auto zero2f = vec2f{0, 0};
inline const auto zero3f = vec3f{0, 0, 0};
inline const auto zero4f = vec4f{0, 0, 0, 0};

// Element access
inline vec3f&       xyz(vec4f& a);
inline const vec3f& xyz(const vec4f& a);

// Vector comparison operations.
inline bool operator==(const vec2f& a, const vec2f& b);
inline bool operator!=(const vec2f& a, const vec2f& b);

// Vector operations.
inline vec2f operator+(const vec2f& a);
inline vec2f operator-(const vec2f& a);
inline vec2f operator+(const vec2f& a, const vec2f& b);
inline vec2f operator+(const vec2f& a, float b);
inline vec2f operator+(float a, const vec2f& b);
inline vec2f operator-(const vec2f& a, const vec2f& b);
inline vec2f operator-(const vec2f& a, float b);
inline vec2f operator-(float a, const vec2f& b);
inline vec2f operator*(const vec2f& a, const vec2f& b);
inline vec2f operator*(const vec2f& a, float b);
inline vec2f operator*(float a, const vec2f& b);
inline vec2f operator/(const vec2f& a, const vec2f& b);
inline vec2f operator/(const vec2f& a, float b);
inline vec2f operator/(float a, const vec2f& b);

// Vector assignments
inline vec2f& operator+=(vec2f& a, const vec2f& b);
inline vec2f& operator+=(vec2f& a, float b);
inline vec2f& operator-=(vec2f& a, const vec2f& b);
inline vec2f& operator-=(vec2f& a, float b);
inline vec2f& operator*=(vec2f& a, const vec2f& b);
inline vec2f& operator*=(vec2f& a, float b);
inline vec2f& operator/=(vec2f& a, const vec2f& b);
inline vec2f& operator/=(vec2f& a, float b);

// Vector products and lengths.
inline float dot(const vec2f& a, const vec2f& b);
inline float cross(const vec2f& a, const vec2f& b);

inline float length(const vec2f& a);
inline vec2f normalize(const vec2f& a);
inline float distance(const vec2f& a, const vec2f& b);
inline float distance_squared(const vec2f& a, const vec2f& b);

// Max element and clamp.
inline vec2f max(const vec2f& a, float b);
inline vec2f min(const vec2f& a, float b);
inline vec2f max(const vec2f& a, const vec2f& b);
inline vec2f min(const vec2f& a, const vec2f& b);
inline vec2f clamp(const vec2f& x, float min, float max);
inline vec2f lerp(const vec2f& a, const vec2f& b, float u);
inline vec2f lerp(const vec2f& a, const vec2f& b, const vec2f& u);

inline float max(const vec2f& a);
inline float min(const vec2f& a);
inline float sum(const vec2f& a);
inline float mean(const vec2f& a);

// Functions applied to std::vector elements
inline vec2f abs(const vec2f& a);
inline vec2f sqrt(const vec2f& a);
inline vec2f exp(const vec2f& a);
inline vec2f log(const vec2f& a);
inline vec2f exp2(const vec2f& a);
inline vec2f log2(const vec2f& a);
inline bool  isfinite(const vec2f& a);
inline vec2f pow(const vec2f& a, float b);
inline vec2f pow(const vec2f& a, const vec2f& b);
inline vec2f gain(const vec2f& a, float b);
inline void  swap(vec2f& a, vec2f& b);

// Vector comparison operations.
inline bool operator==(const vec3f& a, const vec3f& b);
inline bool operator!=(const vec3f& a, const vec3f& b);

// Vector operations.
inline vec3f operator+(const vec3f& a);
inline vec3f operator-(const vec3f& a);
inline vec3f operator+(const vec3f& a, const vec3f& b);
inline vec3f operator+(const vec3f& a, float b);
inline vec3f operator+(float a, const vec3f& b);
inline vec3f operator-(const vec3f& a, const vec3f& b);
inline vec3f operator-(const vec3f& a, float b);
inline vec3f operator-(float a, const vec3f& b);
inline vec3f operator*(const vec3f& a, const vec3f& b);
inline vec3f operator*(const vec3f& a, float b);
inline vec3f operator*(float a, const vec3f& b);
inline vec3f operator/(const vec3f& a, const vec3f& b);
inline vec3f operator/(const vec3f& a, float b);
inline vec3f operator/(float a, const vec3f& b);

// Vector assignments
inline vec3f& operator+=(vec3f& a, const vec3f& b);
inline vec3f& operator+=(vec3f& a, float b);
inline vec3f& operator-=(vec3f& a, const vec3f& b);
inline vec3f& operator-=(vec3f& a, float b);
inline vec3f& operator*=(vec3f& a, const vec3f& b);
inline vec3f& operator*=(vec3f& a, float b);
inline vec3f& operator/=(vec3f& a, const vec3f& b);
inline vec3f& operator/=(vec3f& a, float b);

// Vector products and lengths.
inline float dot(const vec3f& a, const vec3f& b);
inline vec3f cross(const vec3f& a, const vec3f& b);

inline float length(const vec3f& a);
inline vec3f normalize(const vec3f& a);
inline float distance(const vec3f& a, const vec3f& b);
inline float distance_squared(const vec3f& a, const vec3f& b);

inline float angle(const vec3f& a, const vec3f& b);

// Orthogonal vectors.
inline vec3f orthogonal(const vec3f& v);
inline vec3f orthonormalize(const vec3f& a, const vec3f& b);

// Reflected and refracted std::vector.
inline vec3f reflect(const vec3f& w, const vec3f& n);
inline vec3f refract(const vec3f& w, const vec3f& n, float inv_eta);

// Max element and clamp.
inline vec3f max(const vec3f& a, float b);
inline vec3f min(const vec3f& a, float b);
inline vec3f max(const vec3f& a, const vec3f& b);
inline vec3f min(const vec3f& a, const vec3f& b);
inline vec3f clamp(const vec3f& x, float min, float max);
inline vec3f lerp(const vec3f& a, const vec3f& b, float u);
inline vec3f lerp(const vec3f& a, const vec3f& b, const vec3f& u);

inline float max(const vec3f& a);
inline float min(const vec3f& a);
inline float sum(const vec3f& a);
inline float mean(const vec3f& a);

// Functions applied to std::vector elements
inline vec3f abs(const vec3f& a);
inline vec3f sqrt(const vec3f& a);
inline vec3f exp(const vec3f& a);
inline vec3f log(const vec3f& a);
inline vec3f exp2(const vec3f& a);
inline vec3f log2(const vec3f& a);
inline vec3f pow(const vec3f& a, float b);
inline vec3f pow(const vec3f& a, const vec3f& b);
inline vec3f gain(const vec3f& a, float b);
inline bool  isfinite(const vec3f& a);
inline void  swap(vec3f& a, vec3f& b);

// Vector comparison operations.
inline bool operator==(const vec4f& a, const vec4f& b);
inline bool operator!=(const vec4f& a, const vec4f& b);

// Vector operations.
inline vec4f operator+(const vec4f& a);
inline vec4f operator-(const vec4f& a);
inline vec4f operator+(const vec4f& a, const vec4f& b);
inline vec4f operator+(const vec4f& a, float b);
inline vec4f operator+(float a, const vec4f& b);
inline vec4f operator-(const vec4f& a, const vec4f& b);
inline vec4f operator-(const vec4f& a, float b);
inline vec4f operator-(float a, const vec4f& b);
inline vec4f operator*(const vec4f& a, const vec4f& b);
inline vec4f operator*(const vec4f& a, float b);
inline vec4f operator*(float a, const vec4f& b);
inline vec4f operator/(const vec4f& a, const vec4f& b);
inline vec4f operator/(const vec4f& a, float b);
inline vec4f operator/(float a, const vec4f& b);

// Vector assignments
inline vec4f& operator+=(vec4f& a, const vec4f& b);
inline vec4f& operator+=(vec4f& a, float b);
inline vec4f& operator-=(vec4f& a, const vec4f& b);
inline vec4f& operator-=(vec4f& a, float b);
inline vec4f& operator*=(vec4f& a, const vec4f& b);
inline vec4f& operator*=(vec4f& a, float b);
inline vec4f& operator/=(vec4f& a, const vec4f& b);
inline vec4f& operator/=(vec4f& a, float b);

// Vector products and lengths.
inline float dot(const vec4f& a, const vec4f& b);
inline float length(const vec4f& a);
inline vec4f normalize(const vec4f& a);
inline float distance(const vec4f& a, const vec4f& b);
inline float distance_squared(const vec4f& a, const vec4f& b);

inline vec4f slerp(const vec4f& a, const vec4f& b, float u);

// Max element and clamp.
inline vec4f max(const vec4f& a, float b);
inline vec4f min(const vec4f& a, float b);
inline vec4f max(const vec4f& a, const vec4f& b);
inline vec4f min(const vec4f& a, const vec4f& b);
inline vec4f clamp(const vec4f& x, float min, float max);
inline vec4f lerp(const vec4f& a, const vec4f& b, float u);
inline vec4f lerp(const vec4f& a, const vec4f& b, const vec4f& u);

inline float max(const vec4f& a);
inline float min(const vec4f& a);
inline float sum(const vec4f& a);
inline float mean(const vec4f& a);

// Functions applied to std::vector elements
inline vec4f abs(const vec4f& a);
inline vec4f sqrt(const vec4f& a);
inline vec4f exp(const vec4f& a);
inline vec4f log(const vec4f& a);
inline vec4f exp2(const vec4f& a);
inline vec4f log2(const vec4f& a);
inline vec4f pow(const vec4f& a, float b);
inline vec4f pow(const vec4f& a, const vec4f& b);
inline vec4f gain(const vec4f& a, float b);
inline bool  isfinite(const vec4f& a);
inline void  swap(vec4f& a, vec4f& b);

// Quaternion operatons represented as xi + yj + zk + w
// const auto identity_quat4f = vec4f{0, 0, 0, 1};
inline vec4f quat_mul(const vec4f& a, float b);
inline vec4f quat_mul(const vec4f& a, const vec4f& b);
inline vec4f quat_conjugate(const vec4f& a);
inline vec4f quat_inverse(const vec4f& a);

}  // namespace yocto::math

// -----------------------------------------------------------------------------
// INTEGER VECTORS
// -----------------------------------------------------------------------------
namespace yocto::math {

struct vec2i {
  int x = 0;
  int y = 0;

  vec2i();
  vec2i(int x, int y);
  explicit vec2i(int v);
  explicit operator vec2f() const;
  explicit operator bool() const;

  int&       operator[](int i);
  const int& operator[](int i) const;
};

struct vec3i {
  int x = 0;
  int y = 0;
  int z = 0;

  vec3i();
  vec3i(int x, int y, int z);
  vec3i(const vec2i& v, int z);
  explicit vec3i(int v);
  explicit operator vec3f() const;
  explicit operator bool() const;

  int&       operator[](int i);
  const int& operator[](int i) const;
};

struct vec4i {
  int x = 0;
  int y = 0;
  int z = 0;
  int w = 0;

  vec4i();
  vec4i(int x, int y, int z, int w);
  vec4i(const vec3i& v, int w);
  explicit vec4i(int v);
  explicit operator vec4f() const;
  explicit operator bool() const;

  int&       operator[](int i);
  const int& operator[](int i) const;
};

struct vec3b {
  byte x = 0;
  byte y = 0;
  byte z = 0;

  vec3b();
  vec3b(byte x, byte y, byte z);
  explicit vec3b(byte v);
  explicit operator bool() const;

  byte&       operator[](int i);
  const byte& operator[](int i) const;
};

struct vec4b {
  byte x = 0;
  byte y = 0;
  byte z = 0;
  byte w = 0;

  vec4b();
  vec4b(byte x, byte y, byte z, byte w);
  explicit vec4b(byte v);
  explicit operator bool() const;

  byte&       operator[](int i);
  const byte& operator[](int i) const;
};

// Zero std::vector constants.
inline const auto zero2i = vec2i{0, 0};
inline const auto zero3i = vec3i{0, 0, 0};
inline const auto zero4i = vec4i{0, 0, 0, 0};
inline const auto zero3b = vec3b{0, 0, 0};
inline const auto zero4b = vec4b{0, 0, 0, 0};

// Element access
inline vec3i&       xyz(vec4i& a);
inline const vec3i& xyz(const vec4i& a);

// Element access
inline vec3b&       xyz(vec4b& a);
inline const vec3b& xyz(const vec4b& a);

// Vector comparison operations.
inline bool operator==(const vec2i& a, const vec2i& b);
inline bool operator!=(const vec2i& a, const vec2i& b);

// Vector operations.
inline vec2i operator+(const vec2i& a);
inline vec2i operator-(const vec2i& a);
inline vec2i operator+(const vec2i& a, const vec2i& b);
inline vec2i operator+(const vec2i& a, int b);
inline vec2i operator+(int a, const vec2i& b);
inline vec2i operator-(const vec2i& a, const vec2i& b);
inline vec2i operator-(const vec2i& a, int b);
inline vec2i operator-(int a, const vec2i& b);
inline vec2i operator*(const vec2i& a, const vec2i& b);
inline vec2i operator*(const vec2i& a, int b);
inline vec2i operator*(int a, const vec2i& b);
inline vec2i operator/(const vec2i& a, const vec2i& b);
inline vec2i operator/(const vec2i& a, int b);
inline vec2i operator/(int a, const vec2i& b);

// Vector assignments
inline vec2i& operator+=(vec2i& a, const vec2i& b);
inline vec2i& operator+=(vec2i& a, int b);
inline vec2i& operator-=(vec2i& a, const vec2i& b);
inline vec2i& operator-=(vec2i& a, int b);
inline vec2i& operator*=(vec2i& a, const vec2i& b);
inline vec2i& operator*=(vec2i& a, int b);
inline vec2i& operator/=(vec2i& a, const vec2i& b);
inline vec2i& operator/=(vec2i& a, int b);

// Max element and clamp.
inline vec2i max(const vec2i& a, int b);
inline vec2i min(const vec2i& a, int b);
inline vec2i max(const vec2i& a, const vec2i& b);
inline vec2i min(const vec2i& a, const vec2i& b);
inline vec2i clamp(const vec2i& x, int min, int max);

inline int max(const vec2i& a);
inline int min(const vec2i& a);
inline int sum(const vec2i& a);

// Functions applied to std::vector elements
inline vec2i abs(const vec2i& a);
inline void  swap(vec2i& a, vec2i& b);

// Vector comparison operations.
inline bool operator==(const vec3i& a, const vec3i& b);
inline bool operator!=(const vec3i& a, const vec3i& b);

// Vector operations.
inline vec3i operator+(const vec3i& a);
inline vec3i operator-(const vec3i& a);
inline vec3i operator+(const vec3i& a, const vec3i& b);
inline vec3i operator+(const vec3i& a, int b);
inline vec3i operator+(int a, const vec3i& b);
inline vec3i operator-(const vec3i& a, const vec3i& b);
inline vec3i operator-(const vec3i& a, int b);
inline vec3i operator-(int a, const vec3i& b);
inline vec3i operator*(const vec3i& a, const vec3i& b);
inline vec3i operator*(const vec3i& a, int b);
inline vec3i operator*(int a, const vec3i& b);
inline vec3i operator/(const vec3i& a, const vec3i& b);
inline vec3i operator/(const vec3i& a, int b);
inline vec3i operator/(int a, const vec3i& b);

// Vector assignments
inline vec3i& operator+=(vec3i& a, const vec3i& b);
inline vec3i& operator+=(vec3i& a, int b);
inline vec3i& operator-=(vec3i& a, const vec3i& b);
inline vec3i& operator-=(vec3i& a, int b);
inline vec3i& operator*=(vec3i& a, const vec3i& b);
inline vec3i& operator*=(vec3i& a, int b);
inline vec3i& operator/=(vec3i& a, const vec3i& b);
inline vec3i& operator/=(vec3i& a, int b);

// Max element and clamp.
inline vec3i max(const vec3i& a, int b);
inline vec3i min(const vec3i& a, int b);
inline vec3i max(const vec3i& a, const vec3i& b);
inline vec3i min(const vec3i& a, const vec3i& b);
inline vec3i clamp(const vec3i& x, int min, int max);

inline int max(const vec3i& a);
inline int min(const vec3i& a);
inline int sum(const vec3i& a);

// Functions applied to std::vector elements
inline vec3i abs(const vec3i& a);
inline void  swap(vec3i& a, vec3i& b);

// Vector comparison operations.
inline bool operator==(const vec4i& a, const vec4i& b);
inline bool operator!=(const vec4i& a, const vec4i& b);

// Vector operations.
inline vec4i operator+(const vec4i& a);
inline vec4i operator-(const vec4i& a);
inline vec4i operator+(const vec4i& a, const vec4i& b);
inline vec4i operator+(const vec4i& a, int b);
inline vec4i operator+(int a, const vec4i& b);
inline vec4i operator-(const vec4i& a, const vec4i& b);
inline vec4i operator-(const vec4i& a, int b);
inline vec4i operator-(int a, const vec4i& b);
inline vec4i operator*(const vec4i& a, const vec4i& b);
inline vec4i operator*(const vec4i& a, int b);
inline vec4i operator*(int a, const vec4i& b);
inline vec4i operator/(const vec4i& a, const vec4i& b);
inline vec4i operator/(const vec4i& a, int b);
inline vec4i operator/(int a, const vec4i& b);

// Vector assignments
inline vec4i& operator+=(vec4i& a, const vec4i& b);
inline vec4i& operator+=(vec4i& a, int b);
inline vec4i& operator-=(vec4i& a, const vec4i& b);
inline vec4i& operator-=(vec4i& a, int b);
inline vec4i& operator*=(vec4i& a, const vec4i& b);
inline vec4i& operator*=(vec4i& a, int b);
inline vec4i& operator/=(vec4i& a, const vec4i& b);
inline vec4i& operator/=(vec4i& a, int b);

// Max element and clamp.
inline vec4i max(const vec4i& a, int b);
inline vec4i min(const vec4i& a, int b);
inline vec4i max(const vec4i& a, const vec4i& b);
inline vec4i min(const vec4i& a, const vec4i& b);
inline vec4i clamp(const vec4i& x, int min, int max);

inline int max(const vec4i& a);
inline int min(const vec4i& a);
inline int sum(const vec4i& a);

// Functions applied to std::vector elements
inline vec4i abs(const vec4i& a);
inline void  swap(vec4i& a, vec4i& b);

}  // namespace yocto::math

// -----------------------------------------------------------------------------
// VECTOR HASHING
// -----------------------------------------------------------------------------
namespace std {

// Hash functor for std::vector for use with hash_map
template <>
struct hash<yocto::math::vec2i>;
template <>
struct hash<yocto::math::vec3i>;
template <>
struct hash<yocto::math::vec4i>;

}  // namespace std

// -----------------------------------------------------------------------------
// MATRICES
// -----------------------------------------------------------------------------
namespace yocto::math {

// Small Fixed-size matrices stored in column major format.
struct mat2f {
  vec2f x = {1, 0};
  vec2f y = {0, 1};

  mat2f();
  mat2f(const vec2f& x, const vec2f& y);

  vec2f&       operator[](int i);
  const vec2f& operator[](int i) const;
};

// Small Fixed-size matrices stored in column major format.
struct mat3f {
  vec3f x = {1, 0, 0};
  vec3f y = {0, 1, 0};
  vec3f z = {0, 0, 1};

  mat3f();
  mat3f(const vec3f& x, const vec3f& y, const vec3f& z);

  vec3f&       operator[](int i);
  const vec3f& operator[](int i) const;
};

// Small Fixed-size matrices stored in column major format.
struct mat4f {
  vec4f x = {1, 0, 0, 0};
  vec4f y = {0, 1, 0, 0};
  vec4f z = {0, 0, 1, 0};
  vec4f w = {0, 0, 0, 1};

  mat4f();
  mat4f(const vec4f& x, const vec4f& y, const vec4f& z, const vec4f& w);

  vec4f&       operator[](int i);
  const vec4f& operator[](int i) const;
};

// Identity matrices constants.
inline const auto identity2x2f = mat2f{{1, 0}, {0, 1}};
inline const auto identity3x3f = mat3f{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
inline const auto identity4x4f = mat4f{
    {1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};

// Matrix comparisons.
inline bool operator==(const mat2f& a, const mat2f& b);
inline bool operator!=(const mat2f& a, const mat2f& b);

// Matrix operations.
inline mat2f operator+(const mat2f& a, const mat2f& b);
inline mat2f operator*(const mat2f& a, float b);
inline vec2f operator*(const mat2f& a, const vec2f& b);
inline vec2f operator*(const vec2f& a, const mat2f& b);
inline mat2f operator*(const mat2f& a, const mat2f& b);

// Matrix assignments.
inline mat2f& operator+=(mat2f& a, const mat2f& b);
inline mat2f& operator*=(mat2f& a, const mat2f& b);
inline mat2f& operator*=(mat2f& a, float b);

// Matrix diagonals and transposes.
inline vec2f diagonal(const mat2f& a);
inline mat2f transpose(const mat2f& a);

// Matrix adjoints, determinants and inverses.
inline float determinant(const mat2f& a);
inline mat2f adjoint(const mat2f& a);
inline mat2f inverse(const mat2f& a);

// Matrix comparisons.
inline bool operator==(const mat3f& a, const mat3f& b);
inline bool operator!=(const mat3f& a, const mat3f& b);

// Matrix operations.
inline mat3f operator+(const mat3f& a, const mat3f& b);
inline mat3f operator*(const mat3f& a, float b);
inline vec3f operator*(const mat3f& a, const vec3f& b);
inline vec3f operator*(const vec3f& a, const mat3f& b);
inline mat3f operator*(const mat3f& a, const mat3f& b);

// Matrix assignments.
inline mat3f& operator+=(mat3f& a, const mat3f& b);
inline mat3f& operator*=(mat3f& a, const mat3f& b);
inline mat3f& operator*=(mat3f& a, float b);

// Matrix diagonals and transposes.
inline vec3f diagonal(const mat3f& a);
inline mat3f transpose(const mat3f& a);

// Matrix adjoints, determinants and inverses.
inline float determinant(const mat3f& a);
inline mat3f adjoint(const mat3f& a);
inline mat3f inverse(const mat3f& a);

// Constructs a basis from a direction
inline mat3f basis_fromz(const vec3f& v);

// Matrix comparisons.
inline bool operator==(const mat4f& a, const mat4f& b);
inline bool operator!=(const mat4f& a, const mat4f& b);

// Matrix operations.
inline mat4f operator+(const mat4f& a, const mat4f& b);
inline mat4f operator*(const mat4f& a, float b);
inline vec4f operator*(const mat4f& a, const vec4f& b);
inline vec4f operator*(const vec4f& a, const mat4f& b);
inline mat4f operator*(const mat4f& a, const mat4f& b);

// Matrix assignments.
inline mat4f& operator+=(mat4f& a, const mat4f& b);
inline mat4f& operator*=(mat4f& a, const mat4f& b);
inline mat4f& operator*=(mat4f& a, float b);

// Matrix diagonals and transposes.
inline vec4f diagonal(const mat4f& a);
inline mat4f transpose(const mat4f& a);

}  // namespace yocto::math

// -----------------------------------------------------------------------------
// RIGID BODY TRANSFORMS/FRAMES
// -----------------------------------------------------------------------------
namespace yocto::math {

// Rigid frames stored as a column-major affine transform matrix.
struct frame2f {
  vec2f x = {1, 0};
  vec2f y = {0, 1};
  vec2f o = {0, 0};

  frame2f();
  frame2f(const vec2f& x, const vec2f& y, const vec2f& o);
  explicit frame2f(const vec2f& o);
  frame2f(const mat2f& m, const vec2f& t);
  explicit frame2f(const mat3f& m);
  operator mat3f() const;

  vec2f&       operator[](int i);
  const vec2f& operator[](int i) const;
};

// Rigid frames stored as a column-major affine transform matrix.
struct frame3f {
  vec3f x = {1, 0, 0};
  vec3f y = {0, 1, 0};
  vec3f z = {0, 0, 1};
  vec3f o = {0, 0, 0};

  frame3f();
  frame3f(const vec3f& x, const vec3f& y, const vec3f& z, const vec3f& o);
  explicit frame3f(const vec3f& o);
  frame3f(const mat3f& m, const vec3f& t);
  explicit frame3f(const mat4f& m);
  operator mat4f() const;

  vec3f&       operator[](int i);
  const vec3f& operator[](int i) const;
};

// Indentity frames.
inline const auto identity2x3f = frame2f{{1, 0}, {0, 1}, {0, 0}};
inline const auto identity3x4f = frame3f{
    {1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {0, 0, 0}};

// Frame properties
inline const mat2f& rotation(const frame2f& a);

// Frame comparisons.
inline bool operator==(const frame2f& a, const frame2f& b);
inline bool operator!=(const frame2f& a, const frame2f& b);

// Frame composition, equivalent to affine matrix product.
inline frame2f  operator*(const frame2f& a, const frame2f& b);
inline frame2f& operator*=(frame2f& a, const frame2f& b);

// Frame inverse, equivalent to rigid affine inverse.
inline frame2f inverse(const frame2f& a, bool non_rigid = false);

// Frame properties
inline const mat3f& rotation(const frame3f& a);

// Frame comparisons.
inline bool operator==(const frame3f& a, const frame3f& b);
inline bool operator!=(const frame3f& a, const frame3f& b);

// Frame composition, equivalent to affine matrix product.
inline frame3f  operator*(const frame3f& a, const frame3f& b);
inline frame3f& operator*=(frame3f& a, const frame3f& b);

// Frame inverse, equivalent to rigid affine inverse.
inline frame3f inverse(const frame3f& a, bool non_rigid = false);

// Frame construction from axis.
inline frame3f frame_fromz(const vec3f& o, const vec3f& v);
inline frame3f frame_fromzx(const vec3f& o, const vec3f& z_, const vec3f& x_);

}  // namespace yocto::math

// -----------------------------------------------------------------------------
// QUATERNIONS
// -----------------------------------------------------------------------------
namespace yocto::math {

// Quaternions to represent rotations
struct quat4f {
  float x = 0;
  float y = 0;
  float z = 0;
  float w = 0;

  // constructors
  quat4f();
  quat4f(float x, float y, float z, float w);
};

// Constants
inline const auto identity_quat4f = quat4f{0, 0, 0, 1};

// Quaternion operatons
inline quat4f operator+(const quat4f& a, const quat4f& b);
inline quat4f operator*(const quat4f& a, float b);
inline quat4f operator/(const quat4f& a, float b);
inline quat4f operator*(const quat4f& a, const quat4f& b);

// Quaterion operations
inline float  dot(const quat4f& a, const quat4f& b);
inline float  length(const quat4f& a);
inline quat4f normalize(const quat4f& a);
inline quat4f conjugate(const quat4f& a);
inline quat4f inverse(const quat4f& a);
inline float  uangle(const quat4f& a, const quat4f& b);
inline quat4f lerp(const quat4f& a, const quat4f& b, float t);
inline quat4f nlerp(const quat4f& a, const quat4f& b, float t);
inline quat4f slerp(const quat4f& a, const quat4f& b, float t);

}  // namespace yocto::math

// -----------------------------------------------------------------------------
// AXIS ALIGNED BOUNDING BOXES
// -----------------------------------------------------------------------------
namespace yocto::math {

// Axis aligned bounding box represented as a min/max std::vector pairs.
struct bbox2f {
  vec2f min = {flt_max, flt_max};
  vec2f max = {flt_min, flt_min};

  bbox2f();
  bbox2f(const vec2f& min, const vec2f& max);

  vec2f&       operator[](int i);
  const vec2f& operator[](int i) const;
};

// Axis aligned bounding box represented as a min/max std::vector pairs.
struct bbox3f {
  vec3f min = {flt_max, flt_max, flt_max};
  vec3f max = {flt_min, flt_min, flt_min};

  bbox3f();
  bbox3f(const vec3f& min, const vec3f& max);

  vec3f&       operator[](int i);
  const vec3f& operator[](int i) const;
};

// Empty bbox constant.
inline const auto invalidb2f = bbox2f{};
inline const auto invalidb3f = bbox3f{};

// Bounding box properties
inline vec2f center(const bbox2f& a);
inline vec2f size(const bbox2f& a);

// Bounding box comparisons.
inline bool operator==(const bbox2f& a, const bbox2f& b);
inline bool operator!=(const bbox2f& a, const bbox2f& b);

// Bounding box expansions with points and other boxes.
inline bbox2f merge(const bbox2f& a, const vec2f& b);
inline bbox2f merge(const bbox2f& a, const bbox2f& b);
inline void   expand(bbox2f& a, const vec2f& b);
inline void   expand(bbox2f& a, const bbox2f& b);

// Bounding box properties
inline vec3f center(const bbox3f& a);
inline vec3f size(const bbox3f& a);

// Bounding box comparisons.
inline bool operator==(const bbox3f& a, const bbox3f& b);
inline bool operator!=(const bbox3f& a, const bbox3f& b);

// Bounding box expansions with points and other boxes.
inline bbox3f merge(const bbox3f& a, const vec3f& b);
inline bbox3f merge(const bbox3f& a, const bbox3f& b);
inline void   expand(bbox3f& a, const vec3f& b);
inline void   expand(bbox3f& a, const bbox3f& b);

// Primitive bounds.
inline bbox3f point_bounds(const vec3f& p);
inline bbox3f point_bounds(const vec3f& p, float r);
inline bbox3f line_bounds(const vec3f& p0, const vec3f& p1);
inline bbox3f line_bounds(const vec3f& p0, const vec3f& p1, float r0, float r1);
inline bbox3f triangle_bounds(
    const vec3f& p0, const vec3f& p1, const vec3f& p2);
inline bbox3f quad_bounds(
    const vec3f& p0, const vec3f& p1, const vec3f& p2, const vec3f& p3);

}  // namespace yocto::math

// -----------------------------------------------------------------------------
// RAYS
// -----------------------------------------------------------------------------
namespace yocto::math {

// Ray esplison
inline const auto ray_eps = 1e-4f;

struct ray2f {
  vec2f o    = {0, 0};
  vec2f d    = {0, 1};
  float tmin = ray_eps;
  float tmax = flt_max;

  ray2f();
  ray2f(const vec2f& o, const vec2f& d, float tmin = ray_eps,
      float tmax = flt_max);
};

// Rays with origin, direction and min/max t value.
struct ray3f {
  vec3f o    = {0, 0, 0};
  vec3f d    = {0, 0, 1};
  float tmin = ray_eps;
  float tmax = flt_max;

  ray3f();
  ray3f(const vec3f& o, const vec3f& d, float tmin = ray_eps,
      float tmax = flt_max);
};

}  // namespace yocto::math

// -----------------------------------------------------------------------------
// TRANSFORMS
// -----------------------------------------------------------------------------
namespace yocto::math {

// Transforms points, vectors and directions by matrices.
inline vec2f transform_point(const mat3f& a, const vec2f& b);
inline vec2f transform_vector(const mat3f& a, const vec2f& b);
inline vec2f transform_direction(const mat3f& a, const vec2f& b);
inline vec2f transform_normal(const mat3f& a, const vec2f& b);
inline vec2f transform_vector(const mat2f& a, const vec2f& b);
inline vec2f transform_direction(const mat2f& a, const vec2f& b);
inline vec2f transform_normal(const mat2f& a, const vec2f& b);

inline vec3f transform_point(const mat4f& a, const vec3f& b);
inline vec3f transform_vector(const mat4f& a, const vec3f& b);
inline vec3f transform_direction(const mat4f& a, const vec3f& b);
inline vec3f transform_vector(const mat3f& a, const vec3f& b);
inline vec3f transform_direction(const mat3f& a, const vec3f& b);
inline vec3f transform_normal(const mat3f& a, const vec3f& b);

// Transforms points, vectors and directions by frames.
inline vec2f transform_point(const frame2f& a, const vec2f& b);
inline vec2f transform_vector(const frame2f& a, const vec2f& b);
inline vec2f transform_direction(const frame2f& a, const vec2f& b);
inline vec2f transform_normal(
    const frame2f& a, const vec2f& b, bool non_rigid = false);

// Transforms points, vectors and directions by frames.
inline vec3f transform_point(const frame3f& a, const vec3f& b);
inline vec3f transform_vector(const frame3f& a, const vec3f& b);
inline vec3f transform_direction(const frame3f& a, const vec3f& b);
inline vec3f transform_normal(
    const frame3f& a, const vec3f& b, bool non_rigid = false);

// Transforms rays and bounding boxes by matrices.
inline ray3f  transform_ray(const mat4f& a, const ray3f& b);
inline ray3f  transform_ray(const frame3f& a, const ray3f& b);
inline bbox3f transform_bbox(const mat4f& a, const bbox3f& b);
inline bbox3f transform_bbox(const frame3f& a, const bbox3f& b);

// Translation, scaling and rotations transforms.
inline frame3f translation_frame(const vec3f& a);
inline frame3f scaling_frame(const vec3f& a);
inline frame3f rotation_frame(const vec3f& axis, float angle);
inline frame3f rotation_frame(const vec4f& quat);
inline frame3f rotation_frame(const quat4f& quat);
inline frame3f rotation_frame(const mat3f& rot);

// Lookat frame. Z-axis can be inverted with inv_xz.
inline frame3f lookat_frame(const vec3f& eye, const vec3f& center,
    const vec3f& up, bool inv_xz = false);

// OpenGL frustum, ortho and perspecgive matrices.
inline mat4f frustum_mat(float l, float r, float b, float t, float n, float f);
inline mat4f ortho_mat(float l, float r, float b, float t, float n, float f);
inline mat4f ortho2d_mat(float left, float right, float bottom, float top);
inline mat4f ortho_mat(float xmag, float ymag, float near, float far);
inline mat4f perspective_mat(float fovy, float aspect, float near, float far);
inline mat4f perspective_mat(float fovy, float aspect, float near);

// Rotation conversions.
inline std::pair<vec3f, float> rotation_axisangle(const vec4f& quat);
inline vec4f                   rotation_quat(const vec3f& axis, float angle);
inline vec4f                   rotation_quat(const vec4f& axisangle);

}  // namespace yocto::math

// -----------------------------------------------------------------------------
// GEOMETRY UTILITIES
// -----------------------------------------------------------------------------
namespace yocto::math {

// Line properties.
inline vec3f line_tangent(const vec3f& p0, const vec3f& p1);
inline float line_length(const vec3f& p0, const vec3f& p1);

// Triangle properties.
inline vec3f triangle_normal(const vec3f& p0, const vec3f& p1, const vec3f& p2);
inline float triangle_area(const vec3f& p0, const vec3f& p1, const vec3f& p2);

// Quad propeties.
inline vec3f quad_normal(
    const vec3f& p0, const vec3f& p1, const vec3f& p2, const vec3f& p3);
inline float quad_area(
    const vec3f& p0, const vec3f& p1, const vec3f& p2, const vec3f& p3);

// Triangle tangent and bitangent from uv
inline std::pair<vec3f, vec3f> triangle_tangents_fromuv(const vec3f& p0,
    const vec3f& p1, const vec3f& p2, const vec2f& uv0, const vec2f& uv1,
    const vec2f& uv2);

// Quad tangent and bitangent from uv. Note that we pass a current_uv since
// internally we may want to split the quad in two and we need to known where
// to do it. If not interested in the split, just pass zero2f here.
inline std::pair<vec3f, vec3f> quad_tangents_fromuv(const vec3f& p0,
    const vec3f& p1, const vec3f& p2, const vec3f& p3, const vec2f& uv0,
    const vec2f& uv1, const vec2f& uv2, const vec2f& uv3,
    const vec2f& current_uv);

// Interpolates values over a line parameterized from a to b by u. Same as lerp.
template <typename T>
inline T interpolate_line(const T& p0, const T& p1, float u);

// Interpolates values over a triangle parameterized by u and v along the
// (p1-p0) and (p2-p0) directions. Same as barycentric interpolation.
template <typename T>
inline T interpolate_triangle(
    const T& p0, const T& p1, const T& p2, const vec2f& uv);

// Interpolates values over a quad parameterized by u and v along the
// (p1-p0) and (p2-p1) directions. Same as bilinear interpolation.
template <typename T>
inline T interpolate_quad(
    const T& p0, const T& p1, const T& p2, const T& p3, const vec2f& uv);

// Interpolates values along a cubic Bezier segment parametrized by u.
template <typename T>
inline T interpolate_bezier(
    const T& p0, const T& p1, const T& p2, const T& p3, float u);
// Computes the derivative of a cubic Bezier segment parametrized by u.

template <typename T>
inline T interpolate_bezier_derivative(
    const T& p0, const T& p1, const T& p2, const T& p3, float u);

}  // namespace yocto::math

// -----------------------------------------------------------------------------
// RAY-PRIMITIVE INTERSECTION FUNCTIONS
// -----------------------------------------------------------------------------
namespace yocto::math {

// Intersect a ray with a point (approximate)
inline bool intersect_point(
    const ray3f& ray, const vec3f& p, float r, vec2f& uv, float& dist);

// Intersect a ray with a line
inline bool intersect_line(const ray3f& ray, const vec3f& p0, const vec3f& p1,
    float r0, float r1, vec2f& uv, float& dist);

// Intersect a ray with a triangle
inline bool intersect_triangle(const ray3f& ray, const vec3f& p0,
    const vec3f& p1, const vec3f& p2, vec2f& uv, float& dist);

// Intersect a ray with a quad.
inline bool intersect_quad(const ray3f& ray, const vec3f& p0, const vec3f& p1,
    const vec3f& p2, const vec3f& p3, vec2f& uv, float& dist);

// Intersect a ray with a axis-aligned bounding box
inline bool intersect_bbox(const ray3f& ray, const bbox3f& bbox);

// Intersect a ray with a axis-aligned bounding box
inline bool intersect_bbox(
    const ray3f& ray, const vec3f& ray_dinv, const bbox3f& bbox);

}  // namespace yocto::math

// -----------------------------------------------------------------------------
// POINT-PRIMITIVE DISTANCE FUNCTIONS
// -----------------------------------------------------------------------------
namespace yocto::math {

// Check if a point overlaps a position pos withint a maximum distance dist_max.
inline bool overlap_point(const vec3f& pos, float dist_max, const vec3f& p,
    float r, vec2f& uv, float& dist);

// Compute the closest line uv to a give position pos.
inline float closestuv_line(const vec3f& pos, const vec3f& p0, const vec3f& p1);

// Check if a line overlaps a position pos withint a maximum distance dist_max.
inline bool overlap_line(const vec3f& pos, float dist_max, const vec3f& p0,
    const vec3f& p1, float r0, float r1, vec2f& uv, float& dist);

// Compute the closest triangle uv to a give position pos.
inline vec2f closestuv_triangle(
    const vec3f& pos, const vec3f& p0, const vec3f& p1, const vec3f& p2);

// Check if a triangle overlaps a position pos withint a maximum distance
// dist_max.
inline bool overlap_triangle(const vec3f& pos, float dist_max, const vec3f& p0,
    const vec3f& p1, const vec3f& p2, float r0, float r1, float r2, vec2f& uv,
    float& dist);

// Check if a quad overlaps a position pos withint a maximum distance dist_max.
inline bool overlap_quad(const vec3f& pos, float dist_max, const vec3f& p0,
    const vec3f& p1, const vec3f& p2, const vec3f& p3, float r0, float r1,
    float r2, float r3, vec2f& uv, float& dist);

// Check if a bbox overlaps a position pos withint a maximum distance dist_max.
inline bool distance_check_bbox(
    const vec3f& pos, float dist_max, const bbox3f& bbox);

// Check if two bboxe overlap.
inline bool overlap_bbox(const bbox3f& bbox1, const bbox3f& bbox2);

}  // namespace yocto::math

// -----------------------------------------------------------------------------
// COLOR OPERATIONS
// -----------------------------------------------------------------------------
namespace yocto::math {

// Conversion between flots and bytes
inline vec3b  float_to_byte(const vec3f& a);
inline vec3f  byte_to_float(const vec3b& a);
inline vec4b  float_to_byte(const vec4f& a);
inline vec4f  byte_to_float(const vec4b& a);
inline byte   float_to_byte(float a);
inline float  byte_to_float(byte a);
inline ushort float_to_ushort(float a);
inline float  ushort_to_float(ushort a);

// Luminance
inline float luminance(const vec3f& a);

// sRGB non-linear curve
inline float srgb_to_rgb(float srgb);
inline float rgb_to_srgb(float rgb);
inline vec3f srgb_to_rgb(const vec3f& srgb);
inline vec4f srgb_to_rgb(const vec4f& srgb);
inline vec3f rgb_to_srgb(const vec3f& rgb);
inline vec4f rgb_to_srgb(const vec4f& rgb);

// Apply contrast. Grey should be 0.18 for linear and 0.5 for gamma.
inline vec3f lincontrast(const vec3f& rgb, float contrast, float grey);
// Apply contrast in log2. Grey should be 0.18 for linear and 0.5 for gamma.
inline vec3f logcontrast(const vec3f& rgb, float logcontrast, float grey);
// Apply an s-shaped contrast.
inline vec3f contrast(const vec3f& rgb, float contrast);
// Apply saturation.
inline vec3f saturate(const vec3f& rgb, float saturation,
    const vec3f& weights = vec3f{0.333333f});

// Apply tone mapping
inline vec3f tonemap(
    const vec3f& hdr, float exposure, bool filmic = false, bool srgb = true);
inline vec4f tonemap(
    const vec4f& hdr, float exposure, bool filmic = false, bool srgb = true);

// Convert between CIE XYZ and RGB
inline vec3f rgb_to_xyz(const vec3f& rgb);
inline vec3f xyz_to_rgb(const vec3f& xyz);

// Convert between CIE XYZ and xyY
inline vec3f xyz_to_xyY(const vec3f& xyz);
inline vec3f xyY_to_xyz(const vec3f& xyY);

// Converts between HSV and RGB color spaces.
inline vec3f hsv_to_rgb(const vec3f& hsv);
inline vec3f rgb_to_hsv(const vec3f& rgb);

// Approximate color of blackbody radiation from wavelength in nm.
inline vec3f blackbody_to_rgb(float temperature);

}  // namespace yocto::math

// -----------------------------------------------------------------------------
// RANDOM NUMBER GENERATION
// -----------------------------------------------------------------------------
namespace yocto::math {

// PCG random numbers from http://www.pcg-random.org/
struct rng_state {
  uint64_t state = 0x853c49e6748fea9bULL;
  uint64_t inc   = 0xda3e39cb94b95bdbULL;

  rng_state() : state{0x853c49e6748fea9bULL}, inc{0xda3e39cb94b95bdbULL} {}
  rng_state(uint64_t state, uint64_t inc) : state{state}, inc{inc} {}
};

// Next random number, used internally only.
inline uint32_t _advance_rng(rng_state& rng) {
  uint64_t oldstate   = rng.state;
  rng.state           = oldstate * 6364136223846793005ULL + rng.inc;
  uint32_t xorshifted = (uint32_t)(((oldstate >> 18u) ^ oldstate) >> 27u);
  uint32_t rot        = (uint32_t)(oldstate >> 59u);
  return (xorshifted >> rot) | (xorshifted << ((-rot) & 31));
}

// Init a random number generator with a state state from the sequence seq.
inline rng_state make_rng(uint64_t seed, uint64_t seq = 1) {
  auto rng  = rng_state();
  rng.state = 0U;
  rng.inc   = (seq << 1u) | 1u;
  _advance_rng(rng);
  rng.state += seed;
  _advance_rng(rng);
  return rng;
}

// Next random numbers: floats in [0,1), ints in [0,n).
inline int   rand1i(rng_state& rng, int n) { return _advance_rng(rng) % n; }
inline float rand1f(rng_state& rng) {
  union {
    uint32_t u;
    float    f;
  } x;
  x.u = (_advance_rng(rng) >> 9) | 0x3f800000u;
  return x.f - 1.0f;
  // alternate implementation
  // const static auto scale = (float)(1.0 / numeric_limits<uint32_t>::max());
  // return advance_rng(rng) * scale;
}
inline vec2f rand2f(rng_state& rng) {
  // force order of evaluation by using separate assignments.
  auto x = rand1f(rng);
  auto y = rand1f(rng);
  return {x, y};
}
inline vec3f rand3f(rng_state& rng) {
  // force order of evaluation by using separate assignments.
  auto x = rand1f(rng);
  auto y = rand1f(rng);
  auto z = rand1f(rng);
  return {x, y, z};
}

// Shuffles a sequence of elements
template <typename T>
inline void shuffle(std::vector<T>& vals, rng_state& rng) {
  // https://en.wikipedia.org/wiki/Fisher–Yates_shuffle
  for (auto i = (int)vals.size() - 1; i > 0; i--) {
    auto j = rand1i(rng, i + 1);
    std::swap(vals[j], vals[i]);
  }
}

}  // namespace yocto::math

// -----------------------------------------------------------------------------
// PERLIN NOISE FUNCTION
// -----------------------------------------------------------------------------
namespace yocto::math {

// Compute the revised Perlin noise function. Wrap provides a wrapping noise
// but must be power of two (wraps at 256 anyway). For octave based noise,
// good values are obtained with octaves=6 (numerber of noise calls),
// lacunarity=~2.0 (spacing between successive octaves: 2.0 for warpping
// output), gain=0.5 (relative weighting applied to each successive octave),
// offset=1.0 (used to invert the ridges).
inline float perlin_noise(const vec3f& p, const vec3i& wrap = zero3i);
inline float perlin_ridge(const vec3f& p, float lacunarity = 2,
    float gain = 0.5, int octaves = 6, float offset = 1,
    const vec3i& wrap = zero3i);
inline float perlin_fbm(const vec3f& p, float lacunarity = 2, float gain = 0.5,
    int octaves = 6, const vec3i& wrap = zero3i);
inline float perlin_turbulence(const vec3f& p, float lacunarity = 2,
    float gain = 0.5, int octaves = 6, const vec3i& wrap = zero3i);

}  // namespace yocto::math

// -----------------------------------------------------------------------------
// SHADING FUNCTIONS
// -----------------------------------------------------------------------------
namespace yocto::math {

// Schlick approximation of the Fresnel term.
inline vec3f fresnel_schlick(
    const vec3f& specular, const vec3f& normal, const vec3f& outgoing);
// Compute the fresnel term for dielectrics.
inline float fresnel_dielectric(
    float eta, const vec3f& normal, const vec3f& outgoing);
// Compute the fresnel term for metals.
inline vec3f fresnel_conductor(const vec3f& eta, const vec3f& etak,
    const vec3f& normal, const vec3f& outgoing);

// Convert eta to reflectivity
inline vec3f eta_to_reflectivity(const vec3f& eta);
// Convert reflectivity to  eta.
inline vec3f reflectivity_to_eta(const vec3f& reflectivity);
// Convert conductor eta to reflectivity.
inline vec3f eta_to_reflectivity(const vec3f& eta, const vec3f& etak);
// Convert eta to edge tint parametrization.
inline std::pair<vec3f, vec3f> eta_to_edgetint(
    const vec3f& eta, const vec3f& etak);
// Convert reflectivity and edge tint to eta.
inline std::pair<vec3f, vec3f> edgetint_to_eta(
    const vec3f& reflectivity, const vec3f& edgetint);

// Evaluates the microfacet distribution.
inline float microfacet_distribution(float roughness, const vec3f& normal,
    const vec3f& halfway, bool ggx = true);
// Evaluates the microfacet shadowing.
inline float microfacet_shadowing(float roughness, const vec3f& normal,
    const vec3f& halfway, const vec3f& outgoing, const vec3f& incoming,
    bool ggx = true);

// Samples a microfacet distribution.
inline vec3f sample_microfacet(
    float roughness, const vec3f& normal, const vec2f& rn, bool ggx = true);
// Pdf for microfacet distribution sampling.
inline float sample_microfacet_pdf(float roughness, const vec3f& normal,
    const vec3f& halfway, bool ggx = true);

// Samples a microfacet distribution with the distribution of visible normals.
inline vec3f sample_microfacet(float roughness, const vec3f& normal,
    const vec3f& outgoing, const vec2f& rn, bool ggx = true);
// Pdf for microfacet distribution sampling with the distribution of visible
// normals.
inline float sample_microfacet_pdf(float roughness, const vec3f& normal,
    const vec3f& halfway, const vec3f& outgoing, bool ggx = true);

// Evaluates a diffuse BRDF lobe.
inline vec3f eval_diffuse_reflection(
    const vec3f& normal, const vec3f& outgoing, const vec3f& incoming);
// Evaluates a specular BRDF lobe.
inline vec3f eval_microfacet_reflection(float ior, float roughness,
    const vec3f& normal, const vec3f& outgoing, const vec3f& incoming);
// Evaluates a metal BRDF lobe.
inline vec3f eval_microfacet_reflection(const vec3f& eta, const vec3f& etak,
    float roughness, const vec3f& normal, const vec3f& outgoing,
    const vec3f& incoming);
// Evaluates a transmission BRDF lobe.
inline vec3f eval_microfacet_transmission(float ior, float roughness,
    const vec3f& normal, const vec3f& outgoing, const vec3f& incoming);
// Evaluates a refraction BRDF lobe.
inline vec3f eval_microfacet_refraction(float ior, float roughness,
    const vec3f& normal, const vec3f& outgoing, const vec3f& incoming);

// Sample a diffuse BRDF lobe.
inline vec3f sample_diffuse_reflection(
    const vec3f& normal, const vec3f& outgoing, const vec2f& rn);
// Sample a specular BRDF lobe.
inline vec3f sample_microfacet_reflection(float ior, float roughness,
    const vec3f& normal, const vec3f& outgoing, const vec2f& rn);
// Sample a metal BRDF lobe.
inline vec3f sample_microfacet_reflection(const vec3f& eta, const vec3f& etak,
    float roughness, const vec3f& normal, const vec3f& outgoing,
    const vec2f& rn);
// Sample a transmission BRDF lobe.
inline vec3f sample_microfacet_transmission(float ior, float roughness,
    const vec3f& normal, const vec3f& outgoing, const vec2f& rn);
// Sample a refraction BRDF lobe.
inline vec3f sample_microfacet_refraction(float ior, float roughness,
    const vec3f& normal, const vec3f& outgoing, float rnl, const vec2f& rn);

// Pdf for diffuse BRDF lobe sampling.
inline float sample_diffuse_reflection_pdf(
    const vec3f& normal, const vec3f& outgoing, const vec3f& incoming);
// Pdf for specular BRDF lobe sampling.
inline float sample_microfacet_reflection_pdf(float ior, float roughness,
    const vec3f& normal, const vec3f& outgoing, const vec3f& incoming);
// Pdf for metal BRDF lobe sampling.
inline float sample_microfacet_reflection_pdf(const vec3f& eta,
    const vec3f& etak, float roughness, const vec3f& normal,
    const vec3f& outgoing, const vec3f& incoming);
// Pdf for transmission BRDF lobe sampling.
inline float sample_microfacet_transmission_pdf(float ior, float roughness,
    const vec3f& normal, const vec3f& outgoing, const vec3f& incoming);
// Pdf for refraction BRDF lobe sampling.
inline float sample_microfacet_refraction_pdf(float ior, float roughness,
    const vec3f& normal, const vec3f& outgoing, const vec3f& incoming);

// Evaluate a delta specular BRDF lobe.
inline vec3f eval_delta_reflection(float ior, const vec3f& normal,
    const vec3f& outgoing, const vec3f& incoming);
// Evaluate a delta metal BRDF lobe.
inline vec3f eval_delta_reflection(const vec3f& eta, const vec3f& etak,
    const vec3f& normal, const vec3f& outgoing, const vec3f& incoming);
// Evaluate a delta transmission BRDF lobe.
inline vec3f eval_delta_transmission(float ior, const vec3f& normal,
    const vec3f& outgoing, const vec3f& incoming);
// Evaluate a delta refraction BRDF lobe.
inline vec3f eval_delta_refraction(float ior, const vec3f& normal,
    const vec3f& outgoing, const vec3f& incoming);

// Sample a delta specular BRDF lobe.
inline vec3f sample_delta_reflection(
    float ior, const vec3f& normal, const vec3f& outgoing);
// Sample a delta metal BRDF lobe.
inline vec3f sample_delta_reflection(const vec3f& eta, const vec3f& etak,
    const vec3f& normal, const vec3f& outgoing);
// Sample a delta transmission BRDF lobe.
inline vec3f sample_delta_transmission(
    float ior, const vec3f& normal, const vec3f& outgoing);
// Sample a delta refraction BRDF lobe.
inline vec3f sample_delta_refraction(
    float ior, const vec3f& normal, const vec3f& outgoing, float rnl);

// Pdf for delta specular BRDF lobe sampling.
inline float sample_delta_reflection_pdf(float ior, const vec3f& normal,
    const vec3f& outgoing, const vec3f& incoming);
// Pdf for delta metal BRDF lobe sampling.
inline float sample_delta_reflection_pdf(const vec3f& eta, const vec3f& etak,
    const vec3f& normal, const vec3f& outgoing, const vec3f& incoming);
// Pdf for delta transmission BRDF lobe sampling.
inline float sample_delta_transmission_pdf(float ior, const vec3f& normal,
    const vec3f& outgoing, const vec3f& incoming);
// Pdf for delta refraction BRDF lobe sampling.
inline float sample_delta_refraction_pdf(float ior, const vec3f& normal,
    const vec3f& outgoing, const vec3f& incoming);

// Evaluate transmittance
inline vec3f eval_transmittance(const vec3f& density, float distance);
// Sample a distance proportionally to transmittance
inline float sample_transmittance(
    const vec3f& density, float max_distance, float rl, float rd);
// Pdf for distance sampling
inline float sample_transmittance_pdf(
    const vec3f& density, float distance, float max_distance);

// Evaluate phase function
inline float eval_phasefunction(
    float anisotropy, const vec3f& outgoing, const vec3f& incoming);
// Sample phase function
inline vec3f sample_phasefunction(
    float anisotropy, const vec3f& outgoing, const vec2f& rn);
// Pdf for phase function sampling
inline float sample_phasefunction_pdf(
    float anisotropy, const vec3f& outgoing, const vec3f& incoming);

}  // namespace yocto::math

// -----------------------------------------------------------------------------
// MONETACARLO SAMPLING FUNCTIONS
// -----------------------------------------------------------------------------
namespace yocto::math {

// Sample an hemispherical direction with uniform distribution.
inline vec3f sample_hemisphere(const vec2f& ruv);
inline float sample_hemisphere_pdf(const vec3f& direction);

// Sample an hemispherical direction with uniform distribution.
inline vec3f sample_hemisphere(const vec3f& normal, const vec2f& ruv);
inline float sample_hemisphere_pdf(const vec3f& normal, const vec3f& direction);

// Sample a spherical direction with uniform distribution.
inline vec3f sample_sphere(const vec2f& ruv);
inline float sample_sphere_pdf(const vec3f& w);

// Sample an hemispherical direction with cosine distribution.
inline vec3f sample_hemisphere_cos(const vec2f& ruv);
inline float sample_hemisphere_cos_pdf(const vec3f& direction);

// Sample an hemispherical direction with cosine distribution.
inline vec3f sample_hemisphere_cos(const vec3f& normal, const vec2f& ruv);
inline float sample_hemisphere_cos_pdf(
    const vec3f& normal, const vec3f& direction);

// Sample an hemispherical direction with cosine power distribution.
inline vec3f sample_hemisphere_cospower(float exponent, const vec2f& ruv);
inline float sample_hemisphere_cospower_pdf(
    float exponent, const vec3f& direction);

// Sample a point uniformly on a disk.
inline vec2f sample_disk(const vec2f& ruv);
inline float sample_disk_pdf(const vec2f& point);

// Sample a point uniformly on a cylinder, without caps.
inline vec3f sample_cylinder(const vec2f& ruv);
inline float sample_cylinder_pdf(const vec3f& point);

// Sample a point uniformly on a triangle returning the baricentric coordinates.
inline vec2f sample_triangle(const vec2f& ruv);

// Sample a point uniformly on a triangle.
inline vec3f sample_triangle(
    const vec3f& p0, const vec3f& p1, const vec3f& p2, const vec2f& ruv);
// Pdf for uniform triangle sampling, i.e. triangle area.
inline float sample_triangle_pdf(
    const vec3f& p0, const vec3f& p1, const vec3f& p2);

// Sample an index with uniform distribution.
inline int   sample_uniform(int size, float r);
inline float sample_uniform_pdf(int size);

// Sample an index with uniform distribution.
inline float sample_uniform(const std::vector<float>& elements, float r);
inline float sample_uniform_pdf(const std::vector<float>& elements);

// Sample a discrete distribution represented by its cdf.
[[deprecated]] inline int sample_discrete(
    const std::vector<float>& cdf, float r);
// Pdf for uniform discrete distribution sampling.
[[deprecated]] inline float sample_discrete_pdf(
    const std::vector<float>& cdf, int idx);

// Sample a discrete distribution represented by its cdf.
inline int sample_discrete_cdf(const std::vector<float>& cdf, float r);
// Pdf for uniform discrete distribution sampling.
inline float sample_discrete_cdf_pdf(const std::vector<float>& cdf, int idx);

// Sample a discrete distribution represented by its weights.
inline int sample_discrete_weights(const std::vector<float>& weights, float r);
// Pdf for uniform discrete distribution sampling.
inline float sample_discrete_weights_pdf(
    const std::vector<float>& weights, int idx);

// Sample a discrete distribution represented by its weights.
template <size_t N>
inline int sample_discrete_weights(
    const std::array<float, N>& weights, float r);
// Pdf for uniform discrete distribution sampling.
template <size_t N>
inline float sample_discrete_weights_pdf(
    const std::array<float, N>& weights, int idx);

}  // namespace yocto::math

// -----------------------------------------------------------------------------
// USER INTERFACE UTILITIES
// -----------------------------------------------------------------------------
namespace yocto::math {

// Computes the image uv coordinates corresponding to the view parameters.
// Returns negative coordinates if out of the image.
inline vec2i get_image_coords(const vec2f& mouse_pos, const vec2f& center,
    float scale, const vec2i& txt_size);

// Center image and autofit.
inline void update_imview(vec2f& center, float& scale, const vec2i& imsize,
    const vec2i& winsize, bool zoom_to_fit);

// Turntable for UI navigation.
inline void update_turntable(vec3f& from, vec3f& to, vec3f& up,
    const vec2f& rotate, float dolly, const vec2f& pan);

// Turntable for UI navigation.
inline void update_turntable(frame3f& frame, float& focus, const vec2f& rotate,
    float dolly, const vec2f& pan);

// FPS camera for UI navigation for a frame parametrization.
inline void update_fpscam(
    frame3f& frame, const vec3f& transl, const vec2f& rotate);

// Generate a ray from a camera
inline ray3f camera_ray(
    const frame3f& frame, float lens, const vec2f& film, const vec2f& image_uv);

}  // namespace yocto::math

// -----------------------------------------------------------------------------
//
//
// IMPLEMENTATION
//
//
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// IMPLEMENTATION OF MATH CONSTANTS AND FUNCTIONS
// -----------------------------------------------------------------------------
namespace yocto::math {

inline float abs(float a) { return a < 0 ? -a : a; }
inline float min(float a, float b) { return (a < b) ? a : b; }
inline float max(float a, float b) { return (a > b) ? a : b; }
inline float clamp(float a, float min_, float max_) {
  return min(max(a, min_), max_);
}
inline float sign(float a) { return a < 0 ? -1 : 1; }
inline float sqrt(float a) { return std::sqrt(a); }
inline float sin(float a) { return std::sin(a); }
inline float cos(float a) { return std::cos(a); }
inline float tan(float a) { return std::tan(a); }
inline float asin(float a) { return std::asin(a); }
inline float acos(float a) { return std::acos(a); }
inline float atan(float a) { return std::atan(a); }
inline float log(float a) { return std::log(a); }
inline float exp(float a) { return std::exp(a); }
inline float log2(float a) { return std::log2(a); }
inline float exp2(float a) { return std::exp2(a); }
inline float pow(float a, float b) { return std::pow(a, b); }
inline float isfinite(float a) { return std::isfinite(a); }
inline float atan2(float a, float b) { return std::atan2(a, b); }
inline float fmod(float a, float b) { return std::fmod(a, b); }
inline void  swap(float& a, float& b) { std::swap(a, b); }
inline float radians(float a) { return a * pif / 180; }
inline float degrees(float a) { return a * 180 / pif; }
inline float lerp(float a, float b, float u) { return a * (1 - u) + b * u; }
inline float step(float a, float u) { return u < a ? 0 : 1; }
inline float smoothstep(float a, float b, float u) {
  auto t = clamp((u - a) / (b - a), 0.0f, 1.0f);
  return t * t * (3 - 2 * t);
}
inline float bias(float a, float bias) {
  return a / ((1 / bias - 2) * (1 - a) + 1);
}
inline float gain(float a, float gain) {
  return (a < 0.5f) ? bias(a * 2, gain) / 2
                    : bias(a * 2 - 1, 1 - gain) / 2 + 0.5f;
}

inline int  abs(int a) { return a < 0 ? -a : a; }
inline int  min(int a, int b) { return (a < b) ? a : b; }
inline int  max(int a, int b) { return (a > b) ? a : b; }
inline int  clamp(int a, int min_, int max_) { return min(max(a, min_), max_); }
inline int  sign(int a) { return a < 0 ? -1 : 1; }
inline int  pow2(int a) { return 1 << a; }
inline void swap(int& a, int& b) { std::swap(a, b); }

}  // namespace yocto::math

// -----------------------------------------------------------------------------
// VECTORS
// -----------------------------------------------------------------------------
namespace yocto::math {

// Vec2
inline vec2f::vec2f() {}
inline vec2f::vec2f(float x, float y) : x{x}, y{y} {}
inline vec2f::vec2f(float v) : x{v}, y{v} {}
inline vec2f::operator bool() const { return x || y; }

inline float& vec2f::operator[](int i) { return (&x)[i]; }
inline const float& vec2f::operator[](int i) const { return (&x)[i]; }

// Vec3
inline vec3f::vec3f() {}
inline vec3f::vec3f(float x, float y, float z) : x{x}, y{y}, z{z} {}
inline vec3f::vec3f(const vec2f& v, float z) : x{v.x}, y{v.y}, z{z} {}
inline vec3f::vec3f(float v) : x{v}, y{v}, z{v} {}
inline vec3f::operator bool() const { return x || y || z; }

inline float& vec3f::operator[](int i) { return (&x)[i]; }
inline const float& vec3f::operator[](int i) const { return (&x)[i]; }

// Vec4
inline vec4f::vec4f() {}
inline vec4f::vec4f(float x, float y, float z, float w)
    : x{x}, y{y}, z{z}, w{w} {}
inline vec4f::vec4f(const vec3f& v, float w) : x{v.x}, y{v.y}, z{v.z}, w{w} {}
inline vec4f::vec4f(float v) : x{v}, y{v}, z{v}, w{v} {}
inline vec4f::operator bool() const { return x || y || z || w; }

inline float& vec4f::operator[](int i) { return (&x)[i]; }
inline const float& vec4f::operator[](int i) const { return (&x)[i]; }

// Element access
inline vec3f&       xyz(vec4f& a) { return (vec3f&)a; }
inline const vec3f& xyz(const vec4f& a) { return (const vec3f&)a; }

// Vector comparison operations.
inline bool operator==(const vec2f& a, const vec2f& b) {
  return a.x == b.x && a.y == b.y;
}
inline bool operator!=(const vec2f& a, const vec2f& b) {
  return a.x != b.x || a.y != b.y;
}

// Vector operations.
inline vec2f operator+(const vec2f& a) { return a; }
inline vec2f operator-(const vec2f& a) { return {-a.x, -a.y}; }
inline vec2f operator+(const vec2f& a, const vec2f& b) {
  return {a.x + b.x, a.y + b.y};
}
inline vec2f operator+(const vec2f& a, float b) { return {a.x + b, a.y + b}; }
inline vec2f operator+(float a, const vec2f& b) { return {a + b.x, a + b.y}; }
inline vec2f operator-(const vec2f& a, const vec2f& b) {
  return {a.x - b.x, a.y - b.y};
}
inline vec2f operator-(const vec2f& a, float b) { return {a.x - b, a.y - b}; }
inline vec2f operator-(float a, const vec2f& b) { return {a - b.x, a - b.y}; }
inline vec2f operator*(const vec2f& a, const vec2f& b) {
  return {a.x * b.x, a.y * b.y};
}
inline vec2f operator*(const vec2f& a, float b) { return {a.x * b, a.y * b}; }
inline vec2f operator*(float a, const vec2f& b) { return {a * b.x, a * b.y}; }
inline vec2f operator/(const vec2f& a, const vec2f& b) {
  return {a.x / b.x, a.y / b.y};
}
inline vec2f operator/(const vec2f& a, float b) { return {a.x / b, a.y / b}; }
inline vec2f operator/(float a, const vec2f& b) { return {a / b.x, a / b.y}; }

// Vector assignments
inline vec2f& operator+=(vec2f& a, const vec2f& b) { return a = a + b; }
inline vec2f& operator+=(vec2f& a, float b) { return a = a + b; }
inline vec2f& operator-=(vec2f& a, const vec2f& b) { return a = a - b; }
inline vec2f& operator-=(vec2f& a, float b) { return a = a - b; }
inline vec2f& operator*=(vec2f& a, const vec2f& b) { return a = a * b; }
inline vec2f& operator*=(vec2f& a, float b) { return a = a * b; }
inline vec2f& operator/=(vec2f& a, const vec2f& b) { return a = a / b; }
inline vec2f& operator/=(vec2f& a, float b) { return a = a / b; }

// Vector products and lengths.
inline float dot(const vec2f& a, const vec2f& b) {
  return a.x * b.x + a.y * b.y;
}
inline float cross(const vec2f& a, const vec2f& b) {
  return a.x * b.y - a.y * b.x;
}

inline float length(const vec2f& a) { return sqrt(dot(a, a)); }
inline vec2f normalize(const vec2f& a) {
  auto l = length(a);
  return (l != 0) ? a / l : a;
}
inline float distance(const vec2f& a, const vec2f& b) { return length(a - b); }
inline float distance_squared(const vec2f& a, const vec2f& b) {
  return dot(a - b, a - b);
}

// Max element and clamp.
inline vec2f max(const vec2f& a, float b) { return {max(a.x, b), max(a.y, b)}; }
inline vec2f min(const vec2f& a, float b) { return {min(a.x, b), min(a.y, b)}; }
inline vec2f max(const vec2f& a, const vec2f& b) {
  return {max(a.x, b.x), max(a.y, b.y)};
}
inline vec2f min(const vec2f& a, const vec2f& b) {
  return {min(a.x, b.x), min(a.y, b.y)};
}
inline vec2f clamp(const vec2f& x, float min, float max) {
  return {clamp(x.x, min, max), clamp(x.y, min, max)};
}
inline vec2f lerp(const vec2f& a, const vec2f& b, float u) {
  return a * (1 - u) + b * u;
}
inline vec2f lerp(const vec2f& a, const vec2f& b, const vec2f& u) {
  return a * (1 - u) + b * u;
}

inline float max(const vec2f& a) { return max(a.x, a.y); }
inline float min(const vec2f& a) { return min(a.x, a.y); }
inline float sum(const vec2f& a) { return a.x + a.y; }
inline float mean(const vec2f& a) { return sum(a) / 2; }

// Functions applied to std::vector elements
inline vec2f abs(const vec2f& a) { return {abs(a.x), abs(a.y)}; };
inline vec2f sqrt(const vec2f& a) { return {sqrt(a.x), sqrt(a.y)}; };
inline vec2f exp(const vec2f& a) { return {exp(a.x), exp(a.y)}; };
inline vec2f log(const vec2f& a) { return {log(a.x), log(a.y)}; };
inline vec2f exp2(const vec2f& a) { return {exp2(a.x), exp2(a.y)}; };
inline vec2f log2(const vec2f& a) { return {log2(a.x), log2(a.y)}; };
inline bool isfinite(const vec2f& a) { return isfinite(a.x) && isfinite(a.y); };
inline vec2f pow(const vec2f& a, float b) {
  return {pow(a.x, b), pow(a.y, b)};
};
inline vec2f pow(const vec2f& a, const vec2f& b) {
  return {pow(a.x, b.x), pow(a.y, b.y)};
};
inline vec2f gain(const vec2f& a, float b) {
  return {gain(a.x, b), gain(a.y, b)};
};
inline void swap(vec2f& a, vec2f& b) { std::swap(a, b); }

// Vector comparison operations.
inline bool operator==(const vec3f& a, const vec3f& b) {
  return a.x == b.x && a.y == b.y && a.z == b.z;
}
inline bool operator!=(const vec3f& a, const vec3f& b) {
  return a.x != b.x || a.y != b.y || a.z != b.z;
}

// Vector operations.
inline vec3f operator+(const vec3f& a) { return a; }
inline vec3f operator-(const vec3f& a) { return {-a.x, -a.y, -a.z}; }
inline vec3f operator+(const vec3f& a, const vec3f& b) {
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}
inline vec3f operator+(const vec3f& a, float b) {
  return {a.x + b, a.y + b, a.z + b};
}
inline vec3f operator+(float a, const vec3f& b) {
  return {a + b.x, a + b.y, a + b.z};
}
inline vec3f operator-(const vec3f& a, const vec3f& b) {
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}
inline vec3f operator-(const vec3f& a, float b) {
  return {a.x - b, a.y - b, a.z - b};
}
inline vec3f operator-(float a, const vec3f& b) {
  return {a - b.x, a - b.y, a - b.z};
}
inline vec3f operator*(const vec3f& a, const vec3f& b) {
  return {a.x * b.x, a.y * b.y, a.z * b.z};
}
inline vec3f operator*(const vec3f& a, float b) {
  return {a.x * b, a.y * b, a.z * b};
}
inline vec3f operator*(float a, const vec3f& b) {
  return {a * b.x, a * b.y, a * b.z};
}
inline vec3f operator/(const vec3f& a, const vec3f& b) {
  return {a.x / b.x, a.y / b.y, a.z / b.z};
}
inline vec3f operator/(const vec3f& a, float b) {
  return {a.x / b, a.y / b, a.z / b};
}
inline vec3f operator/(float a, const vec3f& b) {
  return {a / b.x, a / b.y, a / b.z};
}

// Vector assignments
inline vec3f& operator+=(vec3f& a, const vec3f& b) { return a = a + b; }
inline vec3f& operator+=(vec3f& a, float b) { return a = a + b; }
inline vec3f& operator-=(vec3f& a, const vec3f& b) { return a = a - b; }
inline vec3f& operator-=(vec3f& a, float b) { return a = a - b; }
inline vec3f& operator*=(vec3f& a, const vec3f& b) { return a = a * b; }
inline vec3f& operator*=(vec3f& a, float b) { return a = a * b; }
inline vec3f& operator/=(vec3f& a, const vec3f& b) { return a = a / b; }
inline vec3f& operator/=(vec3f& a, float b) { return a = a / b; }

// Vector products and lengths.
inline float dot(const vec3f& a, const vec3f& b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
inline vec3f cross(const vec3f& a, const vec3f& b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

inline float length(const vec3f& a) { return sqrt(dot(a, a)); }
inline vec3f normalize(const vec3f& a) {
  auto l = length(a);
  return (l != 0) ? a / l : a;
}
inline float distance(const vec3f& a, const vec3f& b) { return length(a - b); }
inline float distance_squared(const vec3f& a, const vec3f& b) {
  return dot(a - b, a - b);
}

inline float angle(const vec3f& a, const vec3f& b) {
  return acos(clamp(dot(normalize(a), normalize(b)), (float)-1, (float)1));
}

// Orthogonal vectors.
inline vec3f orthogonal(const vec3f& v) {
  // http://lolengine.net/blog/2013/09/21/picking-orthogonal-std::vector-combing-coconuts)
  return abs(v.x) > abs(v.z) ? vec3f{-v.y, v.x, 0} : vec3f{0, -v.z, v.y};
}
inline vec3f orthonormalize(const vec3f& a, const vec3f& b) {
  return normalize(a - b * dot(a, b));
}

// Reflected and refracted vector.
inline vec3f reflect(const vec3f& w, const vec3f& n) {
  return -w + 2 * dot(n, w) * n;
}
inline vec3f refract(const vec3f& w, const vec3f& n, float inv_eta) {
  auto cosine = dot(n, w);
  auto k      = 1 + inv_eta * inv_eta * (cosine * cosine - 1);
  if (k < 0) return {0, 0, 0};  // tir
  return -w * inv_eta + (inv_eta * cosine - sqrt(k)) * n;
}

// Max element and clamp.
inline vec3f max(const vec3f& a, float b) {
  return {max(a.x, b), max(a.y, b), max(a.z, b)};
}
inline vec3f min(const vec3f& a, float b) {
  return {min(a.x, b), min(a.y, b), min(a.z, b)};
}
inline vec3f max(const vec3f& a, const vec3f& b) {
  return {max(a.x, b.x), max(a.y, b.y), max(a.z, b.z)};
}
inline vec3f min(const vec3f& a, const vec3f& b) {
  return {min(a.x, b.x), min(a.y, b.y), min(a.z, b.z)};
}
inline vec3f clamp(const vec3f& x, float min, float max) {
  return {clamp(x.x, min, max), clamp(x.y, min, max), clamp(x.z, min, max)};
}
inline vec3f lerp(const vec3f& a, const vec3f& b, float u) {
  return a * (1 - u) + b * u;
}
inline vec3f lerp(const vec3f& a, const vec3f& b, const vec3f& u) {
  return a * (1 - u) + b * u;
}

inline float max(const vec3f& a) { return max(max(a.x, a.y), a.z); }
inline float min(const vec3f& a) { return min(min(a.x, a.y), a.z); }
inline float sum(const vec3f& a) { return a.x + a.y + a.z; }
inline float mean(const vec3f& a) { return sum(a) / 3; }

// Functions applied to std::vector elements
inline vec3f abs(const vec3f& a) { return {abs(a.x), abs(a.y), abs(a.z)}; };
inline vec3f sqrt(const vec3f& a) { return {sqrt(a.x), sqrt(a.y), sqrt(a.z)}; };
inline vec3f exp(const vec3f& a) { return {exp(a.x), exp(a.y), exp(a.z)}; };
inline vec3f log(const vec3f& a) { return {log(a.x), log(a.y), log(a.z)}; };
inline vec3f exp2(const vec3f& a) { return {exp2(a.x), exp2(a.y), exp2(a.z)}; };
inline vec3f log2(const vec3f& a) { return {log2(a.x), log2(a.y), log2(a.z)}; };
inline vec3f pow(const vec3f& a, float b) {
  return {pow(a.x, b), pow(a.y, b), pow(a.z, b)};
};
inline vec3f pow(const vec3f& a, const vec3f& b) {
  return {pow(a.x, b.x), pow(a.y, b.y), pow(a.z, b.z)};
};
inline vec3f gain(const vec3f& a, float b) {
  return {gain(a.x, b), gain(a.y, b), gain(a.z, b)};
};
inline bool isfinite(const vec3f& a) {
  return isfinite(a.x) && isfinite(a.y) && isfinite(a.z);
};
inline void swap(vec3f& a, vec3f& b) { std::swap(a, b); }

// Vector comparison operations.
inline bool operator==(const vec4f& a, const vec4f& b) {
  return a.x == b.x && a.y == b.y && a.z == b.z && a.w == b.w;
}
inline bool operator!=(const vec4f& a, const vec4f& b) {
  return a.x != b.x || a.y != b.y || a.z != b.z || a.w != b.w;
}

// Vector operations.
inline vec4f operator+(const vec4f& a) { return a; }
inline vec4f operator-(const vec4f& a) { return {-a.x, -a.y, -a.z, -a.w}; }
inline vec4f operator+(const vec4f& a, const vec4f& b) {
  return {a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w};
}
inline vec4f operator+(const vec4f& a, float b) {
  return {a.x + b, a.y + b, a.z + b, a.w + b};
}
inline vec4f operator+(float a, const vec4f& b) {
  return {a + b.x, a + b.y, a + b.z, a + b.w};
}
inline vec4f operator-(const vec4f& a, const vec4f& b) {
  return {a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w};
}
inline vec4f operator-(const vec4f& a, float b) {
  return {a.x - b, a.y - b, a.z - b, a.w - b};
}
inline vec4f operator-(float a, const vec4f& b) {
  return {a - b.x, a - b.y, a - b.z, a - b.w};
}
inline vec4f operator*(const vec4f& a, const vec4f& b) {
  return {a.x * b.x, a.y * b.y, a.z * b.z, a.w * b.w};
}
inline vec4f operator*(const vec4f& a, float b) {
  return {a.x * b, a.y * b, a.z * b, a.w * b};
}
inline vec4f operator*(float a, const vec4f& b) {
  return {a * b.x, a * b.y, a * b.z, a * b.w};
}
inline vec4f operator/(const vec4f& a, const vec4f& b) {
  return {a.x / b.x, a.y / b.y, a.z / b.z, a.w / b.w};
}
inline vec4f operator/(const vec4f& a, float b) {
  return {a.x / b, a.y / b, a.z / b, a.w / b};
}
inline vec4f operator/(float a, const vec4f& b) {
  return {a / b.x, a / b.y, a / b.z, a / b.w};
}

// Vector assignments
inline vec4f& operator+=(vec4f& a, const vec4f& b) { return a = a + b; }
inline vec4f& operator+=(vec4f& a, float b) { return a = a + b; }
inline vec4f& operator-=(vec4f& a, const vec4f& b) { return a = a - b; }
inline vec4f& operator-=(vec4f& a, float b) { return a = a - b; }
inline vec4f& operator*=(vec4f& a, const vec4f& b) { return a = a * b; }
inline vec4f& operator*=(vec4f& a, float b) { return a = a * b; }
inline vec4f& operator/=(vec4f& a, const vec4f& b) { return a = a / b; }
inline vec4f& operator/=(vec4f& a, float b) { return a = a / b; }

// Vector products and lengths.
inline float dot(const vec4f& a, const vec4f& b) {
  return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}
inline float length(const vec4f& a) { return sqrt(dot(a, a)); }
inline vec4f normalize(const vec4f& a) {
  auto l = length(a);
  return (l != 0) ? a / l : a;
}
inline float distance(const vec4f& a, const vec4f& b) { return length(a - b); }
inline float distance_squared(const vec4f& a, const vec4f& b) {
  return dot(a - b, a - b);
}

inline vec4f slerp(const vec4f& a, const vec4f& b, float u) {
  // https://en.wikipedia.org/wiki/Slerp
  auto an = normalize(a), bn = normalize(b);
  auto d = dot(an, bn);
  if (d < 0) {
    bn = -bn;
    d  = -d;
  }
  if (d > (float)0.9995) return normalize(an + u * (bn - an));
  auto th = acos(clamp(d, (float)-1, (float)1));
  if (!th) return an;
  return an * (sin(th * (1 - u)) / sin(th)) + bn * (sin(th * u) / sin(th));
}

// Max element and clamp.
inline vec4f max(const vec4f& a, float b) {
  return {max(a.x, b), max(a.y, b), max(a.z, b), max(a.w, b)};
}
inline vec4f min(const vec4f& a, float b) {
  return {min(a.x, b), min(a.y, b), min(a.z, b), min(a.w, b)};
}
inline vec4f max(const vec4f& a, const vec4f& b) {
  return {max(a.x, b.x), max(a.y, b.y), max(a.z, b.z), max(a.w, b.w)};
}
inline vec4f min(const vec4f& a, const vec4f& b) {
  return {min(a.x, b.x), min(a.y, b.y), min(a.z, b.z), min(a.w, b.w)};
}
inline vec4f clamp(const vec4f& x, float min, float max) {
  return {clamp(x.x, min, max), clamp(x.y, min, max), clamp(x.z, min, max),
      clamp(x.w, min, max)};
}
inline vec4f lerp(const vec4f& a, const vec4f& b, float u) {
  return a * (1 - u) + b * u;
}
inline vec4f lerp(const vec4f& a, const vec4f& b, const vec4f& u) {
  return a * (1 - u) + b * u;
}

inline float max(const vec4f& a) { return max(max(max(a.x, a.y), a.z), a.w); }
inline float min(const vec4f& a) { return min(min(min(a.x, a.y), a.z), a.w); }
inline float sum(const vec4f& a) { return a.x + a.y + a.z + a.w; }
inline float mean(const vec4f& a) { return sum(a) / 4; }

// Functions applied to std::vector elements
inline vec4f abs(const vec4f& a) {
  return {abs(a.x), abs(a.y), abs(a.z), abs(a.w)};
};
inline vec4f sqrt(const vec4f& a) {
  return {sqrt(a.x), sqrt(a.y), sqrt(a.z), sqrt(a.w)};
};
inline vec4f exp(const vec4f& a) {
  return {exp(a.x), exp(a.y), exp(a.z), exp(a.w)};
};
inline vec4f log(const vec4f& a) {
  return {log(a.x), log(a.y), log(a.z), log(a.w)};
};
inline vec4f exp2(const vec4f& a) {
  return {exp2(a.x), exp2(a.y), exp2(a.z), exp2(a.w)};
};
inline vec4f log2(const vec4f& a) {
  return {log2(a.x), log2(a.y), log2(a.z), log2(a.w)};
};
inline vec4f pow(const vec4f& a, float b) {
  return {pow(a.x, b), pow(a.y, b), pow(a.z, b), pow(a.w, b)};
};
inline vec4f pow(const vec4f& a, const vec4f& b) {
  return {pow(a.x, b.x), pow(a.y, b.y), pow(a.z, b.z), pow(a.w, b.w)};
};
inline vec4f gain(const vec4f& a, float b) {
  return {gain(a.x, b), gain(a.y, b), gain(a.z, b), gain(a.w, b)};
};
inline bool isfinite(const vec4f& a) {
  return isfinite(a.x) && isfinite(a.y) && isfinite(a.z) && isfinite(a.w);
};
inline void swap(vec4f& a, vec4f& b) { std::swap(a, b); }

// Quaternion operatons represented as xi + yj + zk + w
// const auto identity_quat4f = vec4f{0, 0, 0, 1};
inline vec4f quat_mul(const vec4f& a, float b) {
  return {a.x * b, a.y * b, a.z * b, a.w * b};
}
inline vec4f quat_mul(const vec4f& a, const vec4f& b) {
  return {a.x * b.w + a.w * b.x + a.y * b.w - a.z * b.y,
      a.y * b.w + a.w * b.y + a.z * b.x - a.x * b.z,
      a.z * b.w + a.w * b.z + a.x * b.y - a.y * b.x,
      a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}
inline vec4f quat_conjugate(const vec4f& a) { return {-a.x, -a.y, -a.z, a.w}; }
inline vec4f quat_inverse(const vec4f& a) {
  return quat_conjugate(a) / dot(a, a);
}

}  // namespace yocto::math

// -----------------------------------------------------------------------------
// INTEGER VECTORS
// -----------------------------------------------------------------------------
namespace yocto::math {

// Vector data types
inline vec2i::vec2i() {}
inline vec2i::vec2i(int x, int y) : x{x}, y{y} {}
inline vec2i::vec2i(int v) : x{v}, y{v} {}
inline vec2i::operator vec2f() const { return {(float)x, (float)y}; }
inline vec2i::operator bool() const { return x || y; }

inline int& vec2i::operator[](int i) { return (&x)[i]; }
inline const int& vec2i::operator[](int i) const { return (&x)[i]; }

// Vector data types
inline vec3i::vec3i() {}
inline vec3i::vec3i(int x, int y, int z) : x{x}, y{y}, z{z} {}
inline vec3i::vec3i(const vec2i& v, int z) : x{v.x}, y{v.y}, z{z} {}
inline vec3i::vec3i(int v) : x{v}, y{v}, z{v} {}
inline vec3i::operator vec3f() const { return {(float)x, (float)y, (float)z}; }
inline vec3i::operator bool() const { return x || y || z; }

inline int& vec3i::operator[](int i) { return (&x)[i]; }
inline const int& vec3i::operator[](int i) const { return (&x)[i]; }

// Vector data types
inline vec4i::vec4i() {}
inline vec4i::vec4i(int x, int y, int z, int w) : x{x}, y{y}, z{z}, w{w} {}
inline vec4i::vec4i(const vec3i& v, int w) : x{v.x}, y{v.y}, z{v.z}, w{w} {}
inline vec4i::vec4i(int v) : x{v}, y{v}, z{v}, w{v} {}
inline vec4i::operator vec4f() const {
  return {(float)x, (float)y, (float)z, (float)w};
}
inline vec4i::operator bool() const { return x || y || z || w; }

inline int& vec4i::operator[](int i) { return (&x)[i]; }
inline const int& vec4i::operator[](int i) const { return (&x)[i]; }

// Vector data types
inline vec3b::vec3b() {}
inline vec3b::vec3b(byte x, byte y, byte z) : x{x}, y{y}, z{z} {}
inline vec3b::vec3b(byte v) : x{v}, y{v}, z{v} {}
inline vec3b::operator bool() const { return x || y || z; }

inline byte& vec3b::operator[](int i) { return (&x)[i]; }
inline const byte& vec3b::operator[](int i) const { return (&x)[i]; }

// Vector data types
inline vec4b::vec4b() {}
inline vec4b::vec4b(byte x, byte y, byte z, byte w) : x{x}, y{y}, z{z}, w{w} {}
inline vec4b::vec4b(byte v) : x{v}, y{v}, z{v}, w{v} {}
inline vec4b::operator bool() const { return x || y || z || w; }

inline byte& vec4b::operator[](int i) { return (&x)[i]; }
inline const byte& vec4b::operator[](int i) const { return (&x)[i]; }

// Element access
inline vec3i&       xyz(vec4i& a) { return (vec3i&)a; }
inline const vec3i& xyz(const vec4i& a) { return (const vec3i&)a; }

// Element access
inline vec3b&       xyz(vec4b& a) { return (vec3b&)a; }
inline const vec3b& xyz(const vec4b& a) { return (const vec3b&)a; }

// Vector comparison operations.
inline bool operator==(const vec2i& a, const vec2i& b) {
  return a.x == b.x && a.y == b.y;
}
inline bool operator!=(const vec2i& a, const vec2i& b) {
  return a.x != b.x || a.y != b.y;
}

// Vector operations.
inline vec2i operator+(const vec2i& a) { return a; }
inline vec2i operator-(const vec2i& a) { return {-a.x, -a.y}; }
inline vec2i operator+(const vec2i& a, const vec2i& b) {
  return {a.x + b.x, a.y + b.y};
}
inline vec2i operator+(const vec2i& a, int b) { return {a.x + b, a.y + b}; }
inline vec2i operator+(int a, const vec2i& b) { return {a + b.x, a + b.y}; }
inline vec2i operator-(const vec2i& a, const vec2i& b) {
  return {a.x - b.x, a.y - b.y};
}
inline vec2i operator-(const vec2i& a, int b) { return {a.x - b, a.y - b}; }
inline vec2i operator-(int a, const vec2i& b) { return {a - b.x, a - b.y}; }
inline vec2i operator*(const vec2i& a, const vec2i& b) {
  return {a.x * b.x, a.y * b.y};
}
inline vec2i operator*(const vec2i& a, int b) { return {a.x * b, a.y * b}; }
inline vec2i operator*(int a, const vec2i& b) { return {a * b.x, a * b.y}; }
inline vec2i operator/(const vec2i& a, const vec2i& b) {
  return {a.x / b.x, a.y / b.y};
}
inline vec2i operator/(const vec2i& a, int b) { return {a.x / b, a.y / b}; }
inline vec2i operator/(int a, const vec2i& b) { return {a / b.x, a / b.y}; }

// Vector assignments
inline vec2i& operator+=(vec2i& a, const vec2i& b) { return a = a + b; }
inline vec2i& operator+=(vec2i& a, int b) { return a = a + b; }
inline vec2i& operator-=(vec2i& a, const vec2i& b) { return a = a - b; }
inline vec2i& operator-=(vec2i& a, int b) { return a = a - b; }
inline vec2i& operator*=(vec2i& a, const vec2i& b) { return a = a * b; }
inline vec2i& operator*=(vec2i& a, int b) { return a = a * b; }
inline vec2i& operator/=(vec2i& a, const vec2i& b) { return a = a / b; }
inline vec2i& operator/=(vec2i& a, int b) { return a = a / b; }

// Max element and clamp.
inline vec2i max(const vec2i& a, int b) { return {max(a.x, b), max(a.y, b)}; }
inline vec2i min(const vec2i& a, int b) { return {min(a.x, b), min(a.y, b)}; }
inline vec2i max(const vec2i& a, const vec2i& b) {
  return {max(a.x, b.x), max(a.y, b.y)};
}
inline vec2i min(const vec2i& a, const vec2i& b) {
  return {min(a.x, b.x), min(a.y, b.y)};
}
inline vec2i clamp(const vec2i& x, int min, int max) {
  return {clamp(x.x, min, max), clamp(x.y, min, max)};
}

inline int max(const vec2i& a) { return max(a.x, a.y); }
inline int min(const vec2i& a) { return min(a.x, a.y); }
inline int sum(const vec2i& a) { return a.x + a.y; }

// Functions applied to std::vector elements
inline vec2i abs(const vec2i& a) { return {abs(a.x), abs(a.y)}; };
inline void  swap(vec2i& a, vec2i& b) { std::swap(a, b); }

// Vector comparison operations.
inline bool operator==(const vec3i& a, const vec3i& b) {
  return a.x == b.x && a.y == b.y && a.z == b.z;
}
inline bool operator!=(const vec3i& a, const vec3i& b) {
  return a.x != b.x || a.y != b.y || a.z != b.z;
}

// Vector operations.
inline vec3i operator+(const vec3i& a) { return a; }
inline vec3i operator-(const vec3i& a) { return {-a.x, -a.y, -a.z}; }
inline vec3i operator+(const vec3i& a, const vec3i& b) {
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}
inline vec3i operator+(const vec3i& a, int b) {
  return {a.x + b, a.y + b, a.z + b};
}
inline vec3i operator+(int a, const vec3i& b) {
  return {a + b.x, a + b.y, a + b.z};
}
inline vec3i operator-(const vec3i& a, const vec3i& b) {
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}
inline vec3i operator-(const vec3i& a, int b) {
  return {a.x - b, a.y - b, a.z - b};
}
inline vec3i operator-(int a, const vec3i& b) {
  return {a - b.x, a - b.y, a - b.z};
}
inline vec3i operator*(const vec3i& a, const vec3i& b) {
  return {a.x * b.x, a.y * b.y, a.z * b.z};
}
inline vec3i operator*(const vec3i& a, int b) {
  return {a.x * b, a.y * b, a.z * b};
}
inline vec3i operator*(int a, const vec3i& b) {
  return {a * b.x, a * b.y, a * b.z};
}
inline vec3i operator/(const vec3i& a, const vec3i& b) {
  return {a.x / b.x, a.y / b.y, a.z / b.z};
}
inline vec3i operator/(const vec3i& a, int b) {
  return {a.x / b, a.y / b, a.z / b};
}
inline vec3i operator/(int a, const vec3i& b) {
  return {a / b.x, a / b.y, a / b.z};
}

// Vector assignments
inline vec3i& operator+=(vec3i& a, const vec3i& b) { return a = a + b; }
inline vec3i& operator+=(vec3i& a, int b) { return a = a + b; }
inline vec3i& operator-=(vec3i& a, const vec3i& b) { return a = a - b; }
inline vec3i& operator-=(vec3i& a, int b) { return a = a - b; }
inline vec3i& operator*=(vec3i& a, const vec3i& b) { return a = a * b; }
inline vec3i& operator*=(vec3i& a, int b) { return a = a * b; }
inline vec3i& operator/=(vec3i& a, const vec3i& b) { return a = a / b; }
inline vec3i& operator/=(vec3i& a, int b) { return a = a / b; }

// Max element and clamp.
inline vec3i max(const vec3i& a, int b) {
  return {max(a.x, b), max(a.y, b), max(a.z, b)};
}
inline vec3i min(const vec3i& a, int b) {
  return {min(a.x, b), min(a.y, b), min(a.z, b)};
}
inline vec3i max(const vec3i& a, const vec3i& b) {
  return {max(a.x, b.x), max(a.y, b.y), max(a.z, b.z)};
}
inline vec3i min(const vec3i& a, const vec3i& b) {
  return {min(a.x, b.x), min(a.y, b.y), min(a.z, b.z)};
}
inline vec3i clamp(const vec3i& x, int min, int max) {
  return {clamp(x.x, min, max), clamp(x.y, min, max), clamp(x.z, min, max)};
}

inline int max(const vec3i& a) { return max(max(a.x, a.y), a.z); }
inline int min(const vec3i& a) { return min(min(a.x, a.y), a.z); }
inline int sum(const vec3i& a) { return a.x + a.y + a.z; }

// Functions applied to std::vector elements
inline vec3i abs(const vec3i& a) { return {abs(a.x), abs(a.y), abs(a.z)}; };
inline void  swap(vec3i& a, vec3i& b) { std::swap(a, b); }

// Vector comparison operations.
inline bool operator==(const vec4i& a, const vec4i& b) {
  return a.x == b.x && a.y == b.y && a.z == b.z && a.w == b.w;
}
inline bool operator!=(const vec4i& a, const vec4i& b) {
  return a.x != b.x || a.y != b.y || a.z != b.z || a.w != b.w;
}

// Vector operations.
inline vec4i operator+(const vec4i& a) { return a; }
inline vec4i operator-(const vec4i& a) { return {-a.x, -a.y, -a.z, -a.w}; }
inline vec4i operator+(const vec4i& a, const vec4i& b) {
  return {a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w};
}
inline vec4i operator+(const vec4i& a, int b) {
  return {a.x + b, a.y + b, a.z + b, a.w + b};
}
inline vec4i operator+(int a, const vec4i& b) {
  return {a + b.x, a + b.y, a + b.z, a + b.w};
}
inline vec4i operator-(const vec4i& a, const vec4i& b) {
  return {a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w};
}
inline vec4i operator-(const vec4i& a, int b) {
  return {a.x - b, a.y - b, a.z - b, a.w - b};
}
inline vec4i operator-(int a, const vec4i& b) {
  return {a - b.x, a - b.y, a - b.z, a - b.w};
}
inline vec4i operator*(const vec4i& a, const vec4i& b) {
  return {a.x * b.x, a.y * b.y, a.z * b.z, a.w * b.w};
}
inline vec4i operator*(const vec4i& a, int b) {
  return {a.x * b, a.y * b, a.z * b, a.w * b};
}
inline vec4i operator*(int a, const vec4i& b) {
  return {a * b.x, a * b.y, a * b.z, a * b.w};
}
inline vec4i operator/(const vec4i& a, const vec4i& b) {
  return {a.x / b.x, a.y / b.y, a.z / b.z, a.w / b.w};
}
inline vec4i operator/(const vec4i& a, int b) {
  return {a.x / b, a.y / b, a.z / b, a.w / b};
}
inline vec4i operator/(int a, const vec4i& b) {
  return {a / b.x, a / b.y, a / b.z, a / b.w};
}

// Vector assignments
inline vec4i& operator+=(vec4i& a, const vec4i& b) { return a = a + b; }
inline vec4i& operator+=(vec4i& a, int b) { return a = a + b; }
inline vec4i& operator-=(vec4i& a, const vec4i& b) { return a = a - b; }
inline vec4i& operator-=(vec4i& a, int b) { return a = a - b; }
inline vec4i& operator*=(vec4i& a, const vec4i& b) { return a = a * b; }
inline vec4i& operator*=(vec4i& a, int b) { return a = a * b; }
inline vec4i& operator/=(vec4i& a, const vec4i& b) { return a = a / b; }
inline vec4i& operator/=(vec4i& a, int b) { return a = a / b; }

// Max element and clamp.
inline vec4i max(const vec4i& a, int b) {
  return {max(a.x, b), max(a.y, b), max(a.z, b), max(a.w, b)};
}
inline vec4i min(const vec4i& a, int b) {
  return {min(a.x, b), min(a.y, b), min(a.z, b), min(a.w, b)};
}
inline vec4i max(const vec4i& a, const vec4i& b) {
  return {max(a.x, b.x), max(a.y, b.y), max(a.z, b.z), max(a.w, b.w)};
}
inline vec4i min(const vec4i& a, const vec4i& b) {
  return {min(a.x, b.x), min(a.y, b.y), min(a.z, b.z), min(a.w, b.w)};
}
inline vec4i clamp(const vec4i& x, int min, int max) {
  return {clamp(x.x, min, max), clamp(x.y, min, max), clamp(x.z, min, max),
      clamp(x.w, min, max)};
}

inline int max(const vec4i& a) { return max(max(max(a.x, a.y), a.z), a.w); }
inline int min(const vec4i& a) { return min(min(min(a.x, a.y), a.z), a.w); }
inline int sum(const vec4i& a) { return a.x + a.y + a.z + a.w; }

// Functions applied to std::vector elements
inline vec4i abs(const vec4i& a) {
  return {abs(a.x), abs(a.y), abs(a.z), abs(a.w)};
};
inline void swap(vec4i& a, vec4i& b) { std::swap(a, b); }

}  // namespace yocto::math

// -----------------------------------------------------------------------------
// IMPLEMENRTATION OF VECTOR HASHING
// -----------------------------------------------------------------------------
namespace std {

// Hash functor for std::vector for use with hash_map
template <>
struct hash<yocto::math::vec2i> {
  size_t operator()(const yocto::math::vec2i& v) const {
    static const auto hasher = std::hash<int>();
    auto              h      = (size_t)0;
    h ^= hasher(v.x) + 0x9e3779b9 + (h << 6) + (h >> 2);
    h ^= hasher(v.y) + 0x9e3779b9 + (h << 6) + (h >> 2);
    return h;
  }
};
template <>
struct hash<yocto::math::vec3i> {
  size_t operator()(const yocto::math::vec3i& v) const {
    static const auto hasher = std::hash<int>();
    auto              h      = (size_t)0;
    h ^= hasher(v.x) + 0x9e3779b9 + (h << 6) + (h >> 2);
    h ^= hasher(v.y) + 0x9e3779b9 + (h << 6) + (h >> 2);
    h ^= hasher(v.z) + 0x9e3779b9 + (h << 6) + (h >> 2);
    return h;
  }
};
template <>
struct hash<yocto::math::vec4i> {
  size_t operator()(const yocto::math::vec4i& v) const {
    static const auto hasher = std::hash<int>();
    auto              h      = (size_t)0;
    h ^= hasher(v.x) + 0x9e3779b9 + (h << 6) + (h >> 2);
    h ^= hasher(v.y) + 0x9e3779b9 + (h << 6) + (h >> 2);
    h ^= hasher(v.z) + 0x9e3779b9 + (h << 6) + (h >> 2);
    h ^= hasher(v.w) + 0x9e3779b9 + (h << 6) + (h >> 2);
    return h;
  }
};

}  // namespace std

// -----------------------------------------------------------------------------
// MATRICES
// -----------------------------------------------------------------------------
namespace yocto::math {

// Small Fixed-size matrices stored in column major format.
inline mat2f::mat2f() {}
inline mat2f::mat2f(const vec2f& x, const vec2f& y) : x{x}, y{y} {}

inline vec2f& mat2f::operator[](int i) { return (&x)[i]; }
inline const vec2f& mat2f::operator[](int i) const { return (&x)[i]; }

// Small Fixed-size matrices stored in column major format.
inline mat3f::mat3f() {}
inline mat3f::mat3f(const vec3f& x, const vec3f& y, const vec3f& z)
    : x{x}, y{y}, z{z} {}

inline vec3f& mat3f::operator[](int i) { return (&x)[i]; }
inline const vec3f& mat3f::operator[](int i) const { return (&x)[i]; }

// Small Fixed-size matrices stored in column major format.
inline mat4f::mat4f() {}
inline mat4f::mat4f(
    const vec4f& x, const vec4f& y, const vec4f& z, const vec4f& w)
    : x{x}, y{y}, z{z}, w{w} {}

inline vec4f& mat4f::operator[](int i) { return (&x)[i]; }
inline const vec4f& mat4f::operator[](int i) const { return (&x)[i]; }

// Matrix comparisons.
inline bool operator==(const mat2f& a, const mat2f& b) {
  return a.x == b.x && a.y == b.y;
}
inline bool operator!=(const mat2f& a, const mat2f& b) { return !(a == b); }

// Matrix operations.
inline mat2f operator+(const mat2f& a, const mat2f& b) {
  return {a.x + b.x, a.y + b.y};
}
inline mat2f operator*(const mat2f& a, float b) { return {a.x * b, a.y * b}; }
inline vec2f operator*(const mat2f& a, const vec2f& b) {
  return a.x * b.x + a.y * b.y;
}
inline vec2f operator*(const vec2f& a, const mat2f& b) {
  return {dot(a, b.x), dot(a, b.y)};
}
inline mat2f operator*(const mat2f& a, const mat2f& b) {
  return {a * b.x, a * b.y};
}

// Matrix assignments.
inline mat2f& operator+=(mat2f& a, const mat2f& b) { return a = a + b; }
inline mat2f& operator*=(mat2f& a, const mat2f& b) { return a = a * b; }
inline mat2f& operator*=(mat2f& a, float b) { return a = a * b; }

// Matrix diagonals and transposes.
inline vec2f diagonal(const mat2f& a) { return {a.x.x, a.y.y}; }
inline mat2f transpose(const mat2f& a) {
  return {{a.x.x, a.y.x}, {a.x.y, a.y.y}};
}

// Matrix adjoints, determinants and inverses.
inline float determinant(const mat2f& a) { return cross(a.x, a.y); }
inline mat2f adjoint(const mat2f& a) {
  return {{a.y.y, -a.x.y}, {-a.y.x, a.x.x}};
}
inline mat2f inverse(const mat2f& a) {
  return adjoint(a) * (1 / determinant(a));
}

// Matrix comparisons.
inline bool operator==(const mat3f& a, const mat3f& b) {
  return a.x == b.x && a.y == b.y && a.z == b.z;
}
inline bool operator!=(const mat3f& a, const mat3f& b) { return !(a == b); }

// Matrix operations.
inline mat3f operator+(const mat3f& a, const mat3f& b) {
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}
inline mat3f operator*(const mat3f& a, float b) {
  return {a.x * b, a.y * b, a.z * b};
}
inline vec3f operator*(const mat3f& a, const vec3f& b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
inline vec3f operator*(const vec3f& a, const mat3f& b) {
  return {dot(a, b.x), dot(a, b.y), dot(a, b.z)};
}
inline mat3f operator*(const mat3f& a, const mat3f& b) {
  return {a * b.x, a * b.y, a * b.z};
}

// Matrix assignments.
inline mat3f& operator+=(mat3f& a, const mat3f& b) { return a = a + b; }
inline mat3f& operator*=(mat3f& a, const mat3f& b) { return a = a * b; }
inline mat3f& operator*=(mat3f& a, float b) { return a = a * b; }

// Matrix diagonals and transposes.
inline vec3f diagonal(const mat3f& a) { return {a.x.x, a.y.y, a.z.z}; }
inline mat3f transpose(const mat3f& a) {
  return {
      {a.x.x, a.y.x, a.z.x},
      {a.x.y, a.y.y, a.z.y},
      {a.x.z, a.y.z, a.z.z},
  };
}

// Matrix adjoints, determinants and inverses.
inline float determinant(const mat3f& a) { return dot(a.x, cross(a.y, a.z)); }
inline mat3f adjoint(const mat3f& a) {
  return transpose(mat3f{cross(a.y, a.z), cross(a.z, a.x), cross(a.x, a.y)});
}
inline mat3f inverse(const mat3f& a) {
  return adjoint(a) * (1 / determinant(a));
}

// Constructs a basis from a direction
inline mat3f basis_fromz(const vec3f& v) {
  // https://graphics.pixar.com/library/OrthonormalB/paper.pdf
  auto z    = normalize(v);
  auto sign = copysignf(1.0f, z.z);
  auto a    = -1.0f / (sign + z.z);
  auto b    = z.x * z.y * a;
  auto x    = vec3f{1.0f + sign * z.x * z.x * a, sign * b, -sign * z.x};
  auto y    = vec3f{b, sign + z.y * z.y * a, -z.y};
  return {x, y, z};
}

// Matrix comparisons.
inline bool operator==(const mat4f& a, const mat4f& b) {
  return a.x == b.x && a.y == b.y && a.z == b.z && a.w == b.w;
}
inline bool operator!=(const mat4f& a, const mat4f& b) { return !(a == b); }

// Matrix operations.
inline mat4f operator+(const mat4f& a, const mat4f& b) {
  return {a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w};
}
inline mat4f operator*(const mat4f& a, float b) {
  return {a.x * b, a.y * b, a.z * b, a.w * b};
}
inline vec4f operator*(const mat4f& a, const vec4f& b) {
  return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}
inline vec4f operator*(const vec4f& a, const mat4f& b) {
  return {dot(a, b.x), dot(a, b.y), dot(a, b.z), dot(a, b.w)};
}
inline mat4f operator*(const mat4f& a, const mat4f& b) {
  return {a * b.x, a * b.y, a * b.z, a * b.w};
}

// Matrix assignments.
inline mat4f& operator+=(mat4f& a, const mat4f& b) { return a = a + b; }
inline mat4f& operator*=(mat4f& a, const mat4f& b) { return a = a * b; }
inline mat4f& operator*=(mat4f& a, float b) { return a = a * b; }

// Matrix diagonals and transposes.
inline vec4f diagonal(const mat4f& a) { return {a.x.x, a.y.y, a.z.z, a.w.w}; }
inline mat4f transpose(const mat4f& a) {
  return {
      {a.x.x, a.y.x, a.z.x, a.w.x},
      {a.x.y, a.y.y, a.z.y, a.w.y},
      {a.x.z, a.y.z, a.z.z, a.w.z},
      {a.x.w, a.y.w, a.z.w, a.w.w},
  };
}

}  // namespace yocto::math

// -----------------------------------------------------------------------------
// RIGID BODY TRANSFORMS/FRAMES
// -----------------------------------------------------------------------------
namespace yocto::math {

// Rigid frames stored as a column-major affine transform matrix.
inline frame2f::frame2f() {}
inline frame2f::frame2f(const vec2f& x, const vec2f& y, const vec2f& o)
    : x{x}, y{y}, o{o} {}
inline frame2f::frame2f(const vec2f& o) : x{1, 0}, y{0, 1}, o{o} {}
inline frame2f::frame2f(const mat2f& m, const vec2f& t)
    : x{m.x}, y{m.y}, o{t} {}
inline frame2f::frame2f(const mat3f& m)
    : x{m.x.x, m.x.y}, y{m.y.x, m.y.y}, o{m.z.x, m.z.y} {}
inline frame2f::operator mat3f() const { return {{x, 0}, {y, 0}, {o, 1}}; }

inline vec2f& frame2f::operator[](int i) { return (&x)[i]; }
inline const vec2f& frame2f::operator[](int i) const { return (&x)[i]; }

// Rigid frames stored as a column-major affine transform matrix.
inline frame3f::frame3f() {}
inline frame3f::frame3f(
    const vec3f& x, const vec3f& y, const vec3f& z, const vec3f& o)
    : x{x}, y{y}, z{z}, o{o} {}
inline frame3f::frame3f(const vec3f& o)
    : x{1, 0, 0}, y{0, 1, 0}, z{0, 0, 1}, o{o} {}
inline frame3f::frame3f(const mat3f& m, const vec3f& t)
    : x{m.x}, y{m.y}, z{m.z}, o{t} {}
inline frame3f::frame3f(const mat4f& m)
    : x{m.x.x, m.x.y, m.x.z}
    , y{m.y.x, m.y.y, m.y.z}
    , z{m.z.x, m.z.y, m.z.z}
    , o{m.w.x, m.w.y, m.w.z} {}
inline frame3f::operator mat4f() const {
  return {{x, 0}, {y, 0}, {z, 0}, {o, 1}};
}

inline vec3f& frame3f::operator[](int i) { return (&x)[i]; }
inline const vec3f& frame3f::operator[](int i) const { return (&x)[i]; }

// Frame properties
inline const mat2f& rotation(const frame2f& a) { return (const mat2f&)a; }

// Frame comparisons.
inline bool operator==(const frame2f& a, const frame2f& b) {
  return a.x == b.x && a.y == b.y && a.o == b.o;
}
inline bool operator!=(const frame2f& a, const frame2f& b) { return !(a == b); }

// Frame composition, equivalent to affine matrix product.
inline frame2f operator*(const frame2f& a, const frame2f& b) {
  return {rotation(a) * rotation(b), rotation(a) * b.o + a.o};
}
inline frame2f& operator*=(frame2f& a, const frame2f& b) { return a = a * b; }

// Frame inverse, equivalent to rigid affine inverse.
inline frame2f inverse(const frame2f& a, bool non_rigid) {
  if (non_rigid) {
    auto minv = inverse(rotation(a));
    return {minv, -(minv * a.o)};
  } else {
    auto minv = transpose(rotation(a));
    return {minv, -(minv * a.o)};
  }
}

// Frame properties
inline const mat3f& rotation(const frame3f& a) { return (const mat3f&)a; }

// Frame comparisons.
inline bool operator==(const frame3f& a, const frame3f& b) {
  return a.x == b.x && a.y == b.y && a.z == b.z && a.o == b.o;
}
inline bool operator!=(const frame3f& a, const frame3f& b) { return !(a == b); }

// Frame composition, equivalent to affine matrix product.
inline frame3f operator*(const frame3f& a, const frame3f& b) {
  return {rotation(a) * rotation(b), rotation(a) * b.o + a.o};
}
inline frame3f& operator*=(frame3f& a, const frame3f& b) { return a = a * b; }

// Frame inverse, equivalent to rigid affine inverse.
inline frame3f inverse(const frame3f& a, bool non_rigid) {
  if (non_rigid) {
    auto minv = inverse(rotation(a));
    return {minv, -(minv * a.o)};
  } else {
    auto minv = transpose(rotation(a));
    return {minv, -(minv * a.o)};
  }
}

// Frame construction from axis.
inline frame3f frame_fromz(const vec3f& o, const vec3f& v) {
  // https://graphics.pixar.com/library/OrthonormalB/paper.pdf
  auto z    = normalize(v);
  auto sign = copysignf(1.0f, z.z);
  auto a    = -1.0f / (sign + z.z);
  auto b    = z.x * z.y * a;
  auto x    = vec3f{1.0f + sign * z.x * z.x * a, sign * b, -sign * z.x};
  auto y    = vec3f{b, sign + z.y * z.y * a, -z.y};
  return {x, y, z, o};
}
inline frame3f frame_fromzx(const vec3f& o, const vec3f& z_, const vec3f& x_) {
  auto z = normalize(z_);
  auto x = orthonormalize(x_, z);
  auto y = normalize(cross(z, x));
  return {x, y, z, o};
}

}  // namespace yocto::math

// -----------------------------------------------------------------------------
// QUATERNIONS
// -----------------------------------------------------------------------------
namespace yocto::math {

// Quaternions to represent rotations
inline quat4f::quat4f() : x{0}, y{0}, z{0}, w{1} {}
inline quat4f::quat4f(float x, float y, float z, float w)
    : x{x}, y{y}, z{z}, w{w} {}

// Quaternion operatons
inline quat4f operator+(const quat4f& a, const quat4f& b) {
  return {a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w};
}
inline quat4f operator*(const quat4f& a, float b) {
  return {a.x * b, a.y * b, a.z * b, a.w * b};
}
inline quat4f operator/(const quat4f& a, float b) {
  return {a.x / b, a.y / b, a.z / b, a.w / b};
}
inline quat4f operator*(const quat4f& a, const quat4f& b) {
  return {a.x * b.w + a.w * b.x + a.y * b.w - a.z * b.y,
      a.y * b.w + a.w * b.y + a.z * b.x - a.x * b.z,
      a.z * b.w + a.w * b.z + a.x * b.y - a.y * b.x,
      a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}

// Quaterion operations
inline float dot(const quat4f& a, const quat4f& b) {
  return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}
inline float  length(const quat4f& a) { return sqrt(dot(a, a)); }
inline quat4f normalize(const quat4f& a) {
  auto l = length(a);
  return (l != 0) ? a / l : a;
}
inline quat4f conjugate(const quat4f& a) { return {-a.x, -a.y, -a.z, a.w}; }
inline quat4f inverse(const quat4f& a) { return conjugate(a) / dot(a, a); }
inline float  uangle(const quat4f& a, const quat4f& b) {
  auto d = dot(a, b);
  return d > 1 ? 0 : acos(d < -1 ? -1 : d);
}
inline quat4f lerp(const quat4f& a, const quat4f& b, float t) {
  return a * (1 - t) + b * t;
}
inline quat4f nlerp(const quat4f& a, const quat4f& b, float t) {
  return normalize(lerp(a, b, t));
}
inline quat4f slerp(const quat4f& a, const quat4f& b, float t) {
  auto th = uangle(a, b);
  return th == 0
             ? a
             : a * (sin(th * (1 - t)) / sin(th)) + b * (sin(th * t) / sin(th));
}

}  // namespace yocto::math

// -----------------------------------------------------------------------------
// AXIS ALIGNED BOUNDING BOXES
// -----------------------------------------------------------------------------
namespace yocto::math {

// Axis aligned bounding box represented as a min/max std::vector pairs.
inline bbox2f::bbox2f() {}
inline bbox2f::bbox2f(const vec2f& min, const vec2f& max)
    : min{min}, max{max} {}

inline vec2f& bbox2f::operator[](int i) { return (&min)[i]; }
inline const vec2f& bbox2f::operator[](int i) const { return (&min)[i]; }

// Axis aligned bounding box represented as a min/max std::vector pairs.
inline bbox3f::bbox3f() {}
inline bbox3f::bbox3f(const vec3f& min, const vec3f& max)
    : min{min}, max{max} {}

inline vec3f& bbox3f::operator[](int i) { return (&min)[i]; }
inline const vec3f& bbox3f::operator[](int i) const { return (&min)[i]; }

// Bounding box properties
inline vec2f center(const bbox2f& a) { return (a.min + a.max) / 2; }
inline vec2f size(const bbox2f& a) { return a.max - a.min; }

// Bounding box comparisons.
inline bool operator==(const bbox2f& a, const bbox2f& b) {
  return a.min == b.min && a.max == b.max;
}
inline bool operator!=(const bbox2f& a, const bbox2f& b) {
  return a.min != b.min || a.max != b.max;
}

// Bounding box expansions with points and other boxes.
inline bbox2f merge(const bbox2f& a, const vec2f& b) {
  return {min(a.min, b), max(a.max, b)};
}
inline bbox2f merge(const bbox2f& a, const bbox2f& b) {
  return {min(a.min, b.min), max(a.max, b.max)};
}
inline void expand(bbox2f& a, const vec2f& b) { a = merge(a, b); }
inline void expand(bbox2f& a, const bbox2f& b) { a = merge(a, b); }

// Bounding box properties
inline vec3f center(const bbox3f& a) { return (a.min + a.max) / 2; }
inline vec3f size(const bbox3f& a) { return a.max - a.min; }

// Bounding box comparisons.
inline bool operator==(const bbox3f& a, const bbox3f& b) {
  return a.min == b.min && a.max == b.max;
}
inline bool operator!=(const bbox3f& a, const bbox3f& b) {
  return a.min != b.min || a.max != b.max;
}

// Bounding box expansions with points and other boxes.
inline bbox3f merge(const bbox3f& a, const vec3f& b) {
  return {min(a.min, b), max(a.max, b)};
}
inline bbox3f merge(const bbox3f& a, const bbox3f& b) {
  return {min(a.min, b.min), max(a.max, b.max)};
}
inline void expand(bbox3f& a, const vec3f& b) { a = merge(a, b); }
inline void expand(bbox3f& a, const bbox3f& b) { a = merge(a, b); }

// Primitive bounds.
inline bbox3f point_bounds(const vec3f& p) { return {p, p}; }
inline bbox3f point_bounds(const vec3f& p, float r) {
  return {min(p - r, p + r), max(p - r, p + r)};
}
inline bbox3f line_bounds(const vec3f& p0, const vec3f& p1) {
  return {min(p0, p1), max(p0, p1)};
}
inline bbox3f line_bounds(
    const vec3f& p0, const vec3f& p1, float r0, float r1) {
  return {min(p0 - r0, p1 - r1), max(p0 + r0, p1 + r1)};
}
inline bbox3f triangle_bounds(
    const vec3f& p0, const vec3f& p1, const vec3f& p2) {
  return {min(p0, min(p1, p2)), max(p0, max(p1, p2))};
}
inline bbox3f quad_bounds(
    const vec3f& p0, const vec3f& p1, const vec3f& p2, const vec3f& p3) {
  return {min(p0, min(p1, min(p2, p3))), max(p0, max(p1, max(p2, p3)))};
}

}  // namespace yocto::math

// -----------------------------------------------------------------------------
// RAYS
// -----------------------------------------------------------------------------
namespace yocto::math {

// Rays with origin, direction and min/max t value.
inline ray2f::ray2f() {}
inline ray2f::ray2f(const vec2f& o, const vec2f& d, float tmin, float tmax)
    : o{o}, d{d}, tmin{tmin}, tmax{tmax} {}

// Rays with origin, direction and min/max t value.
inline ray3f::ray3f() {}
inline ray3f::ray3f(const vec3f& o, const vec3f& d, float tmin, float tmax)
    : o{o}, d{d}, tmin{tmin}, tmax{tmax} {}

}  // namespace yocto::math

// -----------------------------------------------------------------------------
// TRANSFORMS
// -----------------------------------------------------------------------------
namespace yocto::math {

// Transforms points, vectors and directions by matrices.
inline vec2f transform_point(const mat3f& a, const vec2f& b) {
  auto tvb = a * vec3f{b.x, b.y, 1};
  return vec2f{tvb.x, tvb.y} / tvb.z;
}
inline vec2f transform_vector(const mat3f& a, const vec2f& b) {
  auto tvb = a * vec3f{b.x, b.y, 0};
  return vec2f{tvb.x, tvb.y} / tvb.z;
}
inline vec2f transform_direction(const mat3f& a, const vec2f& b) {
  return normalize(transform_vector(a, b));
}
inline vec2f transform_normal(const mat3f& a, const vec2f& b) {
  return normalize(transform_vector(transpose(inverse(a)), b));
}
inline vec2f transform_vector(const mat2f& a, const vec2f& b) { return a * b; }
inline vec2f transform_direction(const mat2f& a, const vec2f& b) {
  return normalize(transform_vector(a, b));
}
inline vec2f transform_normal(const mat2f& a, const vec2f& b) {
  return normalize(transform_vector(transpose(inverse(a)), b));
}

inline vec3f transform_point(const mat4f& a, const vec3f& b) {
  auto tvb = a * vec4f{b.x, b.y, b.z, 1};
  return vec3f{tvb.x, tvb.y, tvb.z} / tvb.w;
}
inline vec3f transform_vector(const mat4f& a, const vec3f& b) {
  auto tvb = a * vec4f{b.x, b.y, b.z, 0};
  return vec3f{tvb.x, tvb.y, tvb.z};
}
inline vec3f transform_direction(const mat4f& a, const vec3f& b) {
  return normalize(transform_vector(a, b));
}
inline vec3f transform_vector(const mat3f& a, const vec3f& b) { return a * b; }
inline vec3f transform_direction(const mat3f& a, const vec3f& b) {
  return normalize(transform_vector(a, b));
}
inline vec3f transform_normal(const mat3f& a, const vec3f& b) {
  return normalize(transform_vector(transpose(inverse(a)), b));
}

// Transforms points, vectors and directions by frames.
inline vec2f transform_point(const frame2f& a, const vec2f& b) {
  return a.x * b.x + a.y * b.y + a.o;
}
inline vec2f transform_vector(const frame2f& a, const vec2f& b) {
  return a.x * b.x + a.y * b.y;
}
inline vec2f transform_direction(const frame2f& a, const vec2f& b) {
  return normalize(transform_vector(a, b));
}
inline vec2f transform_normal(
    const frame2f& a, const vec2f& b, bool non_rigid) {
  if (non_rigid) {
    return transform_normal(rotation(a), b);
  } else {
    return normalize(transform_vector(a, b));
  }
}

// Transforms points, vectors and directions by frames.
inline vec3f transform_point(const frame3f& a, const vec3f& b) {
  return a.x * b.x + a.y * b.y + a.z * b.z + a.o;
}
inline vec3f transform_vector(const frame3f& a, const vec3f& b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
inline vec3f transform_direction(const frame3f& a, const vec3f& b) {
  return normalize(transform_vector(a, b));
}
inline vec3f transform_normal(
    const frame3f& a, const vec3f& b, bool non_rigid) {
  if (non_rigid) {
    return transform_normal(rotation(a), b);
  } else {
    return normalize(transform_vector(a, b));
  }
}

// Transforms rays and bounding boxes by matrices.
inline ray3f transform_ray(const mat4f& a, const ray3f& b) {
  return {transform_point(a, b.o), transform_vector(a, b.d), b.tmin, b.tmax};
}
inline ray3f transform_ray(const frame3f& a, const ray3f& b) {
  return {transform_point(a, b.o), transform_vector(a, b.d), b.tmin, b.tmax};
}
inline bbox3f transform_bbox(const mat4f& a, const bbox3f& b) {
  auto corners = {vec3f{b.min.x, b.min.y, b.min.z},
      vec3f{b.min.x, b.min.y, b.max.z}, vec3f{b.min.x, b.max.y, b.min.z},
      vec3f{b.min.x, b.max.y, b.max.z}, vec3f{b.max.x, b.min.y, b.min.z},
      vec3f{b.max.x, b.min.y, b.max.z}, vec3f{b.max.x, b.max.y, b.min.z},
      vec3f{b.max.x, b.max.y, b.max.z}};
  auto xformed = bbox3f();
  for (auto& corner : corners)
    xformed = merge(xformed, transform_point(a, corner));
  return xformed;
}
inline bbox3f transform_bbox(const frame3f& a, const bbox3f& b) {
  auto corners = {vec3f{b.min.x, b.min.y, b.min.z},
      vec3f{b.min.x, b.min.y, b.max.z}, vec3f{b.min.x, b.max.y, b.min.z},
      vec3f{b.min.x, b.max.y, b.max.z}, vec3f{b.max.x, b.min.y, b.min.z},
      vec3f{b.max.x, b.min.y, b.max.z}, vec3f{b.max.x, b.max.y, b.min.z},
      vec3f{b.max.x, b.max.y, b.max.z}};
  auto xformed = bbox3f();
  for (auto& corner : corners)
    xformed = merge(xformed, transform_point(a, corner));
  return xformed;
}

// Translation, scaling and rotations transforms.
inline frame3f translation_frame(const vec3f& a) {
  return {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}, a};
}
inline frame3f scaling_frame(const vec3f& a) {
  return {{a.x, 0, 0}, {0, a.y, 0}, {0, 0, a.z}, {0, 0, 0}};
}
inline frame3f rotation_frame(const vec3f& axis, float angle) {
  auto s = sin(angle), c = cos(angle);
  auto vv = normalize(axis);
  return {{c + (1 - c) * vv.x * vv.x, (1 - c) * vv.x * vv.y + s * vv.z,
              (1 - c) * vv.x * vv.z - s * vv.y},
      {(1 - c) * vv.x * vv.y - s * vv.z, c + (1 - c) * vv.y * vv.y,
          (1 - c) * vv.y * vv.z + s * vv.x},
      {(1 - c) * vv.x * vv.z + s * vv.y, (1 - c) * vv.y * vv.z - s * vv.x,
          c + (1 - c) * vv.z * vv.z},
      {0, 0, 0}};
}
inline frame3f rotation_frame(const vec4f& quat) {
  auto v = quat;
  return {{v.w * v.w + v.x * v.x - v.y * v.y - v.z * v.z,
              (v.x * v.y + v.z * v.w) * 2, (v.z * v.x - v.y * v.w) * 2},
      {(v.x * v.y - v.z * v.w) * 2,
          v.w * v.w - v.x * v.x + v.y * v.y - v.z * v.z,
          (v.y * v.z + v.x * v.w) * 2},
      {(v.z * v.x + v.y * v.w) * 2, (v.y * v.z - v.x * v.w) * 2,
          v.w * v.w - v.x * v.x - v.y * v.y + v.z * v.z},
      {0, 0, 0}};
}
inline frame3f rotation_frame(const quat4f& quat) {
  auto v = quat;
  return {{v.w * v.w + v.x * v.x - v.y * v.y - v.z * v.z,
              (v.x * v.y + v.z * v.w) * 2, (v.z * v.x - v.y * v.w) * 2},
      {(v.x * v.y - v.z * v.w) * 2,
          v.w * v.w - v.x * v.x + v.y * v.y - v.z * v.z,
          (v.y * v.z + v.x * v.w) * 2},
      {(v.z * v.x + v.y * v.w) * 2, (v.y * v.z - v.x * v.w) * 2,
          v.w * v.w - v.x * v.x - v.y * v.y + v.z * v.z},
      {0, 0, 0}};
}
inline frame3f rotation_frame(const mat3f& rot) {
  return {rot.x, rot.y, rot.z, {0, 0, 0}};
}

// Lookat frame. Z-axis can be inverted with inv_xz.
inline frame3f lookat_frame(
    const vec3f& eye, const vec3f& center, const vec3f& up, bool inv_xz) {
  auto w = normalize(eye - center);
  auto u = normalize(cross(up, w));
  auto v = normalize(cross(w, u));
  if (inv_xz) {
    w = -w;
    u = -u;
  }
  return {u, v, w, eye};
}

// OpenGL frustum, ortho and perspecgive matrices.
inline mat4f frustum_mat(float l, float r, float b, float t, float n, float f) {
  return {{2 * n / (r - l), 0, 0, 0}, {0, 2 * n / (t - b), 0, 0},
      {(r + l) / (r - l), (t + b) / (t - b), -(f + n) / (f - n), -1},
      {0, 0, -2 * f * n / (f - n), 0}};
}
inline mat4f ortho_mat(float l, float r, float b, float t, float n, float f) {
  return {{2 / (r - l), 0, 0, 0}, {0, 2 / (t - b), 0, 0},
      {0, 0, -2 / (f - n), 0},
      {-(r + l) / (r - l), -(t + b) / (t - b), -(f + n) / (f - n), 1}};
}
inline mat4f ortho2d_mat(float left, float right, float bottom, float top) {
  return ortho_mat(left, right, bottom, top, -1, 1);
}
inline mat4f ortho_mat(float xmag, float ymag, float near, float far) {
  return {{1 / xmag, 0, 0, 0}, {0, 1 / ymag, 0, 0}, {0, 0, 2 / (near - far), 0},
      {0, 0, (far + near) / (near - far), 1}};
}
inline mat4f perspective_mat(float fovy, float aspect, float near, float far) {
  auto tg = tan(fovy / 2);
  return {{1 / (aspect * tg), 0, 0, 0}, {0, 1 / tg, 0, 0},
      {0, 0, (far + near) / (near - far), -1},
      {0, 0, 2 * far * near / (near - far), 0}};
}
inline mat4f perspective_mat(float fovy, float aspect, float near) {
  auto tg = tan(fovy / 2);
  return {{1 / (aspect * tg), 0, 0, 0}, {0, 1 / tg, 0, 0}, {0, 0, -1, -1},
      {0, 0, 2 * near, 0}};
}

// Rotation conversions.
inline std::pair<vec3f, float> rotation_axisangle(const vec4f& quat) {
  return {normalize(vec3f{quat.x, quat.y, quat.z}), 2 * acos(quat.w)};
}
inline vec4f rotation_quat(const vec3f& axis, float angle) {
  auto len = length(axis);
  if (!len) return {0, 0, 0, 1};
  return vec4f{sin(angle / 2) * axis.x / len, sin(angle / 2) * axis.y / len,
      sin(angle / 2) * axis.z / len, cos(angle / 2)};
}
inline vec4f rotation_quat(const vec4f& axisangle) {
  return rotation_quat(
      vec3f{axisangle.x, axisangle.y, axisangle.z}, axisangle.w);
}

}  // namespace yocto::math

// -----------------------------------------------------------------------------
// GEOMETRY UTILITIES
// -----------------------------------------------------------------------------
namespace yocto::math {

// Line properties.
inline vec3f line_tangent(const vec3f& p0, const vec3f& p1) {
  return normalize(p1 - p0);
}
inline float line_length(const vec3f& p0, const vec3f& p1) {
  return length(p1 - p0);
}

// Triangle properties.
inline vec3f triangle_normal(
    const vec3f& p0, const vec3f& p1, const vec3f& p2) {
  return normalize(cross(p1 - p0, p2 - p0));
}
inline float triangle_area(const vec3f& p0, const vec3f& p1, const vec3f& p2) {
  return length(cross(p1 - p0, p2 - p0)) / 2;
}

// Quad propeties.
inline vec3f quad_normal(
    const vec3f& p0, const vec3f& p1, const vec3f& p2, const vec3f& p3) {
  return normalize(triangle_normal(p0, p1, p3) + triangle_normal(p2, p3, p1));
}
inline float quad_area(
    const vec3f& p0, const vec3f& p1, const vec3f& p2, const vec3f& p3) {
  return triangle_area(p0, p1, p3) + triangle_area(p2, p3, p1);
}

// Interpolates values over a line parameterized from a to b by u. Same as lerp.
template <typename T>
inline T interpolate_line(const T& p0, const T& p1, float u) {
  return p0 * (1 - u) + p1 * u;
}
// Interpolates values over a triangle parameterized by u and v along the
// (p1-p0) and (p2-p0) directions. Same as barycentric interpolation.
template <typename T>
inline T interpolate_triangle(
    const T& p0, const T& p1, const T& p2, const vec2f& uv) {
  return p0 * (1 - uv.x - uv.y) + p1 * uv.x + p2 * uv.y;
}
// Interpolates values over a quad parameterized by u and v along the
// (p1-p0) and (p2-p1) directions. Same as bilinear interpolation.
template <typename T>
inline T interpolate_quad(
    const T& p0, const T& p1, const T& p2, const T& p3, const vec2f& uv) {
  if (uv.x + uv.y <= 1) {
    return interpolate_triangle(p0, p1, p3, uv);
  } else {
    return interpolate_triangle(p2, p3, p1, 1 - uv);
  }
}

// Interpolates values along a cubic Bezier segment parametrized by u.
template <typename T>
inline T interpolate_bezier(
    const T& p0, const T& p1, const T& p2, const T& p3, float u) {
  return p0 * (1 - u) * (1 - u) * (1 - u) + p1 * 3 * u * (1 - u) * (1 - u) +
         p2 * 3 * u * u * (1 - u) + p3 * u * u * u;
}
// Computes the derivative of a cubic Bezier segment parametrized by u.
template <typename T>
inline T interpolate_bezier_derivative(
    const T& p0, const T& p1, const T& p2, const T& p3, float u) {
  return (p1 - p0) * 3 * (1 - u) * (1 - u) + (p2 - p1) * 6 * u * (1 - u) +
         (p3 - p2) * 3 * u * u;
}

// Triangle tangent and bitangent from uv
inline std::pair<vec3f, vec3f> triangle_tangents_fromuv(const vec3f& p0,
    const vec3f& p1, const vec3f& p2, const vec2f& uv0, const vec2f& uv1,
    const vec2f& uv2) {
  // Follows the definition in http://www.terathon.com/code/tangent.html and
  // https://gist.github.com/aras-p/2843984
  // normal points up from texture space
  auto p   = p1 - p0;
  auto q   = p2 - p0;
  auto s   = vec2f{uv1.x - uv0.x, uv2.x - uv0.x};
  auto t   = vec2f{uv1.y - uv0.y, uv2.y - uv0.y};
  auto div = s.x * t.y - s.y * t.x;

  if (div != 0) {
    auto tu = vec3f{t.y * p.x - t.x * q.x, t.y * p.y - t.x * q.y,
                  t.y * p.z - t.x * q.z} /
              div;
    auto tv = vec3f{s.x * q.x - s.y * p.x, s.x * q.y - s.y * p.y,
                  s.x * q.z - s.y * p.z} /
              div;
    return {tu, tv};
  } else {
    return {{1, 0, 0}, {0, 1, 0}};
  }
}

// Quad tangent and bitangent from uv.
inline std::pair<vec3f, vec3f> quad_tangents_fromuv(const vec3f& p0,
    const vec3f& p1, const vec3f& p2, const vec3f& p3, const vec2f& uv0,
    const vec2f& uv1, const vec2f& uv2, const vec2f& uv3,
    const vec2f& current_uv) {
  if (current_uv.x + current_uv.y <= 1) {
    return triangle_tangents_fromuv(p0, p1, p3, uv0, uv1, uv3);
  } else {
    return triangle_tangents_fromuv(p2, p3, p1, uv2, uv3, uv1);
  }
}

}  // namespace yocto::math

// -----------------------------------------------------------------------------
// IMPLEMENRTATION OF RAY-PRIMITIVE INTERSECTION FUNCTIONS
// -----------------------------------------------------------------------------
namespace yocto::math {

// Intersect a ray with a point (approximate)
inline bool intersect_point(
    const ray3f& ray, const vec3f& p, float r, vec2f& uv, float& dist) {
  // find parameter for line-point minimum distance
  auto w = p - ray.o;
  auto t = dot(w, ray.d) / dot(ray.d, ray.d);

  // exit if not within bounds
  if (t < ray.tmin || t > ray.tmax) return false;

  // test for line-point distance vs point radius
  auto rp  = ray.o + ray.d * t;
  auto prp = p - rp;
  if (dot(prp, prp) > r * r) return false;

  // intersection occurred: set params and exit
  uv   = {0, 0};
  dist = t;
  return true;
}

// Intersect a ray with a line
inline bool intersect_line(const ray3f& ray, const vec3f& p0, const vec3f& p1,
    float r0, float r1, vec2f& uv, float& dist) {
  // setup intersection params
  auto u = ray.d;
  auto v = p1 - p0;
  auto w = ray.o - p0;

  // compute values to solve a linear system
  auto a   = dot(u, u);
  auto b   = dot(u, v);
  auto c   = dot(v, v);
  auto d   = dot(u, w);
  auto e   = dot(v, w);
  auto det = a * c - b * b;

  // check determinant and exit if lines are parallel
  // (could use EPSILONS if desired)
  if (det == 0) return false;

  // compute Parameters on both ray and segment
  auto t = (b * e - c * d) / det;
  auto s = (a * e - b * d) / det;

  // exit if not within bounds
  if (t < ray.tmin || t > ray.tmax) return false;

  // clamp segment param to segment corners
  s = clamp(s, (float)0, (float)1);

  // compute segment-segment distance on the closest points
  auto pr  = ray.o + ray.d * t;
  auto pl  = p0 + (p1 - p0) * s;
  auto prl = pr - pl;

  // check with the line radius at the same point
  auto d2 = dot(prl, prl);
  auto r  = r0 * (1 - s) + r1 * s;
  if (d2 > r * r) return {};

  // intersection occurred: set params and exit
  uv   = {s, sqrt(d2) / r};
  dist = t;
  return true;
}

// Intersect a ray with a triangle
inline bool intersect_triangle(const ray3f& ray, const vec3f& p0,
    const vec3f& p1, const vec3f& p2, vec2f& uv, float& dist) {
  // compute triangle edges
  auto edge1 = p1 - p0;
  auto edge2 = p2 - p0;

  // compute determinant to solve a linear system
  auto pvec = cross(ray.d, edge2);
  auto det  = dot(edge1, pvec);

  // check determinant and exit if triangle and ray are parallel
  // (could use EPSILONS if desired)
  if (det == 0) return false;
  auto inv_det = 1.0f / det;

  // compute and check first bricentric coordinated
  auto tvec = ray.o - p0;
  auto u    = dot(tvec, pvec) * inv_det;
  if (u < 0 || u > 1) return false;

  // compute and check second bricentric coordinated
  auto qvec = cross(tvec, edge1);
  auto v    = dot(ray.d, qvec) * inv_det;
  if (v < 0 || u + v > 1) return false;

  // compute and check ray parameter
  auto t = dot(edge2, qvec) * inv_det;
  if (t < ray.tmin || t > ray.tmax) return false;

  // intersection occurred: set params and exit
  uv   = {u, v};
  dist = t;
  return true;
}

// Intersect a ray with a quad.
inline bool intersect_quad(const ray3f& ray, const vec3f& p0, const vec3f& p1,
    const vec3f& p2, const vec3f& p3, vec2f& uv, float& dist) {
  if (p2 == p3) {
    return intersect_triangle(ray, p0, p1, p3, uv, dist);
  }
  auto hit  = false;
  auto tray = ray;
  if (intersect_triangle(tray, p0, p1, p3, uv, dist)) {
    hit       = true;
    tray.tmax = dist;
  }
  if (intersect_triangle(tray, p2, p3, p1, uv, dist)) {
    hit       = true;
    uv        = 1 - uv;
    tray.tmax = dist;
  }
  return hit;
}

// Intersect a ray with a axis-aligned bounding box
inline bool intersect_bbox(const ray3f& ray, const bbox3f& bbox) {
  // determine intersection ranges
  auto invd = 1.0f / ray.d;
  auto t0   = (bbox.min - ray.o) * invd;
  auto t1   = (bbox.max - ray.o) * invd;
  // flip based on range directions
  if (invd.x < 0.0f) swap(t0.x, t1.x);
  if (invd.y < 0.0f) swap(t0.y, t1.y);
  if (invd.z < 0.0f) swap(t0.z, t1.z);
  auto tmin = max(t0.z, max(t0.y, max(t0.x, ray.tmin)));
  auto tmax = min(t1.z, min(t1.y, min(t1.x, ray.tmax)));
  tmax *= 1.00000024f;  // for double: 1.0000000000000004
  return tmin <= tmax;
}

// Intersect a ray with a axis-aligned bounding box
inline bool intersect_bbox(
    const ray3f& ray, const vec3f& ray_dinv, const bbox3f& bbox) {
  auto it_min = (bbox.min - ray.o) * ray_dinv;
  auto it_max = (bbox.max - ray.o) * ray_dinv;
  auto tmin   = min(it_min, it_max);
  auto tmax   = max(it_min, it_max);
  auto t0     = max(max(tmin), ray.tmin);
  auto t1     = min(min(tmax), ray.tmax);
  t1 *= 1.00000024f;  // for double: 1.0000000000000004
  return t0 <= t1;
}

}  // namespace yocto::math

// -----------------------------------------------------------------------------
// IMPLEMENTATION OF POINT-PRIMITIVE DISTANCE FUNCTIONS
// -----------------------------------------------------------------------------
namespace yocto::math {

// Check if a point overlaps a position pos withint a maximum distance dist_max.
inline bool overlap_point(const vec3f& pos, float dist_max, const vec3f& p,
    float r, vec2f& uv, float& dist) {
  auto d2 = dot(pos - p, pos - p);
  if (d2 > (dist_max + r) * (dist_max + r)) return false;
  uv   = {0, 0};
  dist = sqrt(d2);
  return true;
}

// Compute the closest line uv to a give position pos.
inline float closestuv_line(
    const vec3f& pos, const vec3f& p0, const vec3f& p1) {
  auto ab = p1 - p0;
  auto d  = dot(ab, ab);
  // Project c onto ab, computing parameterized position d(t) = a + t*(b –
  // a)
  auto u = dot(pos - p0, ab) / d;
  u      = clamp(u, (float)0, (float)1);
  return u;
}

// Check if a line overlaps a position pos withint a maximum distance dist_max.
inline bool overlap_line(const vec3f& pos, float dist_max, const vec3f& p0,
    const vec3f& p1, float r0, float r1, vec2f& uv, float& dist) {
  auto u = closestuv_line(pos, p0, p1);
  // Compute projected position from the clamped t d = a + t * ab;
  auto p  = p0 + (p1 - p0) * u;
  auto r  = r0 + (r1 - r0) * u;
  auto d2 = dot(pos - p, pos - p);
  // check distance
  if (d2 > (dist_max + r) * (dist_max + r)) return false;
  // done
  uv   = {u, 0};
  dist = sqrt(d2);
  return true;
}

// Compute the closest triangle uv to a give position pos.
inline vec2f closestuv_triangle(
    const vec3f& pos, const vec3f& p0, const vec3f& p1, const vec3f& p2) {
  // this is a complicated test -> I probably "--"+prefix to use a sequence of
  // test (triangle body, and 3 edges)
  auto ab = p1 - p0;
  auto ac = p2 - p0;
  auto ap = pos - p0;

  auto d1 = dot(ab, ap);
  auto d2 = dot(ac, ap);

  // corner and edge cases
  if (d1 <= 0 && d2 <= 0) return {0, 0};

  auto bp = pos - p1;
  auto d3 = dot(ab, bp);
  auto d4 = dot(ac, bp);
  if (d3 >= 0 && d4 <= d3) return {1, 0};

  auto vc = d1 * d4 - d3 * d2;
  if ((vc <= 0) && (d1 >= 0) && (d3 <= 0)) return {d1 / (d1 - d3), 0};

  auto cp = pos - p2;
  auto d5 = dot(ab, cp);
  auto d6 = dot(ac, cp);
  if (d6 >= 0 && d5 <= d6) return {0, 1};

  auto vb = d5 * d2 - d1 * d6;
  if ((vb <= 0) && (d2 >= 0) && (d6 <= 0)) return {0, d2 / (d2 - d6)};

  auto va = d3 * d6 - d5 * d4;
  if ((va <= 0) && (d4 - d3 >= 0) && (d5 - d6 >= 0)) {
    auto w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
    return {1 - w, w};
  }

  // face case
  auto denom = 1 / (va + vb + vc);
  auto u     = vb * denom;
  auto v     = vc * denom;
  return {u, v};
}

// Check if a triangle overlaps a position pos withint a maximum distance
// dist_max.
inline bool overlap_triangle(const vec3f& pos, float dist_max, const vec3f& p0,
    const vec3f& p1, const vec3f& p2, float r0, float r1, float r2, vec2f& uv,
    float& dist) {
  auto cuv = closestuv_triangle(pos, p0, p1, p2);
  auto p   = p0 * (1 - cuv.x - cuv.y) + p1 * cuv.x + p2 * cuv.y;
  auto r   = r0 * (1 - cuv.x - cuv.y) + r1 * cuv.x + r2 * cuv.y;
  auto dd  = dot(p - pos, p - pos);
  if (dd > (dist_max + r) * (dist_max + r)) return false;
  uv   = cuv;
  dist = sqrt(dd);
  return true;
}

// Check if a quad overlaps a position pos withint a maximum distance dist_max.
inline bool overlap_quad(const vec3f& pos, float dist_max, const vec3f& p0,
    const vec3f& p1, const vec3f& p2, const vec3f& p3, float r0, float r1,
    float r2, float r3, vec2f& uv, float& dist) {
  if (p2 == p3) {
    return overlap_triangle(pos, dist_max, p0, p1, p3, r0, r1, r2, uv, dist);
  }
  auto hit = false;
  if (overlap_triangle(pos, dist_max, p0, p1, p3, r0, r1, r2, uv, dist)) {
    hit      = true;
    dist_max = dist;
  }
  if (!overlap_triangle(pos, dist_max, p2, p3, p1, r2, r3, r1, uv, dist)) {
    hit = true;
    uv  = 1 - uv;
    // dist_max = dist;
  }
  return hit;
}

// Check if a bbox overlaps a position pos withint a maximum distance dist_max.
inline bool distance_check_bbox(
    const vec3f& pos, float dist_max, const bbox3f& bbox) {
  // computing distance
  auto dd = 0.0f;

  // For each axis count any excess distance outside box extents
  if (pos.x < bbox.min.x) dd += (bbox.min.x - pos.x) * (bbox.min.x - pos.x);
  if (pos.x > bbox.max.x) dd += (pos.x - bbox.max.x) * (pos.x - bbox.max.x);
  if (pos.y < bbox.min.y) dd += (bbox.min.y - pos.y) * (bbox.min.y - pos.y);
  if (pos.y > bbox.max.y) dd += (pos.y - bbox.max.y) * (pos.y - bbox.max.y);
  if (pos.z < bbox.min.z) dd += (bbox.min.z - pos.z) * (bbox.min.z - pos.z);
  if (pos.z > bbox.max.z) dd += (pos.z - bbox.max.z) * (pos.z - bbox.max.z);

  // check distance
  return dd < dist_max * dist_max;
}

// Check if two bboxe overlap.
inline bool overlap_bbox(const bbox3f& bbox1, const bbox3f& bbox2) {
  if (bbox1.max.x < bbox2.min.x || bbox1.min.x > bbox2.max.x) return false;
  if (bbox1.max.y < bbox2.min.y || bbox1.min.y > bbox2.max.y) return false;
  if (bbox1.max.z < bbox2.min.z || bbox1.min.z > bbox2.max.z) return false;
  return true;
}

}  // namespace yocto::math

// -----------------------------------------------------------------------------
// IMPLEMENTATION FOR COLOR CONVERSION UTILITIES
// -----------------------------------------------------------------------------
namespace yocto::math {

// Conversion between flots and bytes
inline vec3b float_to_byte(const vec3f& a) {
  return {(byte)clamp(int(a.x * 256), 0, 255),
      (byte)clamp(int(a.y * 256), 0, 255), (byte)clamp(int(a.z * 256), 0, 255)};
}
inline vec3f byte_to_float(const vec3b& a) {
  return {a.x / 255.0f, a.y / 255.0f, a.z / 255.0f};
}
inline vec4b float_to_byte(const vec4f& a) {
  return {(byte)clamp(int(a.x * 256), 0, 255),
      (byte)clamp(int(a.y * 256), 0, 255), (byte)clamp(int(a.z * 256), 0, 255),
      (byte)clamp(int(a.w * 256), 0, 255)};
}
inline vec4f byte_to_float(const vec4b& a) {
  return {a.x / 255.0f, a.y / 255.0f, a.z / 255.0f, a.w / 255.0f};
}
inline byte float_to_byte(float a) { return (byte)clamp(int(a * 256), 0, 255); }
inline float  byte_to_float(byte a) { return a / 255.0f; }
inline ushort float_to_ushort(float a) {
  return (ushort)clamp(int(a * 65536), 0, 65535);
}
inline float ushort_to_float(ushort a) { return a / 65535.0f; }

// Luminance
inline float luminance(const vec3f& a) {
  return (0.2126f * a.x + 0.7152f * a.y + 0.0722f * a.z);
}

// sRGB non-linear curve
inline float srgb_to_rgb(float srgb) {
  return (srgb <= 0.04045) ? srgb / 12.92f
                           : pow((srgb + 0.055f) / (1.0f + 0.055f), 2.4f);
}
inline float rgb_to_srgb(float rgb) {
  return (rgb <= 0.0031308f) ? 12.92f * rgb
                             : (1 + 0.055f) * pow(rgb, 1 / 2.4f) - 0.055f;
}
inline vec3f srgb_to_rgb(const vec3f& srgb) {
  return {srgb_to_rgb(srgb.x), srgb_to_rgb(srgb.y), srgb_to_rgb(srgb.z)};
}
inline vec4f srgb_to_rgb(const vec4f& srgb) {
  return {
      srgb_to_rgb(srgb.x), srgb_to_rgb(srgb.y), srgb_to_rgb(srgb.z), srgb.w};
}
inline vec3f rgb_to_srgb(const vec3f& rgb) {
  return {rgb_to_srgb(rgb.x), rgb_to_srgb(rgb.y), rgb_to_srgb(rgb.z)};
}
inline vec4f rgb_to_srgb(const vec4f& rgb) {
  return {rgb_to_srgb(rgb.x), rgb_to_srgb(rgb.y), rgb_to_srgb(rgb.z), rgb.w};
}

// Apply contrast. Grey should be 0.18 for linear and 0.5 for gamma.
inline vec3f lincontrast(const vec3f& rgb, float contrast, float grey) {
  return max(zero3f, grey + (rgb - grey) * (contrast * 2));
}
// Apply contrast in log2. Grey should be 0.18 for linear and 0.5 for gamma.
inline vec3f logcontrast(const vec3f& rgb, float logcontrast, float grey) {
  auto epsilon  = (float)0.0001;
  auto log_grey = log2(grey);
  auto log_ldr  = log2(rgb + epsilon);
  auto adjusted = log_grey + (log_ldr - log_grey) * (logcontrast * 2);
  return max(zero3f, exp2(adjusted) - epsilon);
}
// Apply an s-shaped contrast.
inline vec3f contrast(const vec3f& rgb, float contrast) {
  return gain(rgb, 1 - contrast);
}
// Apply saturation.
inline vec3f saturate(
    const vec3f& rgb, float saturation, const vec3f& weights) {
  auto grey = dot(weights, rgb);
  return max(zero3f, grey + (rgb - grey) * (saturation * 2));
}

// Filmic tonemapping
inline vec3f tonemap_filmic(const vec3f& hdr_, bool accurate_fit = false) {
  if (!accurate_fit) {
    // https://knarkowicz.wordpress.com/2016/01/06/aces-filmic-tone-mapping-curve/
    auto hdr = hdr_ * 0.6f;  // brings it back to ACES range
    auto ldr = (hdr * hdr * 2.51f + hdr * 0.03f) /
               (hdr * hdr * 2.43f + hdr * 0.59f + 0.14f);
    return max(zero3f, ldr);
  } else {
    // https://github.com/TheRealMJP/BakingLab/blob/master/BakingLab/ACES.hlsl
    // sRGB => XYZ => D65_2_D60 => AP1 => RRT_SAT
    static const auto ACESInputMat = transpose(mat3f{
        {0.59719, 0.35458, 0.04823},
        {0.07600, 0.90834, 0.01566},
        {0.02840, 0.13383, 0.83777},
    });
    // ODT_SAT => XYZ => D60_2_D65 => sRGB
    static const auto ACESOutputMat = transpose(mat3f{
        {1.60475, -0.53108, -0.07367},
        {-0.10208, 1.10813, -0.00605},
        {-0.00327, -0.07276, 1.07602},
    });
    // RRT => ODT
    auto RRTAndODTFit = [](const vec3f& v) -> vec3f {
      return (v * v + v * 0.0245786f - 0.000090537f) /
             (v * v * 0.983729f + v * 0.4329510f + 0.238081f);
    };

    auto ldr = ACESOutputMat * RRTAndODTFit(ACESInputMat * hdr_);
    return max(zero3f, ldr);
  }
}

inline vec3f tonemap(const vec3f& hdr, float exposure, bool filmic, bool srgb) {
  auto rgb = hdr;
  if (exposure != 0) rgb *= exp2(exposure);
  if (filmic) rgb = tonemap_filmic(rgb);
  if (srgb) rgb = rgb_to_srgb(rgb);
  return rgb;
}
inline vec4f tonemap(const vec4f& hdr, float exposure, bool filmic, bool srgb) {
  return {tonemap(xyz(hdr), exposure, filmic, srgb), hdr.w};
}

// Convert between CIE XYZ and RGB
inline vec3f rgb_to_xyz(const vec3f& rgb) {
  // https://en.wikipedia.org/wiki/SRGB
  static const auto mat = mat3f{
      {0.4124, 0.2126, 0.0193},
      {0.3576, 0.7152, 0.1192},
      {0.1805, 0.0722, 0.9504},
  };
  return mat * rgb;
}
inline vec3f xyz_to_rgb(const vec3f& xyz) {
  // https://en.wikipedia.org/wiki/SRGB
  static const auto mat = mat3f{
      {+3.2406, -0.9689, +0.0557},
      {-1.5372, +1.8758, -0.2040},
      {-0.4986, +0.0415, +1.0570},
  };
  return mat * xyz;
}

// Convert between CIE XYZ and xyY
inline vec3f xyz_to_xyY(const vec3f& xyz) {
  if (xyz == zero3f) return zero3f;
  return {
      xyz.x / (xyz.x + xyz.y + xyz.z), xyz.y / (xyz.x + xyz.y + xyz.z), xyz.y};
}
inline vec3f xyY_to_xyz(const vec3f& xyY) {
  if (xyY.y == 0) return zero3f;
  return {xyY.x * xyY.z / xyY.y, xyY.z, (1 - xyY.x - xyY.y) * xyY.z / xyY.y};
}

// Convert HSV to RGB
inline vec3f hsv_to_rgb(const vec3f& hsv) {
  // from Imgui.cpp
  auto h = hsv.x, s = hsv.y, v = hsv.z;
  if (hsv.y == 0) return {v, v, v};

  h       = fmod(h, 1.0f) / (60.0f / 360.0f);
  int   i = (int)h;
  float f = h - (float)i;
  float p = v * (1 - s);
  float q = v * (1 - s * f);
  float t = v * (1 - s * (1 - f));

  switch (i) {
    case 0: return {v, t, p};
    case 1: return {q, v, p};
    case 2: return {p, v, t};
    case 3: return {p, q, v};
    case 4: return {t, p, v};
    case 5: return {v, p, q};
    default: return {v, p, q};
  }
}

inline vec3f rgb_to_hsv(const vec3f& rgb) {
  // from Imgui.cpp
  auto r = rgb.x, g = rgb.y, b = rgb.z;
  auto K = 0.f;
  if (g < b) {
    swap(g, b);
    K = -1;
  }
  if (r < g) {
    swap(r, g);
    K = -2 / 6.0f - K;
  }

  auto chroma = r - (g < b ? g : b);
  return {abs(K + (g - b) / (6 * chroma + 1e-20f)), chroma / (r + 1e-20f), r};
}

// Approximate color of blackbody radiation from wavelength in nm.
inline vec3f blackbody_to_rgb(float temperature) {
  // https://github.com/neilbartlett/color-temperature
  auto rgb = zero3f;
  if ((temperature / 100) < 66) {
    rgb.x = 255;
  } else {
    // a + b x + c Log[x] /.
    // {a -> 351.97690566805693`,
    // b -> 0.114206453784165`,
    // c -> -40.25366309332127
    // x -> (kelvin/100) - 55}
    rgb.x = (temperature / 100) - 55;
    rgb.x = 351.97690566805693f + 0.114206453784165f * rgb.x -
            40.25366309332127f * log(rgb.x);
    if (rgb.x < 0) rgb.x = 0;
    if (rgb.x > 255) rgb.x = 255;
  }

  if ((temperature / 100) < 66) {
    // a + b x + c Log[x] /.
    // {a -> -155.25485562709179`,
    // b -> -0.44596950469579133`,
    // c -> 104.49216199393888`,
    // x -> (kelvin/100) - 2}
    rgb.y = (temperature / 100) - 2;
    rgb.y = -155.25485562709179f - 0.44596950469579133f * rgb.y +
            104.49216199393888f * log(rgb.y);
    if (rgb.y < 0) rgb.y = 0;
    if (rgb.y > 255) rgb.y = 255;
  } else {
    // a + b x + c Log[x] /.
    // {a -> 325.4494125711974`,
    // b -> 0.07943456536662342`,
    // c -> -28.0852963507957`,
    // x -> (kelvin/100) - 50}
    rgb.y = (temperature / 100) - 50;
    rgb.y = 325.4494125711974f + 0.07943456536662342f * rgb.y -
            28.0852963507957f * log(rgb.y);
    if (rgb.y < 0) rgb.y = 0;
    if (rgb.y > 255) rgb.y = 255;
  }

  if ((temperature / 100) >= 66) {
    rgb.z = 255;
  } else {
    if ((temperature / 100) <= 20) {
      rgb.z = 0;
    } else {
      // a + b x + c Log[x] /.
      // {a -> -254.76935184120902`,
      // b -> 0.8274096064007395`,
      // c -> 115.67994401066147`,
      // x -> kelvin/100 - 10}
      rgb.z = (temperature / 100) - 10;
      rgb.z = -254.76935184120902f + 0.8274096064007395f * rgb.z +
              115.67994401066147f * log(rgb.z);
      if (rgb.z < 0) rgb.z = 0;
      if (rgb.z > 255) rgb.z = 255;
    }
  }

  return srgb_to_rgb(rgb / 255);
}

}  // namespace yocto::math

// -----------------------------------------------------------------------------
// IMPLEMENTATION FOR PERLIN NOISE
// -----------------------------------------------------------------------------
namespace yocto::math {

// clang-format off
inline float _stb__perlin_lerp(float a, float b, float t)
{
   return a + (b-a) * t;
}

inline int _stb__perlin_fastfloor(float a)
{
    int ai = (int) a;
    return (a < ai) ? ai-1 : ai;
}

// different grad function from Perlin's, but easy to modify to match reference
inline float _stb__perlin_grad(int hash, float x, float y, float z)
{
   static float basis[12][4] =
   {
      {  1, 1, 0 },
      { -1, 1, 0 },
      {  1,-1, 0 },
      { -1,-1, 0 },
      {  1, 0, 1 },
      { -1, 0, 1 },
      {  1, 0,-1 },
      { -1, 0,-1 },
      {  0, 1, 1 },
      {  0,-1, 1 },
      {  0, 1,-1 },
      {  0,-1,-1 },
   };

   // perlin's gradient has 12 cases so some get used 1/16th of the time
   // and some 2/16ths. We reduce bias by changing those fractions
   // to 5/64ths and 6/64ths, and the same 4 cases get the extra weight.
   static unsigned char indices[64] =
   {
      0,1,2,3,4,5,6,7,8,9,10,11,
      0,9,1,11,
      0,1,2,3,4,5,6,7,8,9,10,11,
      0,1,2,3,4,5,6,7,8,9,10,11,
      0,1,2,3,4,5,6,7,8,9,10,11,
      0,1,2,3,4,5,6,7,8,9,10,11,
   };

   // if you use reference permutation table, change 63 below to 15 to match reference
   // (this is why the ordering of the table above is funky)
   float *grad = basis[indices[hash & 63]];
   return grad[0]*x + grad[1]*y + grad[2]*z;
}

inline float _stb_perlin_noise3(float x, float y, float z, int x_wrap, int y_wrap, int z_wrap)
{
    // not same permutation table as Perlin's reference to avoid copyright issues;
    // Perlin's table can be found at http://mrl.nyu.edu/~perlin/noise/
    // @OPTIMIZE: should this be unsigned char instead of int for cache?
    static unsigned char _stb__perlin_randtab[512] =
    {
    23, 125, 161, 52, 103, 117, 70, 37, 247, 101, 203, 169, 124, 126, 44, 123,
    152, 238, 145, 45, 171, 114, 253, 10, 192, 136, 4, 157, 249, 30, 35, 72,
    175, 63, 77, 90, 181, 16, 96, 111, 133, 104, 75, 162, 93, 56, 66, 240,
    8, 50, 84, 229, 49, 210, 173, 239, 141, 1, 87, 18, 2, 198, 143, 57,
    225, 160, 58, 217, 168, 206, 245, 204, 199, 6, 73, 60, 20, 230, 211, 233,
    94, 200, 88, 9, 74, 155, 33, 15, 219, 130, 226, 202, 83, 236, 42, 172,
    165, 218, 55, 222, 46, 107, 98, 154, 109, 67, 196, 178, 127, 158, 13, 243,
    65, 79, 166, 248, 25, 224, 115, 80, 68, 51, 184, 128, 232, 208, 151, 122,
    26, 212, 105, 43, 179, 213, 235, 148, 146, 89, 14, 195, 28, 78, 112, 76,
    250, 47, 24, 251, 140, 108, 186, 190, 228, 170, 183, 139, 39, 188, 244, 246,
    132, 48, 119, 144, 180, 138, 134, 193, 82, 182, 120, 121, 86, 220, 209, 3,
    91, 241, 149, 85, 205, 150, 113, 216, 31, 100, 41, 164, 177, 214, 153, 231,
    38, 71, 185, 174, 97, 201, 29, 95, 7, 92, 54, 254, 191, 118, 34, 221,
    131, 11, 163, 99, 234, 81, 227, 147, 156, 176, 17, 142, 69, 12, 110, 62,
    27, 255, 0, 194, 59, 116, 242, 252, 19, 21, 187, 53, 207, 129, 64, 135,
    61, 40, 167, 237, 102, 223, 106, 159, 197, 189, 215, 137, 36, 32, 22, 5,

    // and a second copy so we don't need an extra mask or static initializer
    23, 125, 161, 52, 103, 117, 70, 37, 247, 101, 203, 169, 124, 126, 44, 123,
    152, 238, 145, 45, 171, 114, 253, 10, 192, 136, 4, 157, 249, 30, 35, 72,
    175, 63, 77, 90, 181, 16, 96, 111, 133, 104, 75, 162, 93, 56, 66, 240,
    8, 50, 84, 229, 49, 210, 173, 239, 141, 1, 87, 18, 2, 198, 143, 57,
    225, 160, 58, 217, 168, 206, 245, 204, 199, 6, 73, 60, 20, 230, 211, 233,
    94, 200, 88, 9, 74, 155, 33, 15, 219, 130, 226, 202, 83, 236, 42, 172,
    165, 218, 55, 222, 46, 107, 98, 154, 109, 67, 196, 178, 127, 158, 13, 243,
    65, 79, 166, 248, 25, 224, 115, 80, 68, 51, 184, 128, 232, 208, 151, 122,
    26, 212, 105, 43, 179, 213, 235, 148, 146, 89, 14, 195, 28, 78, 112, 76,
    250, 47, 24, 251, 140, 108, 186, 190, 228, 170, 183, 139, 39, 188, 244, 246,
    132, 48, 119, 144, 180, 138, 134, 193, 82, 182, 120, 121, 86, 220, 209, 3,
    91, 241, 149, 85, 205, 150, 113, 216, 31, 100, 41, 164, 177, 214, 153, 231,
    38, 71, 185, 174, 97, 201, 29, 95, 7, 92, 54, 254, 191, 118, 34, 221,
    131, 11, 163, 99, 234, 81, 227, 147, 156, 176, 17, 142, 69, 12, 110, 62,
    27, 255, 0, 194, 59, 116, 242, 252, 19, 21, 187, 53, 207, 129, 64, 135,
    61, 40, 167, 237, 102, 223, 106, 159, 197, 189, 215, 137, 36, 32, 22, 5,
    };

   float u,v,w;
   float n000,n001,n010,n011,n100,n101,n110,n111;
   float n00,n01,n10,n11;
   float n0,n1;

   unsigned int x_mask = (x_wrap-1) & 255;
   unsigned int y_mask = (y_wrap-1) & 255;
   unsigned int z_mask = (z_wrap-1) & 255;
   int px = _stb__perlin_fastfloor(x);
   int py = _stb__perlin_fastfloor(y);
   int pz = _stb__perlin_fastfloor(z);
   int x0 = px & x_mask, x1 = (px+1) & x_mask;
   int y0 = py & y_mask, y1 = (py+1) & y_mask;
   int z0 = pz & z_mask, z1 = (pz+1) & z_mask;
   int r0,r1, r00,r01,r10,r11;

   #define _stb__perlin_ease(a)   (((a*6-15)*a + 10) * a * a * a)

   x -= px; u = _stb__perlin_ease(x);
   y -= py; v = _stb__perlin_ease(y);
   z -= pz; w = _stb__perlin_ease(z);

   r0 = _stb__perlin_randtab[x0];
   r1 = _stb__perlin_randtab[x1];

   r00 = _stb__perlin_randtab[r0+y0];
   r01 = _stb__perlin_randtab[r0+y1];
   r10 = _stb__perlin_randtab[r1+y0];
   r11 = _stb__perlin_randtab[r1+y1];

   n000 = _stb__perlin_grad(_stb__perlin_randtab[r00+z0], x  , y  , z   );
   n001 = _stb__perlin_grad(_stb__perlin_randtab[r00+z1], x  , y  , z-1 );
   n010 = _stb__perlin_grad(_stb__perlin_randtab[r01+z0], x  , y-1, z   );
   n011 = _stb__perlin_grad(_stb__perlin_randtab[r01+z1], x  , y-1, z-1 );
   n100 = _stb__perlin_grad(_stb__perlin_randtab[r10+z0], x-1, y  , z   );
   n101 = _stb__perlin_grad(_stb__perlin_randtab[r10+z1], x-1, y  , z-1 );
   n110 = _stb__perlin_grad(_stb__perlin_randtab[r11+z0], x-1, y-1, z   );
   n111 = _stb__perlin_grad(_stb__perlin_randtab[r11+z1], x-1, y-1, z-1 );

   n00 = _stb__perlin_lerp(n000,n001,w);
   n01 = _stb__perlin_lerp(n010,n011,w);
   n10 = _stb__perlin_lerp(n100,n101,w);
   n11 = _stb__perlin_lerp(n110,n111,w);

   n0 = _stb__perlin_lerp(n00,n01,v);
   n1 = _stb__perlin_lerp(n10,n11,v);

   return _stb__perlin_lerp(n0,n1,u);
}

inline float _stb_perlin_ridge_noise3(float x, float y, float z,float lacunarity, float gain, float offset, int octaves,int x_wrap, int y_wrap, int z_wrap)
{
   int i;
   float frequency = 1.0f;
   float prev = 1.0f;
   float amplitude = 0.5f;
   float sum = 0.0f;

   for (i = 0; i < octaves; i++) {
      float r = (float)(_stb_perlin_noise3(x*frequency,y*frequency,z*frequency,x_wrap,y_wrap,z_wrap));
      r = r<0 ? -r : r; // fabs()
      r = offset - r;
      r = r*r;
      sum += r*amplitude*prev;
      prev = r;
      frequency *= lacunarity;
      amplitude *= gain;
   }
   return sum;
}

inline float _stb_perlin_fbm_noise3(float x, float y, float z,float lacunarity, float gain, int octaves,int x_wrap, int y_wrap, int z_wrap)
{
   int i;
   float frequency = 1.0f;
   float amplitude = 1.0f;
   float sum = 0.0f;

   for (i = 0; i < octaves; i++) {
      sum += _stb_perlin_noise3(x*frequency,y*frequency,z*frequency,x_wrap,y_wrap,z_wrap)*amplitude;
      frequency *= lacunarity;
      amplitude *= gain;
   }
   return sum;
}

inline float _stb_perlin_turbulence_noise3(float x, float y, float z, float lacunarity, float gain, int octaves,int x_wrap, int y_wrap, int z_wrap)
{
   int i;
   float frequency = 1.0f;
   float amplitude = 1.0f;
   float sum = 0.0f;

   for (i = 0; i < octaves; i++) {
      float r = _stb_perlin_noise3(x*frequency,y*frequency,z*frequency,x_wrap,y_wrap,z_wrap)*amplitude;
      r = r<0 ? -r : r; // fabs()
      sum += r;
      frequency *= lacunarity;
      amplitude *= gain;
   }
   return sum;
}
// clang-format on

// adapeted  stb_perlin.h
inline float perlin_noise(const vec3f& p, const vec3i& wrap) {
  return _stb_perlin_noise3(p.x, p.y, p.z, wrap.x, wrap.y, wrap.z);
}

// adapeted  stb_perlin.h
inline float perlin_ridge(const vec3f& p, float lacunarity, float gain,
    int octaves, float offset, const vec3i& wrap) {
  return _stb_perlin_ridge_noise3(
      p.x, p.y, p.z, lacunarity, gain, offset, octaves, wrap.x, wrap.y, wrap.z);
}

// adapeted  stb_perlin.h
inline float perlin_fbm(const vec3f& p, float lacunarity, float gain,
    int octaves, const vec3i& wrap) {
  return _stb_perlin_fbm_noise3(
      p.x, p.y, p.z, lacunarity, gain, octaves, wrap.x, wrap.y, wrap.z);
}

// adapeted  stb_perlin.h
inline float perlin_turbulence(const vec3f& p, float lacunarity, float gain,
    int octaves, const vec3i& wrap) {
  return _stb_perlin_turbulence_noise3(
      p.x, p.y, p.z, lacunarity, gain, octaves, wrap.x, wrap.y, wrap.z);
}

}  // namespace yocto::math

// -----------------------------------------------------------------------------
// IMPLEMENTATION OF SHADING FUNCTIONS
// -----------------------------------------------------------------------------
namespace yocto::math {

// Schlick approximation of the Fresnel term
inline vec3f fresnel_schlick(
    const vec3f& specular, const vec3f& normal, const vec3f& outgoing) {
  if (specular == zero3f) return zero3f;
  auto cosine = dot(normal, outgoing);
  return specular +
         (1 - specular) * pow(clamp(1 - abs(cosine), 0.0f, 1.0f), 5.0f);
}

// Compute the fresnel term for dielectrics.
inline float fresnel_dielectric(
    float eta, const vec3f& normal, const vec3f& outgoing) {
  // Implementation from
  // https://seblagarde.wordpress.com/2013/04/29/memo-on-fresnel-equations/
  auto cosw = abs(dot(normal, outgoing));

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

// Compute the fresnel term for metals.
inline vec3f fresnel_conductor(const vec3f& eta, const vec3f& etak,
    const vec3f& normal, const vec3f& outgoing) {
  // Implementation from
  // https://seblagarde.wordpress.com/2013/04/29/memo-on-fresnel-equations/
  auto cosw = dot(normal, outgoing);
  if (cosw <= 0) return zero3f;

  cosw       = clamp(cosw, (float)-1, (float)1);
  auto cos2  = cosw * cosw;
  auto sin2  = clamp(1 - cos2, (float)0, (float)1);
  auto eta2  = eta * eta;
  auto etak2 = etak * etak;

  auto t0       = eta2 - etak2 - sin2;
  auto a2plusb2 = sqrt(t0 * t0 + 4 * eta2 * etak2);
  auto t1       = a2plusb2 + cos2;
  auto a        = sqrt((a2plusb2 + t0) / 2);
  auto t2       = 2 * a * cosw;
  auto rs       = (t1 - t2) / (t1 + t2);

  auto t3 = cos2 * a2plusb2 + sin2 * sin2;
  auto t4 = t2 * sin2;
  auto rp = rs * (t3 - t4) / (t3 + t4);

  return (rp + rs) / 2;
}

// Convert eta to reflectivity
inline vec3f eta_to_reflectivity(const vec3f& eta) {
  return ((eta - 1) * (eta - 1)) / ((eta + 1) * (eta + 1));
}
// Convert reflectivity to  eta.
inline vec3f reflectivity_to_eta(const vec3f& reflectivity_) {
  auto reflectivity = clamp(reflectivity_, 0.0f, 0.99f);
  return (1 + sqrt(reflectivity)) / (1 - sqrt(reflectivity));
}
// Convert conductor eta to reflectivity
inline vec3f eta_to_reflectivity(const vec3f& eta, const vec3f& etak) {
  return ((eta - 1) * (eta - 1) + etak * etak) /
         ((eta + 1) * (eta + 1) + etak * etak);
}
// Convert eta to edge tint parametrization
inline std::pair<vec3f, vec3f> eta_to_edgetint(
    const vec3f& eta, const vec3f& etak) {
  auto reflectivity = eta_to_reflectivity(eta, etak);
  auto numer        = (1 + sqrt(reflectivity)) / (1 - sqrt(reflectivity)) - eta;
  auto denom        = (1 + sqrt(reflectivity)) / (1 - sqrt(reflectivity)) -
               (1 - reflectivity) / (1 + reflectivity);
  auto edgetint = numer / denom;
  return {reflectivity, edgetint};
}
// Convert reflectivity and edge tint to eta.
inline std::pair<vec3f, vec3f> edgetint_to_eta(
    const vec3f& reflectivity, const vec3f& edgetint) {
  auto r = clamp(reflectivity, 0.0f, 0.99f);
  auto g = edgetint;

  auto r_sqrt = sqrt(r);
  auto n_min  = (1 - r) / (1 + r);
  auto n_max  = (1 + r_sqrt) / (1 - r_sqrt);

  auto n  = lerp(n_max, n_min, g);
  auto k2 = ((n + 1) * (n + 1) * r - (n - 1) * (n - 1)) / (1 - r);
  k2      = max(k2, 0.0f);
  auto k  = sqrt(k2);
  return {n, k};
}

// Evaluate microfacet distribution
inline float microfacet_distribution(
    float roughness, const vec3f& normal, const vec3f& halfway, bool ggx) {
  // https://google.github.io/filament/Filament.html#materialsystem/specularbrdf
  // http://graphicrants.blogspot.com/2013/08/specular-brdf-reference.html
  auto cosine = dot(normal, halfway);
  if (cosine <= 0) return 0;
  auto roughness2 = roughness * roughness;
  auto cosine2    = cosine * cosine;
  if (ggx) {
    return roughness2 / (pif * (cosine2 * roughness2 + 1 - cosine2) *
                            (cosine2 * roughness2 + 1 - cosine2));
  } else {
    return exp((cosine2 - 1) / (roughness2 * cosine2)) /
           (pif * roughness2 * cosine2 * cosine2);
  }
}

// Evaluate the microfacet shadowing1
inline float microfacet_shadowing1(float roughness, const vec3f& normal,
    const vec3f& halfway, const vec3f& direction, bool ggx) {
  // https://google.github.io/filament/Filament.html#materialsystem/specularbrdf
  // http://graphicrants.blogspot.com/2013/08/specular-brdf-reference.html
  auto cosine  = dot(normal, direction);
  auto cosineh = dot(halfway, direction);
  if (cosine * cosineh <= 0) return 0;
  auto roughness2 = roughness * roughness;
  auto cosine2    = cosine * cosine;
  if (ggx) {
    return 2 * abs(cosine) /
           (abs(cosine) + sqrt(cosine2 - roughness2 * cosine2 + roughness2));
  } else {
    auto ci = abs(cosine) / (roughness * sqrt(1 - cosine2));
    return ci < 1.6f ? (3.535f * ci + 2.181f * ci * ci) /
                           (1.0f + 2.276f * ci + 2.577f * ci * ci)
                     : 1.0f;
  }
}

// Evaluate microfacet shadowing
inline float microfacet_shadowing(float roughness, const vec3f& normal,
    const vec3f& halfway, const vec3f& outgoing, const vec3f& incoming,
    bool ggx) {
  return microfacet_shadowing1(roughness, normal, halfway, outgoing, ggx) *
         microfacet_shadowing1(roughness, normal, halfway, incoming, ggx);
}

// Sample a microfacet ditribution.
inline vec3f sample_microfacet(
    float roughness, const vec3f& normal, const vec2f& rn, bool ggx) {
  auto phi   = 2 * pif * rn.x;
  auto theta = 0.0f;
  if (ggx) {
    theta = atan(roughness * sqrt(rn.y / (1 - rn.y)));
  } else {
    auto roughness2 = roughness * roughness;
    theta           = atan(sqrt(-roughness2 * log(1 - rn.y)));
  }
  auto local_half_vector = vec3f{
      cos(phi) * sin(theta), sin(phi) * sin(theta), cos(theta)};
  return transform_direction(basis_fromz(normal), local_half_vector);
}

// Pdf for microfacet distribution sampling.
inline float sample_microfacet_pdf(
    float roughness, const vec3f& normal, const vec3f& halfway, bool ggx) {
  auto cosine = dot(normal, halfway);
  if (cosine < 0) return 0;
  return microfacet_distribution(roughness, normal, halfway, ggx) * cosine;
}

// Sample a microfacet ditribution with the distribution of visible normals.
inline vec3f sample_microfacet(float roughness, const vec3f& normal,
    const vec3f& outgoing, const vec2f& rn, bool ggx) {
  // http://jcgt.org/published/0007/04/01/
  if (ggx) {
    // move to local coordinate system
    auto basis   = basis_fromz(normal);
    auto Ve      = transform_direction(transpose(basis), outgoing);
    auto alpha_x = roughness, alpha_y = roughness;
    // Section 3.2: transforming the view direction to the hemisphere
    // configuration
    auto Vh = normalize(vec3f{alpha_x * Ve.x, alpha_y * Ve.y, Ve.z});
    // Section 4.1: orthonormal basis (with special case if cross product is
    // zero)
    auto lensq = Vh.x * Vh.x + Vh.y * Vh.y;
    auto T1    = lensq > 0 ? vec3f{-Vh.y, Vh.x, 0} * (1 / sqrt(lensq))
                        : vec3f{1, 0, 0};
    auto T2 = cross(Vh, T1);
    // Section 4.2: parameterization of the projected area
    auto r   = sqrt(rn.y);
    auto phi = 2 * pif * rn.x;
    auto t1  = r * cos(phi);
    auto t2  = r * sin(phi);
    auto s   = 0.5f * (1 + Vh.z);
    t2       = (1 - s) * sqrt(1 - t1 * t1) + s * t2;
    // Section 4.3: reprojection onto hemisphere
    auto Nh = t1 * T1 + t2 * T2 + sqrt(max(0.0f, 1 - t1 * t1 - t2 * t2)) * Vh;
    // Section 3.4: transforming the normal back to the ellipsoid configuration
    auto Ne = normalize(vec3f{alpha_x * Nh.x, alpha_y * Nh.y, max(0.0f, Nh.z)});
    // move to world coordinate
    auto local_halfway = Ne;
    return transform_direction(basis, local_halfway);
  } else {
    throw std::invalid_argument{"not implemented yet"};
  }
}

// Pdf for microfacet distribution sampling with the distribution of visible
// normals.
inline float sample_microfacet_pdf(float roughness, const vec3f& normal,
    const vec3f& halfway, const vec3f& outgoing, bool ggx) {
  // http://jcgt.org/published/0007/04/01/
  if (dot(normal, halfway) < 0) return 0;
  if (dot(halfway, outgoing) < 0) return 0;
  return microfacet_distribution(roughness, normal, halfway, ggx) *
         microfacet_shadowing1(roughness, normal, halfway, outgoing, ggx) *
         max(0.0f, dot(halfway, outgoing)) / abs(dot(normal, outgoing));
}

// Evaluate a diffuse BRDF lobe.
inline vec3f eval_diffuse_reflection(
    const vec3f& normal, const vec3f& outgoing, const vec3f& incoming) {
  if (dot(normal, incoming) <= 0 || dot(normal, outgoing) <= 0) return zero3f;
  return vec3f{1} / pif * dot(normal, incoming);
}

// Evaluate a translucent BRDF lobe.
inline vec3f eval_diffuse_transmission(
    const vec3f& normal, const vec3f& outgoing, const vec3f& incoming) {
  if (dot(normal, incoming) * dot(normal, outgoing) >= 0) return zero3f;
  return vec3f{1} / pif * abs(dot(normal, incoming));
}

// Evaluate a specular BRDF lobe.
inline vec3f eval_microfacet_reflection(float ior, float roughness,
    const vec3f& normal, const vec3f& outgoing, const vec3f& incoming) {
  if (dot(normal, incoming) <= 0 || dot(normal, outgoing) <= 0) return zero3f;
  auto halfway = normalize(incoming + outgoing);
  auto F       = fresnel_dielectric(ior, halfway, incoming);
  auto D       = microfacet_distribution(roughness, normal, halfway);
  auto G = microfacet_shadowing(roughness, normal, halfway, outgoing, incoming);
  return vec3f{1} * F * D * G /
         (4 * dot(normal, outgoing) * dot(normal, incoming)) *
         dot(normal, incoming);
}

// Evaluate a metal BRDF lobe.
inline vec3f eval_microfacet_reflection(const vec3f& eta, const vec3f& etak,
    float roughness, const vec3f& normal, const vec3f& outgoing,
    const vec3f& incoming) {
  if (dot(normal, incoming) <= 0 || dot(normal, outgoing) <= 0) return zero3f;
  auto halfway = normalize(incoming + outgoing);
  auto F       = fresnel_conductor(eta, etak, halfway, incoming);
  auto D       = microfacet_distribution(roughness, normal, halfway);
  auto G = microfacet_shadowing(roughness, normal, halfway, outgoing, incoming);
  return F * D * G / (4 * dot(normal, outgoing) * dot(normal, incoming)) *
         dot(normal, incoming);
}

// Evaluate a transmission BRDF lobe.
inline vec3f eval_microfacet_transmission(float ior, float roughness,
    const vec3f& normal, const vec3f& outgoing, const vec3f& incoming) {
  if (dot(normal, incoming) >= 0 || dot(normal, outgoing) <= 0) return zero3f;
  auto reflected = reflect(-incoming, normal);
  auto halfway   = normalize(reflected + outgoing);
  // auto F       = fresnel_schlick(
  //     point.reflectance, abs(dot(halfway, outgoing)), entering);
  auto D = microfacet_distribution(roughness, normal, halfway);
  auto G = microfacet_shadowing(
      roughness, normal, halfway, outgoing, reflected);
  return vec3f{1} * D * G /
         (4 * dot(normal, outgoing) * dot(normal, reflected)) *
         (dot(normal, reflected));
}

// Evaluate a refraction BRDF lobe.
inline vec3f eval_microfacet_refraction(float ior, float roughness,
    const vec3f& normal, const vec3f& outgoing, const vec3f& incoming) {
  auto entering  = dot(normal, outgoing) >= 0;
  auto up_normal = entering ? normal : -normal;
  auto rel_ior   = entering ? ior : (1 / ior);
  if (dot(normal, incoming) * dot(normal, outgoing) >= 0) {
    auto halfway = normalize(incoming + outgoing);
    auto F       = fresnel_dielectric(rel_ior, halfway, outgoing);
    auto D       = microfacet_distribution(roughness, up_normal, halfway);
    auto G       = microfacet_shadowing(
        roughness, up_normal, halfway, outgoing, incoming);
    return vec3f{1} * F * D * G /
           abs(4 * dot(normal, outgoing) * dot(normal, incoming)) *
           abs(dot(normal, incoming));
  } else {
    auto halfway = -normalize(rel_ior * incoming + outgoing) *
                   (entering ? 1 : -1);
    auto F = fresnel_dielectric(rel_ior, halfway, outgoing);
    auto D = microfacet_distribution(roughness, up_normal, halfway);
    auto G = microfacet_shadowing(
        roughness, up_normal, halfway, outgoing, incoming);
    // [Walter 2007] equation 21
    return vec3f{1} *
           abs((dot(outgoing, halfway) * dot(incoming, halfway)) /
               (dot(outgoing, normal) * dot(incoming, normal))) *
           (1 - F) * D * G /
           pow(rel_ior * dot(halfway, incoming) + dot(halfway, outgoing), 2) *
           abs(dot(normal, incoming));
  }
}

// Sample a diffuse BRDF lobe.
inline vec3f sample_diffuse_reflection(
    const vec3f& normal, const vec3f& outgoing, const vec2f& rn) {
  if (dot(normal, outgoing) <= 0) return zero3f;
  return sample_hemisphere_cos(normal, rn);
}

// Sample a translucency BRDF lobe.
inline vec3f sample_diffuse_transmission(
    const vec3f& normal, const vec3f& outgoing, const vec2f& rn) {
  auto up_normal = dot(normal, outgoing) >= 0 ? normal : -normal;
  return sample_hemisphere_cos(-up_normal, rn);
}

// Sample a specular BRDF lobe.
inline vec3f sample_microfacet_reflection(float ior, float roughness,
    const vec3f& normal, const vec3f& outgoing, const vec2f& rn) {
  if (dot(normal, outgoing) <= 0) return zero3f;
  // auto halfway = sample_microfacet(roughness, normal, outgoing, rn);
  auto halfway = sample_microfacet(roughness, normal, rn);
  return reflect(outgoing, halfway);
}

// Sample a metal BRDF lobe.
inline vec3f sample_microfacet_reflection(const vec3f& eta, const vec3f& etak,
    float roughness, const vec3f& normal, const vec3f& outgoing,
    const vec2f& rn) {
  if (dot(normal, outgoing) <= 0) return zero3f;
  // auto halfway = sample_microfacet(roughness, normal, outgoing, rn);
  auto halfway = sample_microfacet(roughness, normal, rn);
  return reflect(outgoing, halfway);
}

// Sample a transmission BRDF lobe.
inline vec3f sample_microfacet_transmission(float ior, float roughness,
    const vec3f& normal, const vec3f& outgoing, const vec2f& rn) {
  if (dot(normal, outgoing) <= 0) return zero3f;
  auto halfway = sample_microfacet(roughness, normal, rn);
  // auto halfway   = sample_microfacet(roughness, normal, outgoing, rn);
  auto reflected = reflect(outgoing, halfway);
  return -reflect(reflected, normal);
}

// Sample a refraction BRDF lobe.
inline vec3f sample_microfacet_refraction(float ior, float roughness,
    const vec3f& normal, const vec3f& outgoing, float rnl, const vec2f& rn) {
  auto entering  = dot(normal, outgoing) >= 0;
  auto up_normal = entering ? normal : -normal;
  // auto halfway   = sample_microfacet(roughness, up_normal, outgoing, rn);
  auto halfway = sample_microfacet(roughness, up_normal, rn);
  if (rnl < fresnel_dielectric(entering ? ior : (1 / ior), halfway, outgoing)) {
    return reflect(outgoing, halfway);
  } else {
    return refract(outgoing, halfway, entering ? (1 / ior) : ior);
  }
}

// Pdf for diffuse BRDF lobe sampling.
inline float sample_diffuse_reflection_pdf(
    const vec3f& normal, const vec3f& outgoing, const vec3f& incoming) {
  if (dot(normal, incoming) <= 0 || dot(normal, outgoing) <= 0) return 0;
  return sample_hemisphere_cos_pdf(normal, incoming);
}

// Pdf for translucency BRDF lobe sampling.
inline float sample_diffuse_transmission_pdf(
    const vec3f& normal, const vec3f& outgoing, const vec3f& incoming) {
  if (dot(normal, incoming) * dot(normal, outgoing) >= 0) return 0;
  auto up_normal = dot(normal, outgoing) >= 0 ? normal : -normal;
  return sample_hemisphere_cos_pdf(-up_normal, incoming);
}

// Pdf for specular BRDF lobe sampling.
inline float sample_microfacet_reflection_pdf(float ior, float roughness,
    const vec3f& normal, const vec3f& outgoing, const vec3f& incoming) {
  if (dot(normal, incoming) <= 0 || dot(normal, outgoing) <= 0) return 0;
  auto halfway = normalize(outgoing + incoming);
  // return sample_microfacet_pdf(roughness, normal, halfway, outgoing) /
  return sample_microfacet_pdf(roughness, normal, halfway) /
         (4 * abs(dot(outgoing, halfway)));
}

// Pdf for metal BRDF lobe sampling.
inline float sample_microfacet_reflection_pdf(const vec3f& eta,
    const vec3f& etak, float roughness, const vec3f& normal,
    const vec3f& outgoing, const vec3f& incoming) {
  if (dot(normal, incoming) <= 0 || dot(normal, outgoing) <= 0) return 0;
  auto halfway = normalize(outgoing + incoming);
  // return sample_microfacet_pdf(roughness, normal, halfway, outgoing) /
  return sample_microfacet_pdf(roughness, normal, halfway) /
         (4 * abs(dot(outgoing, halfway)));
}

// Pdf for transmission BRDF lobe sampling.
inline float sample_microfacet_transmission_pdf(float ior, float roughness,
    const vec3f& normal, const vec3f& outgoing, const vec3f& incoming) {
  if (dot(normal, incoming) >= 0 || dot(normal, outgoing) <= 0) return 0;
  auto reflected = reflect(-incoming, normal);
  auto halfway   = normalize(reflected + outgoing);
  // auto d         = sample_microfacet_pdf(roughness, normal, halfway,
  // outgoing);
  auto d = sample_microfacet_pdf(roughness, normal, halfway);
  return d / (4 * abs(dot(outgoing, halfway)));
}

// Pdf for refraction BRDF lobe sampling.
inline float sample_microfacet_refraction_pdf(float ior, float roughness,
    const vec3f& normal, const vec3f& outgoing, const vec3f& incoming) {
  auto entering  = dot(normal, outgoing) >= 0;
  auto up_normal = entering ? normal : -normal;
  auto rel_ior   = entering ? ior : (1 / ior);
  if (dot(normal, incoming) * dot(normal, outgoing) >= 0) {
    auto halfway = normalize(incoming + outgoing);
    return fresnel_dielectric(rel_ior, halfway, outgoing) *
           //  sample_microfacet_pdf(roughness, up_normal, halfway, outgoing) /
           sample_microfacet_pdf(roughness, up_normal, halfway) /
           (4 * abs(dot(outgoing, halfway)));
  } else {
    auto halfway = -normalize(rel_ior * incoming + outgoing) *
                   (entering ? 1 : -1);
    // [Walter 2007] equation 17
    return (1 - fresnel_dielectric(rel_ior, halfway, outgoing)) *
           //  sample_microfacet_pdf(roughness, up_normal, halfway, outgoing) *
           sample_microfacet_pdf(roughness, up_normal, halfway) *
           abs(dot(halfway, outgoing)) /
           pow(rel_ior * dot(halfway, incoming) + dot(halfway, outgoing), 2);
  }
}

// Evaluate a delta specular BRDF lobe.
inline vec3f eval_delta_reflection(float ior, const vec3f& normal,
    const vec3f& outgoing, const vec3f& incoming) {
  if (dot(normal, incoming) <= 0 || dot(normal, outgoing) <= 0) return zero3f;
  return vec3f{1} * fresnel_dielectric(ior, normal, outgoing);
}

// Evaluate a delta metal BRDF lobe.
inline vec3f eval_delta_reflection(const vec3f& eta, const vec3f& etak,
    const vec3f& normal, const vec3f& outgoing, const vec3f& incoming) {
  if (dot(normal, incoming) <= 0 || dot(normal, outgoing) <= 0) return zero3f;
  return fresnel_conductor(eta, etak, normal, outgoing);
}

// Evaluate a delta transmission BRDF lobe.
inline vec3f eval_delta_transmission(float ior, const vec3f& normal,
    const vec3f& outgoing, const vec3f& incoming) {
  if (dot(normal, incoming) >= 0 || dot(normal, outgoing) <= 0) return zero3f;
  return vec3f{1};
}

// Evaluate a delta refraction BRDF lobe.
inline vec3f eval_delta_refraction(float ior, const vec3f& normal,
    const vec3f& outgoing, const vec3f& incoming) {
  if (abs(ior - 1) < 1e-3)
    return dot(normal, incoming) * dot(normal, outgoing) <= 0 ? vec3f{1}
                                                              : vec3f{0};
  auto entering  = dot(normal, outgoing) >= 0;
  auto up_normal = entering ? normal : -normal;
  auto rel_ior   = entering ? ior : (1 / ior);
  if (dot(normal, incoming) * dot(normal, outgoing) >= 0) {
    return vec3f{1} * fresnel_dielectric(rel_ior, up_normal, outgoing);
  } else {
    return vec3f{1} * (1 / (rel_ior * rel_ior)) *
           (1 - fresnel_dielectric(rel_ior, up_normal, outgoing));
  }
}

// Sample a delta specular BRDF lobe.
inline vec3f sample_delta_reflection(
    float ior, const vec3f& normal, const vec3f& outgoing) {
  if (dot(normal, outgoing) <= 0) return zero3f;
  return reflect(outgoing, normal);
}

// Sample a delta metal BRDF lobe.
inline vec3f sample_delta_reflection(const vec3f& eta, const vec3f& etak,
    const vec3f& normal, const vec3f& outgoing) {
  if (dot(normal, outgoing) <= 0) return zero3f;
  return reflect(outgoing, normal);
}

// Sample a delta transmission BRDF lobe.
inline vec3f sample_delta_transmission(
    float ior, const vec3f& normal, const vec3f& outgoing) {
  if (dot(normal, outgoing) <= 0) return zero3f;
  return -outgoing;
}

// Sample a delta refraction BRDF lobe.
inline vec3f sample_delta_refraction(
    float ior, const vec3f& normal, const vec3f& outgoing, float rnl) {
  if (abs(ior - 1) < 1e-3) return -outgoing;
  auto entering  = dot(normal, outgoing) >= 0;
  auto up_normal = entering ? normal : -normal;
  auto rel_ior   = entering ? ior : (1 / ior);
  if (rnl < fresnel_dielectric(rel_ior, up_normal, outgoing)) {
    return reflect(outgoing, up_normal);
  } else {
    return refract(outgoing, up_normal, 1 / rel_ior);
  }
}

// Pdf for delta specular BRDF lobe sampling.
inline float sample_delta_reflection_pdf(float ior, const vec3f& normal,
    const vec3f& outgoing, const vec3f& incoming) {
  if (dot(normal, incoming) <= 0 || dot(normal, outgoing) <= 0) return 0;
  return 1;
}

// Pdf for delta metal BRDF lobe sampling.
inline float sample_delta_reflection_pdf(const vec3f& eta, const vec3f& etak,
    const vec3f& normal, const vec3f& outgoing, const vec3f& incoming) {
  if (dot(normal, incoming) <= 0 || dot(normal, outgoing) <= 0) return 0;
  return 1;
}

// Pdf for delta transmission BRDF lobe sampling.
inline float sample_delta_transmission_pdf(float ior, const vec3f& normal,
    const vec3f& outgoing, const vec3f& incoming) {
  if (dot(normal, incoming) >= 0 || dot(normal, outgoing) <= 0) return 0;
  return 1;
}

// Pdf for delta refraction BRDF lobe sampling.
inline float sample_delta_refraction_pdf(float ior, const vec3f& normal,
    const vec3f& outgoing, const vec3f& incoming) {
  if (abs(ior - 1) < 1e-3)
    return dot(normal, incoming) * dot(normal, outgoing) < 0 ? 1 : 0;
  auto entering  = dot(normal, outgoing) >= 0;
  auto up_normal = entering ? normal : -normal;
  auto rel_ior   = entering ? ior : (1 / ior);
  if (dot(normal, incoming) * dot(normal, outgoing) >= 0) {
    return fresnel_dielectric(rel_ior, up_normal, outgoing);
  } else {
    return (1 - fresnel_dielectric(rel_ior, up_normal, outgoing));
  }
}

// Convert mean-free-path to transmission
inline vec3f mfp_to_transmission(const vec3f& mfp, float depth) {
  return exp(-depth / mfp);
}

// Evaluate transmittance
inline vec3f eval_transmittance(const vec3f& density, float distance) {
  return exp(-density * distance);
}

// Sample a distance proportionally to transmittance
inline float sample_transmittance(
    const vec3f& density, float max_distance, float rl, float rd) {
  auto channel  = clamp((int)(rl * 3), 0, 2);
  auto distance = (density[channel] == 0) ? flt_max
                                          : -log(1 - rd) / density[channel];
  return min(distance, max_distance);
}

// Pdf for distance sampling
inline float sample_transmittance_pdf(
    const vec3f& density, float distance, float max_distance) {
  if (distance < max_distance) {
    return sum(density * exp(-density * distance)) / 3;
  } else {
    return sum(exp(-density * max_distance)) / 3;
  }
}

// Evaluate phase function
inline float eval_phasefunction(
    float anisotropy, const vec3f& outgoing, const vec3f& incoming) {
  auto cosine = -dot(outgoing, incoming);
  auto denom  = 1 + anisotropy * anisotropy - 2 * anisotropy * cosine;
  return (1 - anisotropy * anisotropy) / (4 * pif * denom * sqrt(denom));
}

// Sample phase function
inline vec3f sample_phasefunction(
    float anisotropy, const vec3f& outgoing, const vec2f& rn) {
  auto cos_theta = 0.0f;
  if (abs(anisotropy) < 1e-3f) {
    cos_theta = 1 - 2 * rn.y;
  } else {
    float square = (1 - anisotropy * anisotropy) /
                   (1 + anisotropy - 2 * anisotropy * rn.y);
    cos_theta = (1 + anisotropy * anisotropy - square * square) /
                (2 * anisotropy);
  }

  auto sin_theta      = sqrt(max(0.0f, 1 - cos_theta * cos_theta));
  auto phi            = 2 * pif * rn.x;
  auto local_incoming = vec3f{
      sin_theta * cos(phi), sin_theta * sin(phi), cos_theta};
  return basis_fromz(-outgoing) * local_incoming;
}

// Pdf for phase function sampling
inline float sample_phasefunction_pdf(
    float anisotropy, const vec3f& outgoing, const vec3f& incoming) {
  return eval_phasefunction(anisotropy, outgoing, incoming);
}

}  // namespace yocto::math

// -----------------------------------------------------------------------------
// IMPLEMENTATION OF MONETACARLO SAMPLING FUNCTIONS
// -----------------------------------------------------------------------------
namespace yocto::math {

// Sample an hemispherical direction with uniform distribution.
inline vec3f sample_hemisphere(const vec2f& ruv) {
  auto z   = ruv.y;
  auto r   = sqrt(clamp(1 - z * z, 0.0f, 1.0f));
  auto phi = 2 * pif * ruv.x;
  return {r * cos(phi), r * sin(phi), z};
}
inline float sample_hemisphere_pdf(const vec3f& direction) {
  return (direction.z <= 0) ? 0 : 1 / (2 * pif);
}

// Sample an hemispherical direction with uniform distribution.
inline vec3f sample_hemisphere(const vec3f& normal, const vec2f& ruv) {
  auto z               = ruv.y;
  auto r               = sqrt(clamp(1 - z * z, 0.0f, 1.0f));
  auto phi             = 2 * pif * ruv.x;
  auto local_direction = vec3f{r * cos(phi), r * sin(phi), z};
  return transform_direction(basis_fromz(normal), local_direction);
}
inline float sample_hemisphere_pdf(
    const vec3f& normal, const vec3f& direction) {
  return (dot(normal, direction) <= 0) ? 0 : 1 / (2 * pif);
}

// Sample a spherical direction with uniform distribution.
inline vec3f sample_sphere(const vec2f& ruv) {
  auto z   = 2 * ruv.y - 1;
  auto r   = sqrt(clamp(1 - z * z, 0.0f, 1.0f));
  auto phi = 2 * pif * ruv.x;
  return {r * cos(phi), r * sin(phi), z};
}
inline float sample_sphere_pdf(const vec3f& w) { return 1 / (4 * pif); }

// Sample an hemispherical direction with cosine distribution.
inline vec3f sample_hemisphere_cos(const vec2f& ruv) {
  auto z   = sqrt(ruv.y);
  auto r   = sqrt(1 - z * z);
  auto phi = 2 * pif * ruv.x;
  return {r * cos(phi), r * sin(phi), z};
}
inline float sample_hemisphere_cos_pdf(const vec3f& direction) {
  return (direction.z <= 0) ? 0 : direction.z / pif;
}

// Sample an hemispherical direction with cosine distribution.
inline vec3f sample_hemisphere_cos(const vec3f& normal, const vec2f& ruv) {
  auto z               = sqrt(ruv.y);
  auto r               = sqrt(1 - z * z);
  auto phi             = 2 * pif * ruv.x;
  auto local_direction = vec3f{r * cos(phi), r * sin(phi), z};
  return transform_direction(basis_fromz(normal), local_direction);
}
inline float sample_hemisphere_cos_pdf(
    const vec3f& normal, const vec3f& direction) {
  auto cosw = dot(normal, direction);
  return (cosw <= 0) ? 0 : cosw / pif;
}

// Sample an hemispherical direction with cosine power distribution.
inline vec3f sample_hemisphere_cospower(float exponent, const vec2f& ruv) {
  auto z   = pow(ruv.y, 1 / (exponent + 1));
  auto r   = sqrt(1 - z * z);
  auto phi = 2 * pif * ruv.x;
  return {r * cos(phi), r * sin(phi), z};
}
inline float sample_hemisphere_cospower_pdf(
    float exponent, const vec3f& direction) {
  return (direction.z <= 0)
             ? 0
             : pow(direction.z, exponent) * (exponent + 1) / (2 * pif);
}

// Sample a point uniformly on a disk.
inline vec2f sample_disk(const vec2f& ruv) {
  auto r   = sqrt(ruv.y);
  auto phi = 2 * pif * ruv.x;
  return {cos(phi) * r, sin(phi) * r};
}
inline float sample_disk_pdf() { return 1 / pif; }

// Sample a point uniformly on a cylinder, without caps.
inline vec3f sample_cylinder(const vec2f& ruv) {
  auto phi = 2 * pif * ruv.x;
  return {sin(phi), cos(phi), ruv.y * 2 - 1};
}
inline float sample_cylinder_pdf(const vec3f& point) { return 1 / pif; }

// Sample a point uniformly on a triangle returning the baricentric coordinates.
inline vec2f sample_triangle(const vec2f& ruv) {
  return {1 - sqrt(ruv.x), ruv.y * sqrt(ruv.x)};
}

// Sample a point uniformly on a triangle.
inline vec3f sample_triangle(
    const vec3f& p0, const vec3f& p1, const vec3f& p2, const vec2f& ruv) {
  auto uv = sample_triangle(ruv);
  return p0 * (1 - uv.x - uv.y) + p1 * uv.x + p2 * uv.y;
}
// Pdf for uniform triangle sampling, i.e. triangle area.
inline float sample_triangle_pdf(
    const vec3f& p0, const vec3f& p1, const vec3f& p2) {
  return 2 / length(cross(p1 - p0, p2 - p0));
}

// Sample an index with uniform distribution.
inline int sample_uniform(int size, float r) {
  return clamp((int)(r * size), 0, size - 1);
}
inline float sample_uniform_pdf(int size) { return (float)1 / (float)size; }

// Sample an index with uniform distribution.
inline float sample_uniform(const std::vector<float>& elements, float r) {
  if (elements.empty()) return {};
  auto size = (int)elements.size();
  return elements[clamp((int)(r * size), 0, size - 1)];
}
inline float sample_uniform_pdf(const std::vector<float>& elements) {
  if (elements.empty()) return 0;
  return 1.0f / (int)elements.size();
}

// Sample a discrete distribution represented by its cdf.
inline int sample_discrete(const std::vector<float>& cdf, float r) {
  r        = clamp(r * cdf.back(), (float)0, cdf.back() - (float)0.00001);
  auto idx = (int)(std::upper_bound(cdf.data(), cdf.data() + cdf.size(), r) -
                   cdf.data());
  return clamp(idx, 0, (int)cdf.size() - 1);
}
// Pdf for uniform discrete distribution sampling.
inline float sample_discrete_pdf(const std::vector<float>& cdf, int idx) {
  if (idx == 0) return cdf.at(0);
  return cdf.at(idx) - cdf.at(idx - 1);
}

// Sample a discrete distribution represented by its cdf.
inline int sample_discrete_cdf(const std::vector<float>& cdf, float r) {
  r        = clamp(r * cdf.back(), (float)0, cdf.back() - (float)0.00001);
  auto idx = (int)(std::upper_bound(cdf.data(), cdf.data() + cdf.size(), r) -
                   cdf.data());
  return clamp(idx, 0, (int)cdf.size() - 1);
}
// Pdf for uniform discrete distribution sampling.
inline float sample_discrete_cdf_pdf(const std::vector<float>& cdf, int idx) {
  if (idx == 0) return cdf.at(0);
  return cdf.at(idx) - cdf.at(idx - 1);
}

// Sample a discrete distribution represented by its cdf.
inline int sample_discrete_weights(const std::vector<float>& weights, float r) {
  auto sum = 0.0f;
  for (auto weight : weights) sum += weight;
  r            = clamp(r * sum, (float)0, sum - (float)0.00001);
  auto cur_sum = 0.0f;
  for (auto idx = 0; idx < weights.size(); idx++) {
    cur_sum += weights[idx];
    if (r < cur_sum) return idx;
  }
  return (int)weights.size() - 1;
}
// Pdf for uniform discrete distribution sampling.
inline float sample_discrete_weights_pdf(
    const std::vector<float>& weights, int idx) {
  return weights[idx];
}

// Sample a discrete distribution represented by its cdf.
template <size_t N>
inline int sample_discrete_weights(
    const std::array<float, N>& weights, float r) {
  auto sum = 0.0f;
  for (auto weight : weights) sum += weight;
  r            = clamp(r * sum, (float)0, sum - (float)0.00001);
  auto cur_sum = 0.0f;
  for (auto idx = 0; idx < weights.size(); idx++) {
    cur_sum += weights[idx];
    if (r < cur_sum) return idx;
  }
  return (int)weights.size() - 1;
}
// Pdf for uniform discrete distribution sampling.
template <size_t N>
inline float sample_discrete_weights_pdf(
    const std::array<float, N>& weights, int idx) {
  return weights[idx];
}

}  // namespace yocto::math

// -----------------------------------------------------------------------------
// IMPLEMENTATION OF USER INTERFACE UTILITIES
// -----------------------------------------------------------------------------
namespace yocto::math {

// Computes the image uv coordinates corresponding to the view parameters.
// Returns negative coordinates if out of the image.
inline vec2i get_image_coords(const vec2f& mouse_pos, const vec2f& center,
    float scale, const vec2i& txt_size) {
  auto xyf = (mouse_pos - center) / scale;
  return vec2i{(int)round(xyf.x + txt_size.x / 2.0f),
      (int)round(xyf.y + txt_size.y / 2.0f)};
}

// Center image and autofit.
inline void update_imview(vec2f& center, float& scale, const vec2i& imsize,
    const vec2i& winsize, bool zoom_to_fit) {
  if (zoom_to_fit) {
    scale  = min(winsize.x / (float)imsize.x, winsize.y / (float)imsize.y);
    center = {(float)winsize.x / 2, (float)winsize.y / 2};
  } else {
    if (winsize.x >= imsize.x * scale) center.x = winsize.x / 2;
    if (winsize.y >= imsize.y * scale) center.y = winsize.y / 2;
  }
}

// Turntable for UI navigation.
inline void update_turntable(vec3f& from, vec3f& to, vec3f& up,
    const vec2f& rotate, float dolly, const vec2f& pan) {
  // rotate if necessary
  if (rotate.x || rotate.y) {
    auto z     = normalize(to - from);
    auto lz    = length(to - from);
    auto phi   = atan2(z.z, z.x) + rotate.x;
    auto theta = acos(z.y) + rotate.y;
    theta      = clamp(theta, 0.001f, pif - 0.001f);
    auto nz    = vec3f{sin(theta) * cos(phi) * lz, cos(theta) * lz,
        sin(theta) * sin(phi) * lz};
    from       = to - nz;
  }

  // dolly if necessary
  if (dolly) {
    auto z  = normalize(to - from);
    auto lz = max(0.001f, length(to - from) * (1 + dolly));
    z *= lz;
    from = to - z;
  }

  // pan if necessary
  if (pan.x || pan.y) {
    auto z = normalize(to - from);
    auto x = normalize(cross(up, z));
    auto y = normalize(cross(z, x));
    auto t = vec3f{pan.x * x.x + pan.y * y.x, pan.x * x.y + pan.y * y.y,
        pan.x * x.z + pan.y * y.z};
    from += t;
    to += t;
  }
}

// Turntable for UI navigation.
inline void update_turntable(frame3f& frame, float& focus, const vec2f& rotate,
    float dolly, const vec2f& pan) {
  // rotate if necessary
  if (rotate != zero2f) {
    auto phi   = atan2(frame.z.z, frame.z.x) + rotate.x;
    auto theta = acos(frame.z.y) + rotate.y;
    theta      = clamp(theta, 0.001f, pif - 0.001f);
    auto new_z = vec3f{
        sin(theta) * cos(phi), cos(theta), sin(theta) * sin(phi)};
    auto new_center = frame.o - frame.z * focus;
    auto new_o      = new_center + new_z * focus;
    frame           = lookat_frame(new_o, new_center, {0, 1, 0});
    focus           = length(new_o - new_center);
  }

  // pan if necessary
  if (dolly) {
    auto c  = frame.o - frame.z * focus;
    focus   = max(focus * (1 + dolly), 0.001f);
    frame.o = c + frame.z * focus;
  }

  // pan if necessary
  if (pan.x || pan.y) {
    frame.o += frame.x * pan.x + frame.y * pan.y;
  }
}

// FPS camera for UI navigation for a frame parametrization.
inline void update_fpscam(
    frame3f& frame, const vec3f& transl, const vec2f& rotate) {
  // https://gamedev.stackexchange.com/questions/30644/how-to-keep-my-quaternion-using-fps-camera-from-tilting-and-messing-up
  auto y = vec3f{0, 1, 0};
  auto z = orthonormalize(frame.z, y);
  auto x = cross(y, z);

  auto rot = rotation_frame(vec3f{1, 0, 0}, rotate.y) *
             frame3f{frame.x, frame.y, frame.z, vec3f{0, 0, 0}} *
             rotation_frame(vec3f{0, 1, 0}, rotate.x);
  auto pos = frame.o + transl.x * x + transl.y * y + transl.z * z;

  frame = {rot.x, rot.y, rot.z, pos};
}

// Generate a ray from a camera
inline ray3f camera_ray(const frame3f& frame, float lens, const vec2f& film,
    const vec2f& image_uv) {
  auto e = zero3f;
  auto q = vec3f{
      film.x * (0.5f - image_uv.x), film.y * (image_uv.y - 0.5f), lens};
  auto q1  = -q;
  auto d   = normalize(q1 - e);
  auto ray = ray3f{transform_point(frame, e), transform_direction(frame, d)};
  return ray;
}

}  // namespace yocto::math

#endif
