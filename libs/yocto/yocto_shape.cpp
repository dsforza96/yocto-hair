//
// Implementation for Yocto/Shape
//

// -----------------------------------------------------------------------------
// INCLUDES
// -----------------------------------------------------------------------------

#include "yocto_shape.h"

#include <atomic>
#include <deque>
#include <memory>
#include <string>
using namespace std::string_literals;

#include "yocto_obj.h"
#include "yocto_ply.h"

// -----------------------------------------------------------------------------
// ALIASES
// -----------------------------------------------------------------------------
namespace yocto::shape {

// Namespace aliases
namespace yply = yocto::ply;
namespace yobj = yocto::obj;

// import math symbols for use
using math::abs;
using math::acos;
using math::atan2;
using math::clamp;
using math::cos;
using math::exp;
using math::flt_max;
using math::fmod;
using math::invalidb3f;
using math::lerp;
using math::log;
using math::make_rng;
using math::max;
using math::min;
using math::pif;
using math::pow;
using math::rng_state;
using math::sample_discrete_cdf;
using math::sample_discrete_cdf_pdf;
using math::sample_uniform;
using math::sample_uniform_pdf;
using math::sin;
using math::sqrt;
using math::swap;
using math::zero2f;
using math::zero3f;
using math::zero4f;

}  // namespace yocto::shape

// -----------------------------------------------------------------------------
// IMPLEMENTATION OF COMPUTATION OF PER-VERTEX PROPETIES
// -----------------------------------------------------------------------------
namespace yocto::shape {

// Compute per-vertex tangents for lines.
std::vector<vec3f> compute_tangents(
    const std::vector<vec2i>& lines, const std::vector<vec3f>& positions) {
  auto tangents = std::vector<vec3f>{positions.size()};
  for (auto& tangent : tangents) tangent = zero3f;
  for (auto& l : lines) {
    auto tangent = line_tangent(positions[l.x], positions[l.y]);
    auto length  = line_length(positions[l.x], positions[l.y]);
    tangents[l.x] += tangent * length;
    tangents[l.y] += tangent * length;
  }
  for (auto& tangent : tangents) tangent = normalize(tangent);
  return tangents;
}

// Compute per-vertex normals for triangles.
std::vector<vec3f> compute_normals(
    const std::vector<vec3i>& triangles, const std::vector<vec3f>& positions) {
  auto normals = std::vector<vec3f>{positions.size()};
  for (auto& normal : normals) normal = zero3f;
  for (auto& t : triangles) {
    auto normal = triangle_normal(
        positions[t.x], positions[t.y], positions[t.z]);
    auto area = triangle_area(positions[t.x], positions[t.y], positions[t.z]);
    normals[t.x] += normal * area;
    normals[t.y] += normal * area;
    normals[t.z] += normal * area;
  }
  for (auto& normal : normals) normal = normalize(normal);
  return normals;
}

// Compute per-vertex normals for quads.
std::vector<vec3f> compute_normals(
    const std::vector<vec4i>& quads, const std::vector<vec3f>& positions) {
  auto normals = std::vector<vec3f>{positions.size()};
  for (auto& normal : normals) normal = zero3f;
  for (auto& q : quads) {
    auto normal = quad_normal(
        positions[q.x], positions[q.y], positions[q.z], positions[q.w]);
    auto area = quad_area(
        positions[q.x], positions[q.y], positions[q.z], positions[q.w]);
    normals[q.x] += normal * area;
    normals[q.y] += normal * area;
    normals[q.z] += normal * area;
    if (q.z != q.w) normals[q.w] += normal * area;
  }
  for (auto& normal : normals) normal = normalize(normal);
  return normals;
}

// Compute per-vertex tangents for lines.
void update_tangents(std::vector<vec3f>& tangents,
    const std::vector<vec2i>& lines, const std::vector<vec3f>& positions) {
  if (tangents.size() != positions.size()) {
    throw std::out_of_range("array should be the same length");
  }
  for (auto& tangent : tangents) tangent = zero3f;
  for (auto& l : lines) {
    auto tangent = line_tangent(positions[l.x], positions[l.y]);
    auto length  = line_length(positions[l.x], positions[l.y]);
    tangents[l.x] += tangent * length;
    tangents[l.y] += tangent * length;
  }
  for (auto& tangent : tangents) tangent = normalize(tangent);
}

// Compute per-vertex normals for triangles.
void update_normals(std::vector<vec3f>& normals,
    const std::vector<vec3i>& triangles, const std::vector<vec3f>& positions) {
  if (normals.size() != positions.size()) {
    throw std::out_of_range("array should be the same length");
  }
  for (auto& normal : normals) normal = zero3f;
  for (auto& t : triangles) {
    auto normal = triangle_normal(
        positions[t.x], positions[t.y], positions[t.z]);
    auto area = triangle_area(positions[t.x], positions[t.y], positions[t.z]);
    normals[t.x] += normal * area;
    normals[t.y] += normal * area;
    normals[t.z] += normal * area;
  }
  for (auto& normal : normals) normal = normalize(normal);
}

// Compute per-vertex normals for quads.
void update_normals(std::vector<vec3f>& normals,
    const std::vector<vec4i>& quads, const std::vector<vec3f>& positions) {
  if (normals.size() != positions.size()) {
    throw std::out_of_range("array should be the same length");
  }
  for (auto& normal : normals) normal = zero3f;
  for (auto& q : quads) {
    auto normal = quad_normal(
        positions[q.x], positions[q.y], positions[q.z], positions[q.w]);
    auto area = quad_area(
        positions[q.x], positions[q.y], positions[q.z], positions[q.w]);
    normals[q.x] += normal * area;
    normals[q.y] += normal * area;
    normals[q.z] += normal * area;
    if (q.z != q.w) normals[q.w] += normal * area;
  }
  for (auto& normal : normals) normal = normalize(normal);
}

// Compute per-vertex tangent frame for triangle meshes.
// Tangent space is defined by a four component std::vector.
// The first three components are the tangent with respect to the U texcoord.
// The fourth component is the sign of the tangent wrt the V texcoord.
// Tangent frame is useful in normal mapping.
std::vector<vec4f> compute_tangent_spaces(const std::vector<vec3i>& triangles,
    const std::vector<vec3f>& positions, const std::vector<vec3f>& normals,
    const std::vector<vec2f>& texcoords) {
  auto tangu = std::vector<vec3f>(positions.size(), zero3f);
  auto tangv = std::vector<vec3f>(positions.size(), zero3f);
  for (auto t : triangles) {
    auto tutv = triangle_tangents_fromuv(positions[t.x], positions[t.y],
        positions[t.z], texcoords[t.x], texcoords[t.y], texcoords[t.z]);
    for (auto vid : {t.x, t.y, t.z}) tangu[vid] += normalize(tutv.first);
    for (auto vid : {t.x, t.y, t.z}) tangv[vid] += normalize(tutv.second);
  }
  for (auto& t : tangu) t = normalize(t);
  for (auto& t : tangv) t = normalize(t);

  auto tangent_spaces = std::vector<vec4f>(positions.size());
  for (auto& tangent : tangent_spaces) tangent = zero4f;
  for (auto i = 0; i < positions.size(); i++) {
    tangu[i] = orthonormalize(tangu[i], normals[i]);
    auto s   = (dot(cross(normals[i], tangu[i]), tangv[i]) < 0) ? -1.0f : 1.0f;
    tangent_spaces[i] = {tangu[i].x, tangu[i].y, tangu[i].z, s};
  }
  return tangent_spaces;
}

// Apply skinning
std::pair<std::vector<vec3f>, std::vector<vec3f>> compute_skinning(
    const std::vector<vec3f>& positions, const std::vector<vec3f>& normals,
    const std::vector<vec4f>& weights, const std::vector<vec4i>& joints,
    const std::vector<frame3f>& xforms) {
  auto skinned_positions = std::vector<vec3f>{positions.size()};
  auto skinned_normals   = std::vector<vec3f>{positions.size()};
  for (auto i = 0; i < positions.size(); i++) {
    skinned_positions[i] =
        transform_point(xforms[joints[i].x], positions[i]) * weights[i].x +
        transform_point(xforms[joints[i].y], positions[i]) * weights[i].y +
        transform_point(xforms[joints[i].z], positions[i]) * weights[i].z +
        transform_point(xforms[joints[i].w], positions[i]) * weights[i].w;
  }
  for (auto i = 0; i < normals.size(); i++) {
    skinned_normals[i] = normalize(
        transform_direction(xforms[joints[i].x], normals[i]) * weights[i].x +
        transform_direction(xforms[joints[i].y], normals[i]) * weights[i].y +
        transform_direction(xforms[joints[i].z], normals[i]) * weights[i].z +
        transform_direction(xforms[joints[i].w], normals[i]) * weights[i].w);
  }
  return {skinned_positions, skinned_normals};
}

// Apply skinning as specified in Khronos glTF
std::pair<std::vector<vec3f>, std::vector<vec3f>> compute_matrix_skinning(
    const std::vector<vec3f>& positions, const std::vector<vec3f>& normals,
    const std::vector<vec4f>& weights, const std::vector<vec4i>& joints,
    const std::vector<mat4f>& xforms) {
  auto skinned_positions = std::vector<vec3f>{positions.size()};
  auto skinned_normals   = std::vector<vec3f>{positions.size()};
  for (auto i = 0; i < positions.size(); i++) {
    auto xform = xforms[joints[i].x] * weights[i].x +
                 xforms[joints[i].y] * weights[i].y +
                 xforms[joints[i].z] * weights[i].z +
                 xforms[joints[i].w] * weights[i].w;
    skinned_positions[i] = transform_point(xform, positions[i]);
    skinned_normals[i]   = normalize(transform_direction(xform, normals[i]));
  }
  return {skinned_positions, skinned_normals};
}

// Apply skinning
void update_skinning(std::vector<vec3f>& skinned_positions,
    std::vector<vec3f>& skinned_normals, const std::vector<vec3f>& positions,
    const std::vector<vec3f>& normals, const std::vector<vec4f>& weights,
    const std::vector<vec4i>& joints, const std::vector<frame3f>& xforms) {
  if (skinned_positions.size() != positions.size() ||
      skinned_normals.size() != normals.size()) {
    throw std::out_of_range("arrays should be the same size");
  }
  for (auto i = 0; i < positions.size(); i++) {
    skinned_positions[i] =
        transform_point(xforms[joints[i].x], positions[i]) * weights[i].x +
        transform_point(xforms[joints[i].y], positions[i]) * weights[i].y +
        transform_point(xforms[joints[i].z], positions[i]) * weights[i].z +
        transform_point(xforms[joints[i].w], positions[i]) * weights[i].w;
  }
  for (auto i = 0; i < normals.size(); i++) {
    skinned_normals[i] = normalize(
        transform_direction(xforms[joints[i].x], normals[i]) * weights[i].x +
        transform_direction(xforms[joints[i].y], normals[i]) * weights[i].y +
        transform_direction(xforms[joints[i].z], normals[i]) * weights[i].z +
        transform_direction(xforms[joints[i].w], normals[i]) * weights[i].w);
  }
}

// Apply skinning as specified in Khronos glTF
void update_matrix_skinning(std::vector<vec3f>& skinned_positions,
    std::vector<vec3f>& skinned_normals, const std::vector<vec3f>& positions,
    const std::vector<vec3f>& normals, const std::vector<vec4f>& weights,
    const std::vector<vec4i>& joints, const std::vector<mat4f>& xforms) {
  if (skinned_positions.size() != positions.size() ||
      skinned_normals.size() != normals.size()) {
    throw std::out_of_range("arrays should be the same size");
  }
  for (auto i = 0; i < positions.size(); i++) {
    auto xform = xforms[joints[i].x] * weights[i].x +
                 xforms[joints[i].y] * weights[i].y +
                 xforms[joints[i].z] * weights[i].z +
                 xforms[joints[i].w] * weights[i].w;
    skinned_positions[i] = transform_point(xform, positions[i]);
    skinned_normals[i]   = normalize(transform_direction(xform, normals[i]));
  }
}

}  // namespace yocto::shape

// -----------------------------------------------------------------------------
// COMPUTATION OF PER_VERTEX PROPETIES
// -----------------------------------------------------------------------------
namespace yocto::shape {

// Flip vertex normals
std::vector<vec3f> flip_normals(const std::vector<vec3f>& normals) {
  auto flipped = normals;
  for (auto& n : flipped) n = -n;
  return flipped;
}
// Flip face orientation
std::vector<vec3i> flip_triangles(const std::vector<vec3i>& triangles) {
  auto flipped = triangles;
  for (auto& t : flipped) swap(t.y, t.z);
  return flipped;
}
std::vector<vec4i> flip_quads(const std::vector<vec4i>& quads) {
  auto flipped = quads;
  for (auto& q : flipped) {
    if (q.z != q.w) {
      swap(q.y, q.w);
    } else {
      swap(q.y, q.z);
      q.w = q.z;
    }
  }
  return flipped;
}

// Align vertex positions. Alignment is 0: none, 1: min, 2: max, 3: center.
std::vector<vec3f> align_vertices(
    const std::vector<vec3f>& positions, const vec3i& alignment) {
  auto bounds = invalidb3f;
  for (auto& p : positions) bounds = merge(bounds, p);
  auto offset = vec3f{0, 0, 0};
  switch (alignment.x) {
    case 1: offset.x = bounds.min.x; break;
    case 2: offset.x = (bounds.min.x + bounds.max.x) / 2; break;
    case 3: offset.x = bounds.max.x; break;
  }
  switch (alignment.y) {
    case 1: offset.y = bounds.min.y; break;
    case 2: offset.y = (bounds.min.y + bounds.max.y) / 2; break;
    case 3: offset.y = bounds.max.y; break;
  }
  switch (alignment.z) {
    case 1: offset.z = bounds.min.z; break;
    case 2: offset.z = (bounds.min.z + bounds.max.z) / 2; break;
    case 3: offset.z = bounds.max.z; break;
  }
  auto aligned = positions;
  for (auto& p : aligned) p -= offset;
  return aligned;
}

}  // namespace yocto::shape

// -----------------------------------------------------------------------------
// EDGEA AND ADJACENCIES
// -----------------------------------------------------------------------------
namespace yocto::shape {

// Initialize an edge map with elements.
edge_map make_edge_map(const std::vector<vec3i>& triangles) {
  auto emap = edge_map{};
  for (auto& t : triangles) {
    insert_edge(emap, {t.x, t.y});
    insert_edge(emap, {t.y, t.z});
    insert_edge(emap, {t.z, t.x});
  }
  return emap;
}
edge_map make_edge_map(const std::vector<vec4i>& quads) {
  auto emap = edge_map{};
  for (auto& q : quads) {
    insert_edge(emap, {q.x, q.y});
    insert_edge(emap, {q.y, q.z});
    if (q.z != q.w) insert_edge(emap, {q.z, q.w});
    insert_edge(emap, {q.w, q.x});
  }
  return emap;
}
void insert_edges(edge_map& emap, const std::vector<vec3i>& triangles) {
  for (auto& t : triangles) {
    insert_edge(emap, {t.x, t.y});
    insert_edge(emap, {t.y, t.z});
    insert_edge(emap, {t.z, t.x});
  }
}
void insert_edges(edge_map& emap, const std::vector<vec4i>& quads) {
  for (auto& q : quads) {
    insert_edge(emap, {q.x, q.y});
    insert_edge(emap, {q.y, q.z});
    if (q.z != q.w) insert_edge(emap, {q.z, q.w});
    insert_edge(emap, {q.w, q.x});
  }
}
// Insert an edge and return its index
int insert_edge(edge_map& emap, const vec2i& edge) {
  auto es = edge.x < edge.y ? edge : vec2i{edge.y, edge.x};
  auto it = emap.index.find(es);
  if (it == emap.index.end()) {
    auto idx = (int)emap.edges.size();
    emap.index.insert(it, {es, idx});
    emap.edges.push_back(es);
    emap.nfaces.push_back(1);
    return idx;
  } else {
    auto idx = it->second;
    emap.nfaces[idx] += 1;
    return idx;
  }
}
// Get number of edges
int num_edges(const edge_map& emap) { return emap.edges.size(); }
// Get the edge index
int edge_index(const edge_map& emap, const vec2i& edge) {
  auto es       = edge.x < edge.y ? edge : vec2i{edge.y, edge.x};
  auto iterator = emap.index.find(es);
  if (iterator == emap.index.end()) return -1;
  return iterator->second;
}
// Get a list of edges, boundary edges, boundary vertices
std::vector<vec2i> get_edges(const edge_map& emap) { return emap.edges; }
std::vector<vec2i> get_boundary(const edge_map& emap) {
  auto boundary = std::vector<vec2i>{};
  for (auto idx = 0; idx < emap.edges.size(); idx++) {
    if (emap.nfaces[idx] < 2) boundary.push_back(emap.edges[idx]);
  }
  return boundary;
}
std::vector<vec2i> get_edges(const std::vector<vec3i>& triangles) {
  return get_edges(make_edge_map(triangles));
}
std::vector<vec2i> get_edges(const std::vector<vec4i>& quads) {
  return get_edges(make_edge_map(quads));
}

// Build adjacencies between faces (sorted counter-clockwise)
std::vector<vec3i> face_adjacencies(const std::vector<vec3i>& triangles) {
  auto get_edge = [](const vec3i& triangle, int i) -> vec2i {
    auto x = triangle[i], y = triangle[i < 2 ? i + 1 : 0];
    return x < y ? vec2i{x, y} : vec2i{y, x};
  };
  auto adjacencies = std::vector<vec3i>{triangles.size(), vec3i{-1, -1, -1}};
  auto edge_map    = std::unordered_map<vec2i, int>();
  edge_map.reserve((size_t)(triangles.size() * 1.5));
  for (int i = 0; i < triangles.size(); ++i) {
    for (int k = 0; k < 3; ++k) {
      auto edge = get_edge(triangles[i], k);
      auto it   = edge_map.find(edge);
      if (it == edge_map.end()) {
        edge_map.insert(it, {edge, i});
      } else {
        auto neighbor     = it->second;
        adjacencies[i][k] = neighbor;
        for (int kk = 0; kk < 3; ++kk) {
          auto edge2 = get_edge(triangles[neighbor], kk);
          if (edge2 == edge) {
            adjacencies[neighbor][kk] = i;
            break;
          }
        }
      }
    }
  }
  return adjacencies;
}

// Build adjacencies between vertices (sorted counter-clockwise)
std::vector<std::vector<int>> vertex_adjacencies(
    const std::vector<vec3i>& triangles,
    const std::vector<vec3i>& adjacencies) {
  auto find_index = [](const vec3i& v, int x) {
    if (v.x == x) return 0;
    if (v.y == x) return 1;
    if (v.z == x) return 2;
    return -1;
  };

  // For each vertex, find any adjacent face.
  auto num_vertices     = 0;
  auto face_from_vertex = std::vector<int>(triangles.size() * 3, -1);

  for (int i = 0; i < triangles.size(); ++i) {
    for (int k = 0; k < 3; k++) {
      face_from_vertex[triangles[i][k]] = i;
      num_vertices                      = max(num_vertices, triangles[i][k]);
    }
  }

  // Init result.
  auto result = std::vector<std::vector<int>>(num_vertices);

  // For each vertex, loop around it and build its adjacency.
  for (int i = 0; i < num_vertices; ++i) {
    result[i].reserve(6);
    auto first_face = face_from_vertex[i];
    if (first_face == -1) continue;

    auto face = first_face;
    while (true) {
      auto k = find_index(triangles[face], i);
      k      = k != 0 ? k - 1 : 2;
      result[i].push_back(triangles[face][k]);
      face = adjacencies[face][k];
      if (face == -1) break;
      if (face == first_face) break;
    }
  }

  return result;
}

// Build adjacencies between each vertex and its adjacent faces.
// Adjacencies are sorted counter-clockwise and have same starting points as
// vertex_adjacencies()
std::vector<std::vector<int>> vertex_to_faces_adjacencies(
    const std::vector<vec3i>& triangles,
    const std::vector<vec3i>& adjacencies) {
  auto find_index = [](const vec3i& v, int x) {
    if (v.x == x) return 0;
    if (v.y == x) return 1;
    if (v.z == x) return 2;
    return -1;
  };

  // For each vertex, find any adjacent face.
  auto num_vertices     = 0;
  auto face_from_vertex = std::vector<int>(triangles.size() * 3, -1);

  for (int i = 0; i < triangles.size(); ++i) {
    for (int k = 0; k < 3; k++) {
      face_from_vertex[triangles[i][k]] = i;
      num_vertices                      = max(num_vertices, triangles[i][k]);
    }
  }

  // Init result.
  auto result = std::vector<std::vector<int>>(num_vertices);

  // For each vertex, loop around it and build its adjacency.
  for (int i = 0; i < num_vertices; ++i) {
    result[i].reserve(6);
    auto first_face = face_from_vertex[i];
    if (first_face == -1) continue;

    auto face = first_face;
    while (true) {
      auto k = find_index(triangles[face], i);
      k      = k != 0 ? k - 1 : 2;
      face   = adjacencies[face][k];
      result[i].push_back(face);
      if (face == -1) break;
      if (face == first_face) break;
    }
  }

  return result;
}

// Compute boundaries as a list of loops (sorted counter-clockwise)
std::vector<std::vector<int>> ordered_boundaries(
    const std::vector<vec3i>& triangles, const std::vector<vec3i>& adjacency,
    int num_vertices) {
  // map every boundary vertex to its next one
  auto next_vert = std::vector<int>(num_vertices, -1);
  for (int i = 0; i < triangles.size(); ++i) {
    for (int k = 0; k < 3; ++k) {
      if (adjacency[i][k] == -1)
        next_vert[triangles[i][k]] = triangles[i][(k + 1) % 3];
    }
  }

  // result
  auto boundaries = std::vector<std::vector<int>>();

  // arrange boundary vertices in loops
  for (int i = 0; i < next_vert.size(); i++) {
    if (next_vert[i] == -1) continue;

    // add new empty boundary
    boundaries.push_back({});
    auto current = i;

    while (true) {
      auto next = next_vert[current];
      if (next == -1) {
        return {};
      }
      next_vert[current] = -1;
      boundaries.back().push_back(current);

      // close loop if necessary
      if (next == i)
        break;
      else
        current = next;
    }
  }

  return boundaries;
}

}  // namespace yocto::shape

#ifdef YOCTO_EMBREE
#include <embree3/rtcore.h>
#endif

// -----------------------------------------------------------------------------
// IMPLEMENTATION FOR EMBREE BVH
// -----------------------------------------------------------------------------
namespace yocto::shape {

#ifdef YOCTO_EMBREE
// Get Embree device
std::atomic<ssize_t> bvh_embree_memory = 0;
static RTCDevice     bvh_embree_device() {
  static RTCDevice device = nullptr;
  if (!device) {
    device = rtcNewDevice("");
    rtcSetDeviceErrorFunction(
        device,
        [](void* ctx, RTCError code, const char* str) {
          switch (code) {
            case RTC_ERROR_UNKNOWN:
              throw std::runtime_error("RTC_ERROR_UNKNOWN: "s + str);
            case RTC_ERROR_INVALID_ARGUMENT:
              throw std::runtime_error("RTC_ERROR_INVALID_ARGUMENT: "s + str);
            case RTC_ERROR_INVALID_OPERATION:
              throw std::runtime_error("RTC_ERROR_INVALID_OPERATION: "s + str);
            case RTC_ERROR_OUT_OF_MEMORY:
              throw std::runtime_error("RTC_ERROR_OUT_OF_MEMORY: "s + str);
            case RTC_ERROR_UNSUPPORTED_CPU:
              throw std::runtime_error("RTC_ERROR_UNSUPPORTED_CPU: "s + str);
            case RTC_ERROR_CANCELLED:
              throw std::runtime_error("RTC_ERROR_CANCELLED: "s + str);
            default: throw std::runtime_error("invalid error code");
          }
        },
        nullptr);
    rtcSetDeviceMemoryMonitorFunction(
        device,
        [](void* userPtr, ssize_t bytes, bool post) {
          bvh_embree_memory += bytes;
          return true;
        },
        nullptr);
  }
  return device;
}

// Initialize Embree BVH
void init_shape_embree_bvh(bvh_shape& shape) {
  auto edevice = bvh_embree_device();
  auto escene  = rtcNewScene(edevice);
  if (!shape.points.empty()) {
    throw std::runtime_error("embree does not support points");
  } else if (!shape.lines.empty()) {
    auto elines     = std::vector<int>{};
    auto epositions = std::vector<vec4f>{};
    auto last_index = -1;
    for (auto& l : shape.lines) {
      if (last_index == l.x) {
        elines.push_back((int)epositions.size() - 1);
        epositions.push_back({shape.positions[l.y], shape.radius[l.y]});
      } else {
        elines.push_back((int)epositions.size());
        epositions.push_back({shape.positions[l.x], shape.radius[l.x]});
        epositions.push_back({shape.positions[l.y], shape.radius[l.y]});
      }
      last_index = l.y;
    }
    auto egeometry = rtcNewGeometry(
        edevice, RTC_GEOMETRY_TYPE_FLAT_LINEAR_CURVE);
    rtcSetGeometryVertexAttributeCount(egeometry, 1);
    auto embree_positions = rtcSetNewGeometryBuffer(egeometry,
        RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT4, 4 * 4, epositions.size());
    auto embree_lines     = rtcSetNewGeometryBuffer(
        egeometry, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT, 4, elines.size());
    memcpy(embree_positions, epositions.data(), epositions.size() * 16);
    memcpy(embree_lines, elines.data(), elines.size() * 4);
    rtcCommitGeometry(egeometry);
    rtcAttachGeometryByID(escene, egeometry, 0);
  } else if (!shape.triangles.empty()) {
    auto egeometry = rtcNewGeometry(edevice, RTC_GEOMETRY_TYPE_TRIANGLE);
    rtcSetGeometryVertexAttributeCount(egeometry, 1);
    auto embree_positions = rtcSetNewGeometryBuffer(egeometry,
        RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, 3 * 4,
        shape.positions.size());
    auto embree_triangles = rtcSetNewGeometryBuffer(egeometry,
        RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, 3 * 4,
        shape.triangles.size());
    memcpy(
        embree_positions, shape.positions.data(), shape.positions.size() * 12);
    memcpy(
        embree_triangles, shape.triangles.data(), shape.triangles.size() * 12);
    rtcCommitGeometry(egeometry);
    rtcAttachGeometryByID(escene, egeometry, 0);
  } else if (!shape.quads.empty()) {
    auto egeometry = rtcNewGeometry(edevice, RTC_GEOMETRY_TYPE_QUAD);
    rtcSetGeometryVertexAttributeCount(egeometry, 1);
    auto embree_positions = rtcSetNewGeometryBuffer(egeometry,
        RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, 3 * 4,
        shape.positions.size());
    auto embree_quads     = rtcSetNewGeometryBuffer(egeometry,
        RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT4, 4 * 4, shape.quads.size());
    memcpy(
        embree_positions, shape.positions.data(), shape.positions.size() * 12);
    memcpy(embree_quads, shape.quads.data(), shape.quads.size() * 16);
    rtcCommitGeometry(egeometry);
    rtcAttachGeometryByID(escene, egeometry, 0);
  } else {
    throw std::runtime_error("empty shapes not supported");
  }
  rtcCommitScene(escene);
  shape.embree_bvh = std::shared_ptr<void>{
      escene, [](void* ptr) { rtcReleaseScene((RTCScene)ptr); }};
}
void init_scene_embree_bvh(bvh_scene& scene) {
  // scene bvh
  auto edevice = bvh_embree_device();
  auto escene  = rtcNewScene(edevice);
  for (auto instance_id = 0; instance_id < scene.instances.size();
       instance_id++) {
    auto& instance  = scene.instances[instance_id];
    auto& shape     = scene.shapes[instance.shape];
    auto  egeometry = rtcNewGeometry(edevice, RTC_GEOMETRY_TYPE_INSTANCE);
    rtcSetGeometryInstancedScene(egeometry, (RTCScene)shape.embree_bvh.get());
    rtcSetGeometryTransform(
        egeometry, 0, RTC_FORMAT_FLOAT3X4_COLUMN_MAJOR, &instance.frame);
    rtcCommitGeometry(egeometry);
    rtcAttachGeometryByID(escene, egeometry, instance_id);
  }
  rtcCommitScene(escene);
  scene.embree_bvh = std::shared_ptr<void>{
      escene, [](void* ptr) { rtcReleaseScene((RTCScene)ptr); }};
}

void update_scene_embree_bvh(
    bvh_scene& scene, const std::vector<int>& updated_instances) {
  // scene bvh
  auto escene = (RTCScene)scene.embree_bvh.get();
  for (auto instance_id : updated_instances) {
    auto& instance    = scene.instances[instance_id];
    auto& shape       = scene.shapes[instance.shape];
    auto  embree_geom = rtcGetGeometry(escene, instance_id);
    rtcSetGeometryInstancedScene(embree_geom, (RTCScene)shape.embree_bvh.get());
    rtcSetGeometryTransform(
        embree_geom, 0, RTC_FORMAT_FLOAT3X4_COLUMN_MAJOR, &instance.frame);
    rtcCommitGeometry(embree_geom);
  }
  rtcCommitScene(escene);
}

bool intersect_shape_embree_bvh(const bvh_shape& shape, const ray3f& ray,
    int& element, vec2f& uv, float& distance, bool find_any) {
  RTCRayHit embree_ray;
  embree_ray.ray.org_x     = ray.o.x;
  embree_ray.ray.org_y     = ray.o.y;
  embree_ray.ray.org_z     = ray.o.z;
  embree_ray.ray.dir_x     = ray.d.x;
  embree_ray.ray.dir_y     = ray.d.y;
  embree_ray.ray.dir_z     = ray.d.z;
  embree_ray.ray.tnear     = ray.tmin;
  embree_ray.ray.tfar      = ray.tmax;
  embree_ray.ray.flags     = 0;
  embree_ray.hit.geomID    = RTC_INVALID_GEOMETRY_ID;
  embree_ray.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
  RTCIntersectContext embree_ctx;
  rtcInitIntersectContext(&embree_ctx);
  rtcIntersect1((RTCScene)shape.embree_bvh.get(), &embree_ctx, &embree_ray);
  if (embree_ray.hit.geomID == RTC_INVALID_GEOMETRY_ID) return false;
  element  = (int)embree_ray.hit.primID;
  uv       = {embree_ray.hit.u, embree_ray.hit.v};
  distance = embree_ray.ray.tfar;
  return true;
}
bool intersect_scene_embree_bvh(const bvh_scene& scene, const ray3f& ray,
    int& instance, int& element, vec2f& uv, float& distance, bool find_any) {
  RTCRayHit embree_ray;
  embree_ray.ray.org_x     = ray.o.x;
  embree_ray.ray.org_y     = ray.o.y;
  embree_ray.ray.org_z     = ray.o.z;
  embree_ray.ray.dir_x     = ray.d.x;
  embree_ray.ray.dir_y     = ray.d.y;
  embree_ray.ray.dir_z     = ray.d.z;
  embree_ray.ray.tnear     = ray.tmin;
  embree_ray.ray.tfar      = ray.tmax;
  embree_ray.ray.flags     = 0;
  embree_ray.hit.geomID    = RTC_INVALID_GEOMETRY_ID;
  embree_ray.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
  RTCIntersectContext embree_ctx;
  rtcInitIntersectContext(&embree_ctx);
  rtcIntersect1((RTCScene)scene.embree_bvh.get(), &embree_ctx, &embree_ray);
  if (embree_ray.hit.geomID == RTC_INVALID_GEOMETRY_ID) return false;
  instance = (int)embree_ray.hit.instID[0];
  element  = (int)embree_ray.hit.primID;
  uv       = {embree_ray.hit.u, embree_ray.hit.v};
  distance = embree_ray.ray.tfar;
  return true;
}
#endif

}  // namespace yocto::shape

// -----------------------------------------------------------------------------
// IMPLEMENTATION FOR BVH
// -----------------------------------------------------------------------------
namespace yocto::shape {

#if !defined(_WIN32) && !defined(_WIN64)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic ignored "-Wunused-variable"
#ifndef __clang__
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#endif
#endif

// Splits a BVH node using the SAH heuristic. Returns split position and axis.
static std::pair<int, int> split_sah(std::vector<int>& primitives,
    const std::vector<bbox3f>& bboxes, const std::vector<vec3f>& centers,
    int start, int end) {
  // initialize split axis and position
  auto split_axis = 0;
  auto mid        = (start + end) / 2;

  // compute primintive bounds and size
  auto cbbox = invalidb3f;
  for (auto i = start; i < end; i++)
    cbbox = merge(cbbox, centers[primitives[i]]);
  auto csize = cbbox.max - cbbox.min;
  if (csize == zero3f) return {mid, split_axis};

  // consider N bins, compute their cost and keep the minimum
  const int nbins    = 16;
  auto      middle   = 0.0f;
  auto      min_cost = flt_max;
  auto      area     = [](auto& b) {
    auto size = b.max - b.min;
    return 1e-12f + 2 * size.x * size.y + 2 * size.x * size.z +
           2 * size.y * size.z;
  };
  for (auto saxis = 0; saxis < 3; saxis++) {
    for (auto b = 1; b < nbins; b++) {
      auto split     = cbbox.min[saxis] + b * csize[saxis] / nbins;
      auto left_bbox = invalidb3f, right_bbox = invalidb3f;
      auto left_nprims = 0, right_nprims = 0;
      for (auto i = start; i < end; i++) {
        if (centers[primitives[i]][saxis] < split) {
          left_bbox = merge(left_bbox, bboxes[primitives[i]]);
          left_nprims += 1;
        } else {
          right_bbox = merge(right_bbox, bboxes[primitives[i]]);
          right_nprims += 1;
        }
      }
      auto cost = 1 + left_nprims * area(left_bbox) / area(cbbox) +
                  right_nprims * area(right_bbox) / area(cbbox);
      if (cost < min_cost) {
        min_cost   = cost;
        middle     = split;
        split_axis = saxis;
      }
    }
  }
  // split
  mid = (int)(std::partition(primitives.data() + start, primitives.data() + end,
                  [split_axis, middle, &centers](
                      auto a) { return centers[a][split_axis] < middle; }) -
              primitives.data());

  // if we were not able to split, just break the primitives in half
  if (mid == start || mid == end) {
    throw std::runtime_error("bad bvh split");
    split_axis = 0;
    mid        = (start + end) / 2;
  }

  return {mid, split_axis};
}

// Splits a BVH node using the balance heuristic. Returns split position and
// axis.
static std::pair<int, int> split_balanced(std::vector<int>& primitives,
    const std::vector<bbox3f>& bboxes, const std::vector<vec3f>& centers,
    int start, int end) {
  // initialize split axis and position
  auto axis = 0;
  auto mid  = (start + end) / 2;

  // compute primintive bounds and size
  auto cbbox = invalidb3f;
  for (auto i = start; i < end; i++)
    cbbox = merge(cbbox, centers[primitives[i]]);
  auto csize = cbbox.max - cbbox.min;
  if (csize == zero3f) return {mid, axis};

  // split along largest
  if (csize.x >= csize.y && csize.x >= csize.z) axis = 0;
  if (csize.y >= csize.x && csize.y >= csize.z) axis = 1;
  if (csize.z >= csize.x && csize.z >= csize.y) axis = 2;

  // balanced tree split: find the largest axis of the
  // bounding box and split along this one right in the middle
  mid = (start + end) / 2;
  std::nth_element(primitives.data() + start, primitives.data() + mid,
      primitives.data() + end, [axis, &centers](auto a, auto b) {
        return centers[a][axis] < centers[b][axis];
      });

  // if we were not able to split, just break the primitives in half
  if (mid == start || mid == end) {
    throw std::runtime_error("bad bvh split");
    axis = 0;
    mid  = (start + end) / 2;
  }

  return {mid, axis};
}

// Splits a BVH node using the middle heutirtic. Returns split position and
// axis.
static std::pair<int, int> split_middle(std::vector<int>& primitives,
    const std::vector<bbox3f>& bboxes, const std::vector<vec3f>& centers,
    int start, int end) {
  // initialize split axis and position
  auto axis = 0;
  auto mid  = (start + end) / 2;

  // compute primintive bounds and size
  auto cbbox = invalidb3f;
  for (auto i = start; i < end; i++)
    cbbox = merge(cbbox, centers[primitives[i]]);
  auto csize = cbbox.max - cbbox.min;
  if (csize == zero3f) return {mid, axis};

  // split along largest
  if (csize.x >= csize.y && csize.x >= csize.z) axis = 0;
  if (csize.y >= csize.x && csize.y >= csize.z) axis = 1;
  if (csize.z >= csize.x && csize.z >= csize.y) axis = 2;

  // split the space in the middle along the largest axis
  auto cmiddle = (cbbox.max + cbbox.min) / 2;
  auto middle  = cmiddle[axis];
  mid = (int)(std::partition(primitives.data() + start, primitives.data() + end,
                  [axis, middle, &centers](
                      auto a) { return centers[a][axis] < middle; }) -
              primitives.data());

  // if we were not able to split, just break the primitives in half
  if (mid == start || mid == end) {
    throw std::runtime_error("bad bvh split");
    axis = 0;
    mid  = (start + end) / 2;
  }

  return {mid, axis};
}

#if !defined(_WIN32) && !defined(_WIN64)
#pragma GCC diagnostic pop
#endif

// Build BVH nodes
static void build_bvh(bvh_tree& bvh, std::vector<bbox3f>& bboxes) {
  // get values
  auto& nodes      = bvh.nodes;
  auto& primitives = bvh.primitives;

  // prepare to build nodes
  nodes.clear();
  nodes.reserve(bboxes.size() * 2);

  // prepare primitives
  bvh.primitives.resize(bboxes.size());
  for (auto idx = 0; idx < bboxes.size(); idx++) bvh.primitives[idx] = idx;

  // prepare centers
  auto centers = std::vector<vec3f>(bboxes.size());
  for (auto idx = 0; idx < bboxes.size(); idx++)
    centers[idx] = center(bboxes[idx]);

  // queue up first node
  auto queue = std::deque<vec3i>{{0, 0, (int)bboxes.size()}};
  nodes.emplace_back();

  // create nodes until the queue is empty
  while (!queue.empty()) {
    // grab node to work on
    auto next = queue.front();
    queue.pop_front();
    auto nodeid = next.x, start = next.y, end = next.z;

    // grab node
    auto& node = nodes[nodeid];

    // compute bounds
    node.bbox = invalidb3f;
    for (auto i = start; i < end; i++)
      node.bbox = merge(node.bbox, bboxes[primitives[i]]);

    // split into two children
    if (end - start > bvh_max_prims) {
      // get split
      auto [mid, axis] = split_middle(primitives, bboxes, centers, start, end);

      // make an internal node
      node.internal = true;
      node.axis     = axis;
      node.num      = 2;
      node.start    = (int)nodes.size();
      nodes.emplace_back();
      nodes.emplace_back();
      queue.push_back({node.start + 0, start, mid});
      queue.push_back({node.start + 1, mid, end});
    } else {
      // Make a leaf node
      node.internal = false;
      node.num      = end - start;
      node.start    = start;
    }
  }

  // cleanup
  nodes.shrink_to_fit();
}

// Update bvh
static void update_bvh(bvh_tree& bvh, const std::vector<bbox3f>& bboxes) {
  for (auto nodeid = (int)bvh.nodes.size() - 1; nodeid >= 0; nodeid--) {
    auto& node = bvh.nodes[nodeid];
    node.bbox  = invalidb3f;
    if (node.internal) {
      for (auto idx = 0; idx < 2; idx++) {
        node.bbox = merge(node.bbox, bvh.nodes[node.start + idx].bbox);
      }
    } else {
      for (auto idx = 0; idx < node.num; idx++) {
        node.bbox = merge(node.bbox, bboxes[bvh.primitives[node.start + idx]]);
      }
    }
  }
}

// Build shape bvh
void make_points_bvh(bvh_tree& bvh, const std::vector<int>& points,
    const std::vector<vec3f>& positions, const std::vector<float>& radius) {
  // build primitives
  auto bboxes = std::vector<bbox3f>(points.size());
  for (auto idx = 0; idx < bboxes.size(); idx++) {
    auto& p     = points[idx];
    bboxes[idx] = point_bounds(positions[p], radius[p]);
  }

  // build nodes
  build_bvh(bvh, bboxes);
}
void make_lines_bvh(bvh_tree& bvh, const std::vector<vec2i>& lines,
    const std::vector<vec3f>& positions, const std::vector<float>& radius) {
  // build primitives
  auto bboxes = std::vector<bbox3f>(lines.size());
  for (auto idx = 0; idx < bboxes.size(); idx++) {
    auto& l     = lines[idx];
    bboxes[idx] = line_bounds(
        positions[l.x], positions[l.y], radius[l.x], radius[l.y]);
  }

  // build nodes
  build_bvh(bvh, bboxes);
}
void make_triangles_bvh(bvh_tree& bvh, const std::vector<vec3i>& triangles,
    const std::vector<vec3f>& positions, const std::vector<float>& radius) {
  // build primitives
  auto bboxes = std::vector<bbox3f>(triangles.size());
  for (auto idx = 0; idx < bboxes.size(); idx++) {
    auto& t     = triangles[idx];
    bboxes[idx] = triangle_bounds(
        positions[t.x], positions[t.y], positions[t.z]);
  }

  // build nodes
  build_bvh(bvh, bboxes);
}
void make_quads_bvh(bvh_tree& bvh, const std::vector<vec4i>& quads,
    const std::vector<vec3f>& positions, const std::vector<float>& radius) {
  // build primitives
  auto bboxes = std::vector<bbox3f>(quads.size());
  for (auto idx = 0; idx < bboxes.size(); idx++) {
    auto& q     = quads[idx];
    bboxes[idx] = quad_bounds(
        positions[q.x], positions[q.y], positions[q.z], positions[q.w]);
  }

  // build nodes
  build_bvh(bvh, bboxes);
}

void update_points_bvh(bvh_tree& bvh, const std::vector<int>& points,
    const std::vector<vec3f>& positions, const std::vector<float>& radius) {
  // build primitives
  auto bboxes = std::vector<bbox3f>(points.size());
  for (auto idx = 0; idx < bboxes.size(); idx++) {
    auto& p     = points[idx];
    bboxes[idx] = point_bounds(positions[p], radius[p]);
  }

  // update nodes
  update_bvh(bvh, bboxes);
}
void update_lines_bvh(bvh_tree& bvh, const std::vector<vec2i>& lines,
    const std::vector<vec3f>& positions, const std::vector<float>& radius) {
  // build primitives
  auto bboxes = std::vector<bbox3f>(lines.size());
  for (auto idx = 0; idx < bboxes.size(); idx++) {
    auto& l     = lines[idx];
    bboxes[idx] = line_bounds(
        positions[l.x], positions[l.y], radius[l.x], radius[l.y]);
  }

  // update nodes
  update_bvh(bvh, bboxes);
}
void update_triangles_bvh(bvh_tree& bvh, const std::vector<vec3i>& triangles,
    const std::vector<vec3f>& positions) {
  // build primitives
  auto bboxes = std::vector<bbox3f>(triangles.size());
  for (auto idx = 0; idx < bboxes.size(); idx++) {
    auto& t     = triangles[idx];
    bboxes[idx] = triangle_bounds(
        positions[t.x], positions[t.y], positions[t.z]);
  }

  // update nodes
  update_bvh(bvh, bboxes);
}
void update_quads_bvh(bvh_tree& bvh, const std::vector<vec4i>& quads,
    const std::vector<vec3f>& positions) {
  // build primitives
  auto bboxes = std::vector<bbox3f>(quads.size());
  for (auto idx = 0; idx < bboxes.size(); idx++) {
    auto& q     = quads[idx];
    bboxes[idx] = quad_bounds(
        positions[q.x], positions[q.y], positions[q.z], positions[q.w]);
  }

  // update nodes
  update_bvh(bvh, bboxes);
}

// Intersect ray with a bvh.
template <typename Intersect>
static bool intersect_elements_bvh(const bvh_tree& bvh,
    Intersect&& intersect_element, const ray3f& ray_, int& element, vec2f& uv,
    float& distance, bool find_any) {
  // check empty
  if (bvh.nodes.empty()) return false;

  // node stack
  int  node_stack[128];
  auto node_cur          = 0;
  node_stack[node_cur++] = 0;

  // shared variables
  auto hit = false;

  // copy ray to modify it
  auto ray = ray_;

  // prepare ray for fast queries
  auto ray_dinv  = vec3f{1 / ray.d.x, 1 / ray.d.y, 1 / ray.d.z};
  auto ray_dsign = vec3i{(ray_dinv.x < 0) ? 1 : 0, (ray_dinv.y < 0) ? 1 : 0,
      (ray_dinv.z < 0) ? 1 : 0};

  // walking stack
  while (node_cur) {
    // grab node
    auto& node = bvh.nodes[node_stack[--node_cur]];

    // intersect bbox
    // if (!intersect_bbox(ray, ray_dinv, ray_dsign, node.bbox)) continue;
    if (!intersect_bbox(ray, ray_dinv, node.bbox)) continue;

    // intersect node, switching based on node type
    // for each type, iterate over the the primitive list
    if (node.internal) {
      // for internal nodes, attempts to proceed along the
      // split axis from smallest to largest nodes
      if (ray_dsign[node.axis]) {
        node_stack[node_cur++] = node.start + 0;
        node_stack[node_cur++] = node.start + 1;
      } else {
        node_stack[node_cur++] = node.start + 1;
        node_stack[node_cur++] = node.start + 0;
      }
    } else {
      for (auto idx = 0; idx < node.num; idx++) {
        auto primitive = bvh.primitives[node.start + idx];
        if (intersect_element(primitive, ray, uv, distance)) {
          hit      = true;
          element  = primitive;
          ray.tmax = distance;
        }
      }
    }

    // check for early exit
    if (find_any && hit) return hit;
  }

  return hit;
}

// Intersect ray with a bvh.
bvh_intersection intersect_points_bvh(const bvh_tree& bvh,
    const std::vector<int>& points, const std::vector<vec3f>& positions,
    const std::vector<float>& radius, const ray3f& ray, bool find_any) {
  auto intersection = bvh_intersection{};
  intersection.hit  = intersect_elements_bvh(
      bvh,
      [&points, &positions, &radius](
          int idx, const ray3f& ray, vec2f& uv, float& distance) {
        auto& p = points[idx];
        return intersect_point(ray, positions[p], radius[p], uv, distance);
      },
      ray, intersection.element, intersection.uv, intersection.distance,
      find_any);
  return intersection;
}
bvh_intersection intersect_lines_bvh(const bvh_tree& bvh,
    const std::vector<vec2i>& lines, const std::vector<vec3f>& positions,
    const std::vector<float>& radius, const ray3f& ray, bool find_any) {
  auto intersection = bvh_intersection{};
  intersection.hit  = intersect_elements_bvh(
      bvh,
      [&lines, &positions, &radius](
          int idx, const ray3f& ray, vec2f& uv, float& distance) {
        auto& l = lines[idx];
        return intersect_line(ray, positions[l.x], positions[l.y], radius[l.x],
            radius[l.y], uv, distance);
      },
      ray, intersection.element, intersection.uv, intersection.distance,
      find_any);
  return intersection;
}
bvh_intersection intersect_triangles_bvh(const bvh_tree& bvh,
    const std::vector<vec3i>& triangles, const std::vector<vec3f>& positions,
    const ray3f& ray, bool find_any) {
  auto intersection = bvh_intersection{};
  intersection.hit  = intersect_elements_bvh(
      bvh,
      [&triangles, &positions](
          int idx, const ray3f& ray, vec2f& uv, float& distance) {
        auto& t = triangles[idx];
        return intersect_triangle(
            ray, positions[t.x], positions[t.y], positions[t.z], uv, distance);
      },
      ray, intersection.element, intersection.uv, intersection.distance,
      find_any);
  return intersection;
}
bvh_intersection intersect_quads_bvh(const bvh_tree& bvh,
    const std::vector<vec4i>& quads, const std::vector<vec3f>& positions,
    const ray3f& ray, bool find_any) {
  auto intersection = bvh_intersection{};
  intersection.hit  = intersect_elements_bvh(
      bvh,
      [&quads, &positions](
          int idx, const ray3f& ray, vec2f& uv, float& distance) {
        auto& t = quads[idx];
        return intersect_quad(ray, positions[t.x], positions[t.y],
            positions[t.z], positions[t.w], uv, distance);
      },
      ray, intersection.element, intersection.uv, intersection.distance,
      find_any);
  return intersection;
}

// Intersect ray with a bvh.
template <typename Overlap>
static bool overlap_elements_bvh(const bvh_tree& bvh, Overlap&& overlap_element,
    const vec3f& pos, float max_distance, int& element, vec2f& uv,
    float& distance, bool find_any) {
  // check if empty
  if (bvh.nodes.empty()) return false;

  // node stack
  int  node_stack[64];
  auto node_cur          = 0;
  node_stack[node_cur++] = 0;

  // hit
  auto hit = false;

  // walking stack
  while (node_cur) {
    // grab node
    auto& node = bvh.nodes[node_stack[--node_cur]];

    // intersect bbox
    if (!distance_check_bbox(pos, max_distance, node.bbox)) continue;

    // intersect node, switching based on node type
    // for each type, iterate over the the primitive list
    if (node.internal) {
      // internal node
      node_stack[node_cur++] = node.start + 0;
      node_stack[node_cur++] = node.start + 1;
    } else {
      for (auto idx = 0; idx < node.num; idx++) {
        auto primitive = bvh.primitives[node.start + idx];
        if (overlap_element(primitive, pos, max_distance, uv, distance)) {
          hit          = true;
          element      = primitive;
          max_distance = distance;
        }
      }
    }

    // check for early exit
    if (find_any && hit) return hit;
  }

  return hit;
}

// Find a shape element that overlaps a point within a given distance
// max distance, returning either the closest or any overlap depending on
// `find_any`. Returns the point distance, the instance id, the shape element
// index and the element barycentric coordinates.
bvh_intersection overlap_points_bvh(const bvh_tree& bvh,
    const std::vector<int>& points, const std::vector<vec3f>& positions,
    const std::vector<float>& radius, const vec3f& pos, float max_distance,
    bool find_any) {
  auto intersection = bvh_intersection{};
  intersection.hit  = overlap_elements_bvh(
      bvh,
      [&points, &positions, &radius](int idx, const vec3f& pos,
          float max_distance, vec2f& uv, float& distance) {
        auto& p = points[idx];
        return overlap_point(
            pos, max_distance, positions[p], radius[p], uv, distance);
      },
      pos, max_distance, intersection.element, intersection.uv,
      intersection.distance, find_any);
  return intersection;
}
bvh_intersection overlap_lines_bvh(const bvh_tree& bvh,
    const std::vector<vec2i>& lines, const std::vector<vec3f>& positions,
    const std::vector<float>& radius, const vec3f& pos, float max_distance,
    bool find_any) {
  auto intersection = bvh_intersection{};
  intersection.hit  = overlap_elements_bvh(
      bvh,
      [&lines, &positions, &radius](int idx, const vec3f& pos,
          float max_distance, vec2f& uv, float& distance) {
        auto& l = lines[idx];
        return overlap_line(pos, max_distance, positions[l.x], positions[l.y],
            radius[l.x], radius[l.y], uv, distance);
      },
      pos, max_distance, intersection.element, intersection.uv,
      intersection.distance, find_any);
  return intersection;
}
bvh_intersection overlap_triangles_bvh(const bvh_tree& bvh,
    const std::vector<vec3i>& triangles, const std::vector<vec3f>& positions,
    const std::vector<float>& radius, const vec3f& pos, float max_distance,
    bool find_any) {
  auto intersection = bvh_intersection{};
  intersection.hit  = overlap_elements_bvh(
      bvh,
      [&triangles, &positions, &radius](int idx, const vec3f& pos,
          float max_distance, vec2f& uv, float& distance) {
        auto& t = triangles[idx];
        return overlap_triangle(pos, max_distance, positions[t.x],
            positions[t.y], positions[t.z], radius[t.x], radius[t.y],
            radius[t.z], uv, distance);
      },
      pos, max_distance, intersection.element, intersection.uv,
      intersection.distance, find_any);
  return intersection;
}
bvh_intersection overlap_quads_bvh(const bvh_tree& bvh,
    const std::vector<vec4i>& quads, const std::vector<vec3f>& positions,
    const std::vector<float>& radius, const vec3f& pos, float max_distance,
    bool find_any) {
  auto intersection = bvh_intersection{};
  intersection.hit  = overlap_elements_bvh(
      bvh,
      [&quads, &positions, &radius](int idx, const vec3f& pos,
          float max_distance, vec2f& uv, float& distance) {
        auto& q = quads[idx];
        return overlap_quad(pos, max_distance, positions[q.x], positions[q.y],
            positions[q.z], positions[q.w], radius[q.x], radius[q.y],
            radius[q.z], radius[q.w], uv, distance);
      },
      pos, max_distance, intersection.element, intersection.uv,
      intersection.distance, find_any);
  return intersection;
}

}  // namespace yocto::shape

// -----------------------------------------------------------------------------
// IMPLEMENTATION FOR SHAPE/SCENE BVH
// -----------------------------------------------------------------------------
namespace yocto::shape {

void init_shape_bvh(bvh_shape& shape, bool embree) {
#ifdef YOCTO_EMBREE
  // call Embree if needed
  if (embree) {
    return init_shape_embree_bvh(shape);
  }
#endif

  // build primitives
  auto bboxes = std::vector<bbox3f>{};
  if (!shape.points.empty()) {
    bboxes = std::vector<bbox3f>(shape.points.size());
    for (auto idx = 0; idx < bboxes.size(); idx++) {
      auto& p     = shape.points[idx];
      bboxes[idx] = point_bounds(shape.positions[p], shape.radius[p]);
    }
  } else if (!shape.lines.empty()) {
    bboxes = std::vector<bbox3f>(shape.lines.size());
    for (auto idx = 0; idx < bboxes.size(); idx++) {
      auto& l     = shape.lines[idx];
      bboxes[idx] = line_bounds(shape.positions[l.x], shape.positions[l.y],
          shape.radius[l.x], shape.radius[l.y]);
    }
  } else if (!shape.triangles.empty()) {
    bboxes = std::vector<bbox3f>(shape.triangles.size());
    for (auto idx = 0; idx < bboxes.size(); idx++) {
      auto& t     = shape.triangles[idx];
      bboxes[idx] = triangle_bounds(
          shape.positions[t.x], shape.positions[t.y], shape.positions[t.z]);
    }
  } else if (!shape.quads.empty()) {
    bboxes = std::vector<bbox3f>(shape.quads.size());
    for (auto idx = 0; idx < bboxes.size(); idx++) {
      auto& q     = shape.quads[idx];
      bboxes[idx] = quad_bounds(shape.positions[q.x], shape.positions[q.y],
          shape.positions[q.z], shape.positions[q.w]);
    }
  }

  // build nodes
  build_bvh(shape.bvh, bboxes);
}

void init_scene_bvh(bvh_scene& scene, bool embree) {
  // Make shape bvh
  for (auto idx = 0; idx < scene.shapes.size(); idx++) {
    init_shape_bvh(scene.shapes[idx], embree);
  }

  // embree
#ifdef YOCTO_EMBREE
  if (embree) {
    return init_scene_embree_bvh(scene);
  }
#endif

  // instance bboxes
  auto bboxes = std::vector<bbox3f>(scene.instances.size());
  for (auto idx = 0; idx < bboxes.size(); idx++) {
    auto& instance = scene.instances[idx];
    auto& shape    = scene.shapes[instance.shape];
    bboxes[idx]    = shape.bvh.nodes.empty()
                      ? invalidb3f
                      : transform_bbox(instance.frame, shape.bvh.nodes[0].bbox);
  }

  // build nodes
  build_bvh(scene.bvh, bboxes);
}

void update_shape_bvh(bvh_shape& shape) {
#ifdef YOCTO_EMBREE
  if (shape.embree_bvh) {
    throw std::runtime_error("embree shape refit not supported");
  }
#endif

  // build primitives
  auto bboxes = std::vector<bbox3f>{};
  if (!shape.points.empty()) {
    bboxes = std::vector<bbox3f>(shape.points.size());
    for (auto idx = 0; idx < bboxes.size(); idx++) {
      auto& p     = shape.points[idx];
      bboxes[idx] = point_bounds(shape.positions[p], shape.radius[p]);
    }
  } else if (!shape.lines.empty()) {
    bboxes = std::vector<bbox3f>(shape.lines.size());
    for (auto idx = 0; idx < bboxes.size(); idx++) {
      auto& l     = shape.lines[idx];
      bboxes[idx] = line_bounds(shape.positions[l.x], shape.positions[l.y],
          shape.radius[l.x], shape.radius[l.y]);
    }
  } else if (!shape.triangles.empty()) {
    bboxes = std::vector<bbox3f>(shape.triangles.size());
    for (auto idx = 0; idx < bboxes.size(); idx++) {
      auto& t     = shape.triangles[idx];
      bboxes[idx] = triangle_bounds(
          shape.positions[t.x], shape.positions[t.y], shape.positions[t.z]);
    }
  } else if (!shape.quads.empty()) {
    bboxes = std::vector<bbox3f>(shape.quads.size());
    for (auto idx = 0; idx < bboxes.size(); idx++) {
      auto& q     = shape.quads[idx];
      bboxes[idx] = quad_bounds(shape.positions[q.x], shape.positions[q.y],
          shape.positions[q.z], shape.positions[q.w]);
    }
  }

  // update nodes
  update_bvh(shape.bvh, bboxes);
}

void update_scene_bvh(bvh_scene& scene,
    const std::vector<int>&      updated_instances,
    const std::vector<int>&      updated_shapes) {
  // update shapes
  for (auto shape : updated_shapes) update_shape_bvh(scene.shapes[shape]);

#ifdef YOCTO_EMBREE
  if (scene.embree_bvh) {
    update_scene_embree_bvh(scene, updated_instances);
  }
#endif

  // build primitives
  auto bboxes = std::vector<bbox3f>(scene.instances.size());
  for (auto idx = 0; idx < bboxes.size(); idx++) {
    auto& instance = scene.instances[idx];
    auto& sbvh     = scene.shapes[instance.shape].bvh;
    bboxes[idx]    = transform_bbox(instance.frame, sbvh.nodes[0].bbox);
  }

  // update nodes
  update_bvh(scene.bvh, bboxes);
}

// Intersect ray with a bvh.
static bool intersect_shape_bvh(const bvh_shape& shape, const ray3f& ray_,
    int& element, vec2f& uv, float& distance, bool find_any) {
#ifdef YOCTO_EMBREE
  // call Embree if needed
  if (shape.embree_bvh) {
    return intersect_shape_embree_bvh(
        shape, ray_, element, uv, distance, find_any);
  }
#endif

  // check empty
  if (shape.bvh.nodes.empty()) return false;

  // node stack
  int  node_stack[128];
  auto node_cur          = 0;
  node_stack[node_cur++] = 0;

  // shared variables
  auto hit = false;

  // copy ray to modify it
  auto ray = ray_;

  // prepare ray for fast queries
  auto ray_dinv  = vec3f{1 / ray.d.x, 1 / ray.d.y, 1 / ray.d.z};
  auto ray_dsign = vec3i{(ray_dinv.x < 0) ? 1 : 0, (ray_dinv.y < 0) ? 1 : 0,
      (ray_dinv.z < 0) ? 1 : 0};

  // walking stack
  while (node_cur) {
    // grab node
    auto& node = shape.bvh.nodes[node_stack[--node_cur]];

    // intersect bbox
    // if (!intersect_bbox(ray, ray_dinv, ray_dsign, node.bbox)) continue;
    if (!intersect_bbox(ray, ray_dinv, node.bbox)) continue;

    // intersect node, switching based on node type
    // for each type, iterate over the the primitive list
    if (node.internal) {
      // for internal nodes, attempts to proceed along the
      // split axis from smallest to largest nodes
      if (ray_dsign[node.axis]) {
        node_stack[node_cur++] = node.start + 0;
        node_stack[node_cur++] = node.start + 1;
      } else {
        node_stack[node_cur++] = node.start + 1;
        node_stack[node_cur++] = node.start + 0;
      }
    } else if (!shape.points.empty()) {
      for (auto idx = node.start; idx < node.start + node.num; idx++) {
        auto& p = shape.points[shape.bvh.primitives[idx]];
        if (intersect_point(
                ray, shape.positions[p], shape.radius[p], uv, distance)) {
          hit      = true;
          element  = shape.bvh.primitives[idx];
          ray.tmax = distance;
        }
      }
    } else if (!shape.lines.empty()) {
      for (auto idx = node.start; idx < node.start + node.num; idx++) {
        auto& l = shape.lines[shape.bvh.primitives[idx]];
        if (intersect_line(ray, shape.positions[l.x], shape.positions[l.y],
                shape.radius[l.x], shape.radius[l.y], uv, distance)) {
          hit      = true;
          element  = shape.bvh.primitives[idx];
          ray.tmax = distance;
        }
      }
    } else if (!shape.triangles.empty()) {
      for (auto idx = node.start; idx < node.start + node.num; idx++) {
        auto& t = shape.triangles[shape.bvh.primitives[idx]];
        if (intersect_triangle(ray, shape.positions[t.x], shape.positions[t.y],
                shape.positions[t.z], uv, distance)) {
          hit      = true;
          element  = shape.bvh.primitives[idx];
          ray.tmax = distance;
        }
      }
    } else if (!shape.quads.empty()) {
      for (auto idx = node.start; idx < node.start + node.num; idx++) {
        auto& q = shape.quads[shape.bvh.primitives[idx]];
        if (intersect_quad(ray, shape.positions[q.x], shape.positions[q.y],
                shape.positions[q.z], shape.positions[q.w], uv, distance)) {
          hit      = true;
          element  = shape.bvh.primitives[idx];
          ray.tmax = distance;
        }
      }
    }

    // check for early exit
    if (find_any && hit) return hit;
  }

  return hit;
}

// Intersect ray with a bvh.
static bool intersect_scene_bvh(const bvh_scene& scene, const ray3f& ray_,
    int& instance, int& element, vec2f& uv, float& distance, bool find_any,
    bool non_rigid_frames) {
#ifdef YOCTO_EMBREE
  // call Embree if needed
  if (scene.embree_bvh) {
    return intersect_scene_embree_bvh(
        scene, ray_, instance, element, uv, distance, find_any);
  }
#endif

  // check empty
  if (scene.bvh.nodes.empty()) return false;

  // node stack
  int  node_stack[128];
  auto node_cur          = 0;
  node_stack[node_cur++] = 0;

  // shared variables
  auto hit = false;

  // copy ray to modify it
  auto ray = ray_;

  // prepare ray for fast queries
  auto ray_dinv  = vec3f{1 / ray.d.x, 1 / ray.d.y, 1 / ray.d.z};
  auto ray_dsign = vec3i{(ray_dinv.x < 0) ? 1 : 0, (ray_dinv.y < 0) ? 1 : 0,
      (ray_dinv.z < 0) ? 1 : 0};

  // walking stack
  while (node_cur) {
    // grab node
    auto& node = scene.bvh.nodes[node_stack[--node_cur]];

    // intersect bbox
    // if (!intersect_bbox(ray, ray_dinv, ray_dsign, node.bbox)) continue;
    if (!intersect_bbox(ray, ray_dinv, node.bbox)) continue;

    // intersect node, switching based on node type
    // for each type, iterate over the the primitive list
    if (node.internal) {
      // for internal nodes, attempts to proceed along the
      // split axis from smallest to largest nodes
      if (ray_dsign[node.axis]) {
        node_stack[node_cur++] = node.start + 0;
        node_stack[node_cur++] = node.start + 1;
      } else {
        node_stack[node_cur++] = node.start + 1;
        node_stack[node_cur++] = node.start + 0;
      }
    } else {
      for (auto idx = node.start; idx < node.start + node.num; idx++) {
        auto& instance_ = scene.instances[scene.bvh.primitives[idx]];
        auto  inv_ray   = transform_ray(
            inverse(instance_.frame, non_rigid_frames), ray);
        if (intersect_shape_bvh(scene.shapes[instance_.shape], inv_ray, element,
                uv, distance, find_any)) {
          hit      = true;
          instance = scene.bvh.primitives[idx];
          ray.tmax = distance;
        }
      }
    }

    // check for early exit
    if (find_any && hit) return hit;
  }

  return hit;
}

// Intersect ray with a bvh.
static bool intersect_instance_bvh(const bvh_scene& scene, int instance,
    const ray3f& ray, int& element, vec2f& uv, float& distance, bool find_any,
    bool non_rigid_frames) {
  auto& instance_ = scene.instances[instance];
  auto inv_ray = transform_ray(inverse(instance_.frame, non_rigid_frames), ray);
  return intersect_shape_bvh(
      scene.shapes[instance_.shape], inv_ray, element, uv, distance, find_any);
}

// Intersect ray with a bvh.
static bool overlap_shape_bvh(const bvh_shape& shape, const vec3f& pos,
    float max_distance, int& element, vec2f& uv, float& distance,
    bool find_any) {
  // check if empty
  if (shape.bvh.nodes.empty()) return false;

  // node stack
  int  node_stack[64];
  auto node_cur          = 0;
  node_stack[node_cur++] = 0;

  // hit
  auto hit = false;

  // walking stack
  while (node_cur) {
    // grab node
    auto& node = shape.bvh.nodes[node_stack[--node_cur]];

    // intersect bbox
    if (!distance_check_bbox(pos, max_distance, node.bbox)) continue;

    // intersect node, switching based on node type
    // for each type, iterate over the the primitive list
    if (node.internal) {
      // internal node
      node_stack[node_cur++] = node.start + 0;
      node_stack[node_cur++] = node.start + 1;
    } else if (!shape.points.empty()) {
      for (auto idx = 0; idx < node.num; idx++) {
        auto  primitive = shape.bvh.primitives[node.start + idx];
        auto& p         = shape.points[primitive];
        if (overlap_point(pos, max_distance, shape.positions[p],
                shape.radius[p], uv, distance)) {
          hit          = true;
          element      = primitive;
          max_distance = distance;
        }
      }
    } else if (!shape.lines.empty()) {
      for (auto idx = 0; idx < node.num; idx++) {
        auto  primitive = shape.bvh.primitives[node.start + idx];
        auto& l         = shape.lines[primitive];
        if (overlap_line(pos, max_distance, shape.positions[l.x],
                shape.positions[l.y], shape.radius[l.x], shape.radius[l.y], uv,
                distance)) {
          hit          = true;
          element      = primitive;
          max_distance = distance;
        }
      }
    } else if (!shape.triangles.empty()) {
      for (auto idx = 0; idx < node.num; idx++) {
        auto  primitive = shape.bvh.primitives[node.start + idx];
        auto& t         = shape.triangles[primitive];
        if (overlap_triangle(pos, max_distance, shape.positions[t.x],
                shape.positions[t.y], shape.positions[t.z], shape.radius[t.x],
                shape.radius[t.y], shape.radius[t.z], uv, distance)) {
          hit          = true;
          element      = primitive;
          max_distance = distance;
        }
      }
    } else if (!shape.quads.empty()) {
      for (auto idx = 0; idx < node.num; idx++) {
        auto  primitive = shape.bvh.primitives[node.start + idx];
        auto& q         = shape.quads[primitive];
        if (overlap_quad(pos, max_distance, shape.positions[q.x],
                shape.positions[q.y], shape.positions[q.z],
                shape.positions[q.w], shape.radius[q.x], shape.radius[q.y],
                shape.radius[q.z], shape.radius[q.w], uv, distance)) {
          hit          = true;
          element      = primitive;
          max_distance = distance;
        }
      }
    }

    // check for early exit
    if (find_any && hit) return hit;
  }

  return hit;
}

// Intersect ray with a bvh.
static bool overlap_scene_bvh(const bvh_scene& scene, const vec3f& pos,
    float max_distance, int& instance, int& element, vec2f& uv, float& distance,
    bool find_any, bool non_rigid_frames) {
  // check if empty
  if (scene.bvh.nodes.empty()) return false;

  // node stack
  int  node_stack[64];
  auto node_cur          = 0;
  node_stack[node_cur++] = 0;

  // hit
  auto hit = false;

  // walking stack
  while (node_cur) {
    // grab node
    auto& node = scene.bvh.nodes[node_stack[--node_cur]];

    // intersect bbox
    if (!distance_check_bbox(pos, max_distance, node.bbox)) continue;

    // intersect node, switching based on node type
    // for each type, iterate over the the primitive list
    if (node.internal) {
      // internal node
      node_stack[node_cur++] = node.start + 0;
      node_stack[node_cur++] = node.start + 1;
    } else {
      for (auto idx = 0; idx < node.num; idx++) {
        auto primitive = scene.bvh.primitives[node.start + idx];
        auto instance_ = scene.instances[primitive];
        auto inv_pos   = transform_point(
            inverse(instance_.frame, non_rigid_frames), pos);
        if (overlap_shape_bvh(scene.shapes[instance_.shape], inv_pos,
                max_distance, element, uv, distance, find_any)) {
          hit          = true;
          instance     = primitive;
          max_distance = distance;
        }
      }
    }

    // check for early exit
    if (find_any && hit) return hit;
  }

  return hit;
}

#if 0
    // Finds the overlap between BVH leaf nodes.
    template <typename OverlapElem>
    void overlap_bvh_elems(const bvh_scene_data& bvh1, const bvh_scene_data& bvh2,
                           bool skip_duplicates, bool skip_self, std::vector<vec2i>& overlaps,
                           const OverlapElem& overlap_elems) {
        // node stack
        vec2i node_stack[128];
        auto node_cur = 0;
        node_stack[node_cur++] = {0, 0};

        // walking stack
        while (node_cur) {
            // grab node
            auto node_idx = node_stack[--node_cur];
            const auto node1 = bvh1->nodes[node_idx.x];
            const auto node2 = bvh2->nodes[node_idx.y];

            // intersect bbox
            if (!overlap_bbox(node1.bbox, node2.bbox)) continue;

            // check for leaves
            if (node1.isleaf && node2.isleaf) {
                // collide primitives
                for (auto i1 = node1.start; i1 < node1.start + node1.count; i1++) {
                    for (auto i2 = node2.start; i2 < node2.start + node2.count;
                         i2++) {
                        auto idx1 = bvh1->sorted_prim[i1];
                        auto idx2 = bvh2->sorted_prim[i2];
                        if (skip_duplicates && idx1 > idx2) continue;
                        if (skip_self && idx1 == idx2) continue;
                        if (overlap_elems(idx1, idx2))
                            overlaps.push_back({idx1, idx2});
                    }
                }
            } else {
                // descend
                if (node1.isleaf) {
                    for (auto idx2 = node2.start; idx2 < node2.start + node2.count;
                         idx2++) {
                        node_stack[node_cur++] = {node_idx.x, (int)idx2};
                    }
                } else if (node2.isleaf) {
                    for (auto idx1 = node1.start; idx1 < node1.start + node1.count;
                         idx1++) {
                        node_stack[node_cur++] = {(int)idx1, node_idx.y};
                    }
                } else {
                    for (auto idx2 = node2.start; idx2 < node2.start + node2.count;
                         idx2++) {
                        for (auto idx1 = node1.start;
                             idx1 < node1.start + node1.count; idx1++) {
                            node_stack[node_cur++] = {(int)idx1, (int)idx2};
                        }
                    }
                }
            }
        }
    }
#endif

bvh_intersection intersect_shape_bvh(
    const bvh_shape& shape, const ray3f& ray, bool find_any) {
  auto intersection = bvh_intersection{};
  intersection.hit  = intersect_shape_bvh(shape, ray, intersection.element,
      intersection.uv, intersection.distance, find_any);
  return intersection;
}
bvh_intersection intersect_scene_bvh(const bvh_scene& scene, const ray3f& ray,
    bool find_any, bool non_rigid_frames) {
  auto intersection = bvh_intersection{};
  intersection.hit  = intersect_scene_bvh(scene, ray, intersection.instance,
      intersection.element, intersection.uv, intersection.distance, find_any,
      non_rigid_frames);
  return intersection;
}
bvh_intersection intersect_instance_bvh(const bvh_scene& scene, int instance,
    const ray3f& ray, bool find_any, bool non_rigid_frames) {
  auto intersection     = bvh_intersection{};
  intersection.hit      = intersect_instance_bvh(scene, instance, ray,
      intersection.element, intersection.uv, intersection.distance, find_any,
      non_rigid_frames);
  intersection.instance = instance;
  return intersection;
}

bvh_intersection overlap_shape_bvh(const bvh_shape& shape, const vec3f& pos,
    float max_distance, bool find_any) {
  auto intersection = bvh_intersection{};
  intersection.hit  = overlap_shape_bvh(shape, pos, max_distance,
      intersection.element, intersection.uv, intersection.distance, find_any);
  return intersection;
}
bvh_intersection overlap_scene_bvh(const bvh_scene& scene, const vec3f& pos,
    float max_distance, bool find_any, bool non_rigid_frames) {
  auto intersection = bvh_intersection{};
  intersection.hit  = overlap_scene_bvh(scene, pos, max_distance,
      intersection.instance, intersection.element, intersection.uv,
      intersection.distance, find_any, non_rigid_frames);
  return intersection;
}

}  // namespace yocto::shape

// -----------------------------------------------------------------------------
// HASH GRID AND NEAREST NEIGHTBORS
// -----------------------------------------------------------------------------

namespace yocto::shape {

// Gets the cell index
vec3i get_cell_index(const hash_grid& grid, const vec3f& position) {
  auto scaledpos = position * grid.cell_inv_size;
  return vec3i{(int)scaledpos.x, (int)scaledpos.y, (int)scaledpos.z};
}

// Create a hash_grid
hash_grid make_hash_grid(float cell_size) {
  auto grid          = hash_grid{};
  grid.cell_size     = cell_size;
  grid.cell_inv_size = 1 / cell_size;
  return grid;
}
hash_grid make_hash_grid(const std::vector<vec3f>& positions, float cell_size) {
  auto grid          = hash_grid{};
  grid.cell_size     = cell_size;
  grid.cell_inv_size = 1 / cell_size;
  for (auto& position : positions) insert_vertex(grid, position);
  return grid;
}
// Inserts a point into the grid
int insert_vertex(hash_grid& grid, const vec3f& position) {
  auto vertex_id = (int)grid.positions.size();
  auto cell      = get_cell_index(grid, position);
  grid.cells[cell].push_back(vertex_id);
  grid.positions.push_back(position);
  return vertex_id;
}
// Finds the nearest neighbors within a given radius
void find_neighbors(const hash_grid& grid, std::vector<int>& neighbors,
    const vec3f& position, float max_radius, int skip_id) {
  auto cell        = get_cell_index(grid, position);
  auto cell_radius = (int)(max_radius * grid.cell_inv_size) + 1;
  neighbors.clear();
  auto max_radius_squared = max_radius * max_radius;
  for (auto k = -cell_radius; k <= cell_radius; k++) {
    for (auto j = -cell_radius; j <= cell_radius; j++) {
      for (auto i = -cell_radius; i <= cell_radius; i++) {
        auto ncell         = cell + vec3i{i, j, k};
        auto cell_iterator = grid.cells.find(ncell);
        if (cell_iterator == grid.cells.end()) continue;
        auto& ncell_vertices = cell_iterator->second;
        for (auto vertex_id : ncell_vertices) {
          if (distance_squared(grid.positions[vertex_id], position) >
              max_radius_squared)
            continue;
          if (vertex_id == skip_id) continue;
          neighbors.push_back(vertex_id);
        }
      }
    }
  }
}
void find_neighbors(const hash_grid& grid, std::vector<int>& neighbors,
    const vec3f& position, float max_radius) {
  find_neighbors(grid, neighbors, position, max_radius, -1);
}
void find_neighbors(const hash_grid& grid, std::vector<int>& neighbors,
    int vertex, float max_radius) {
  find_neighbors(grid, neighbors, grid.positions[vertex], max_radius, vertex);
}

}  // namespace yocto::shape

// -----------------------------------------------------------------------------
// PROCEDURAL MODELING
// -----------------------------------------------------------------------------
namespace yocto::shape {

// Extract isoline from surface scalar field.
void meandering_triangles(const std::vector<float>& field, float isoline,
    int selected_tag, int t0, int t1, std::vector<vec3i>& triangles,
    std::vector<int>& tags, std::vector<vec3f>& positions,
    std::vector<vec3f>& normals) {
  auto num_triangles = triangles.size();

  // Edgemap to keep track of the added vertex on each splitted edge.
  // key: edge (ordered std::pair), value: vertex index
  auto emap = std::unordered_map<vec2i, int>();

  // Helper procedures.
  auto make_edge = [](int a, int b) -> vec2i {
    return a < b ? vec2i{a, b} : vec2i{b, a};
  };
  auto add_vertex = [&](int a, int b, float coeff) -> int {
    auto position = coeff * positions[a] + (1 - coeff) * positions[b];
    auto normal   = normalize(coeff * normals[a] + (1 - coeff) * normals[b]);
    auto index    = (int)positions.size();
    positions.push_back(position);
    normals.push_back(normal);
    return index;
  };
  auto get_tag = [&](int v) { return field[v] > isoline ? t1 : t0; };

  for (int i = 0; i < num_triangles; ++i) {
    if (tags[i] != selected_tag) continue;

    auto tr = triangles[i];

    // Find which vertex has different tag, if any.
    int j = -1;
    for (int k = 0; k < 3; k++) {
      if (get_tag(tr[k]) != get_tag(tr[(k + 1) % 3]) &&
          get_tag(tr[k]) != get_tag(tr[(k + 2) % 3])) {
        j = k;
      }
    }

    // If all vertices have same tag, then tag the whole triangle and continue.
    if (j == -1) {
      tags[i] = get_tag(tr.x);
      continue;
    }

    // Reorder the triangle such that vertex with different tag is always z
    tr = {tr[(j + 2) % 3], tr[(j + 1) % 3], tr[j]};

    auto values = vec3f{field[tr.x], field[tr.y], field[tr.z]};
    values -= isoline;

    // Create or retrieve new vertices that split the edges
    int new_verts[2];
    for (int k : {0, 1}) {
      int   vert = tr[k];
      float a    = values[k];
      float b    = values.z;
      auto  edge = make_edge(tr.z, vert);
      auto  it   = emap.find(edge);

      if (it != emap.end()) {
        // Edge already processed.
        new_verts[k] = it->second;
      } else {
        // Compute new vertex via interpolation.
        float alpha  = fabs(a / (b - a));
        new_verts[k] = add_vertex(tr.z, vert, alpha);
        emap.insert(it, {edge, new_verts[k]});
      }
    }

    /*
                     tr.z
                       /\
                      /  \
                     /    \
                    /  i   \
                   /        \
    new_verts[1]  /..........\  new_verts[0]
                 /   . n_f[0] \
                /       .      \
               /           .    \
              / new_faces[1]  .  \
             /___________________.\
       tr.y                      tr.x

    */

    // Add two new faces.
    triangles.push_back({new_verts[0], new_verts[1], tr.x});
    tags.push_back(get_tag(tr.x));

    triangles.push_back({tr.y, tr.x, new_verts[1]});
    tags.push_back(get_tag(tr.y));

    // Edit old face.
    triangles[i] = vec3i{new_verts[0], tr.z, new_verts[1]};
    tags[i]      = get_tag(tr.z);
  }
}

}  // namespace yocto::shape

// -----------------------------------------------------------------------------
// IMPLEMENTATION OF SHAPE ELEMENT CONVERSION AND GROUPING
// -----------------------------------------------------------------------------
namespace yocto::shape {

// Convert quads to triangles
std::vector<vec3i> quads_to_triangles(const std::vector<vec4i>& quads) {
  auto triangles = std::vector<vec3i>{};
  triangles.reserve(quads.size() * 2);
  for (auto& q : quads) {
    triangles.push_back({q.x, q.y, q.w});
    if (q.z != q.w) triangles.push_back({q.z, q.w, q.y});
  }
  return triangles;
}

// Convert triangles to quads by creating degenerate quads
std::vector<vec4i> triangles_to_quads(const std::vector<vec3i>& triangles) {
  auto quads = std::vector<vec4i>{};
  quads.reserve(triangles.size());
  for (auto& t : triangles) quads.push_back({t.x, t.y, t.z, t.z});
  return quads;
}

// Convert beziers to lines using 3 lines for each bezier.
std::vector<vec2i> bezier_to_lines(const std::vector<vec4i>& beziers) {
  auto lines = std::vector<vec2i>{};
  lines.reserve(beziers.size() * 3);
  for (auto b : beziers) {
    lines.push_back({b.x, b.y});
    lines.push_back({b.y, b.z});
    lines.push_back({b.z, b.w});
  }
  return lines;
}

// Convert face varying data to single primitives. Returns the quads indices
// and filled vectors for pos, norm and texcoord.
std::tuple<std::vector<vec4i>, std::vector<vec3f>, std::vector<vec3f>,
    std::vector<vec2f>>
split_facevarying(const std::vector<vec4i>& quadspos,
    const std::vector<vec4i>&               quadsnorm,
    const std::vector<vec4i>&               quadstexcoord,
    const std::vector<vec3f>& positions, const std::vector<vec3f>& normals,
    const std::vector<vec2f>& texcoords) {
  auto split = std::tuple<std::vector<vec4i>, std::vector<vec3f>,
      std::vector<vec3f>, std::vector<vec2f>>{};
  auto& [split_quads, split_positions, split_normals, split_texcoords] = split;
  // make faces unique
  std::unordered_map<vec3i, int> vert_map;
  split_quads.resize(quadspos.size());
  for (auto fid = 0; fid < quadspos.size(); fid++) {
    for (auto c = 0; c < 4; c++) {
      auto v = vec3i{
          (&quadspos[fid].x)[c],
          (!quadsnorm.empty()) ? (&quadsnorm[fid].x)[c] : -1,
          (!quadstexcoord.empty()) ? (&quadstexcoord[fid].x)[c] : -1,
      };
      auto it = vert_map.find(v);
      if (it == vert_map.end()) {
        auto s = (int)vert_map.size();
        vert_map.insert(it, {v, s});
        (&split_quads[fid].x)[c] = s;
      } else {
        (&split_quads[fid].x)[c] = it->second;
      }
    }
  }

  // fill vert data
  split_positions.clear();
  if (!positions.empty()) {
    split_positions.resize(vert_map.size());
    for (auto& [vert, index] : vert_map) {
      split_positions[index] = positions[vert.x];
    }
  }
  split_normals.clear();
  if (!normals.empty()) {
    split_normals.resize(vert_map.size());
    for (auto& [vert, index] : vert_map) {
      split_normals[index] = normals[vert.y];
    }
  }
  split_texcoords.clear();
  if (!texcoords.empty()) {
    split_texcoords.resize(vert_map.size());
    for (auto& [vert, index] : vert_map) {
      split_texcoords[index] = texcoords[vert.z];
    }
  }

  return split;
}

// Split primitives per id
template <typename T>
std::vector<std::vector<T>> ungroup_elems_impl(
    const std::vector<T>& elems, const std::vector<int>& ids) {
  auto max_id      = *max_element(ids.begin(), ids.end());
  auto split_elems = std::vector<std::vector<T>>(max_id + 1);
  for (auto elem_id = 0; elem_id < elems.size(); elem_id++) {
    split_elems[ids[elem_id]].push_back(elems[elem_id]);
  }
  return split_elems;
}
std::vector<std::vector<vec2i>> ungroup_lines(
    const std::vector<vec2i>& lines, const std::vector<int>& ids) {
  return ungroup_elems_impl(lines, ids);
}
std::vector<std::vector<vec3i>> ungroup_triangles(
    const std::vector<vec3i>& triangles, const std::vector<int>& ids) {
  return ungroup_elems_impl(triangles, ids);
}
std::vector<std::vector<vec4i>> ungroup_quads(
    const std::vector<vec4i>& quads, const std::vector<int>& ids) {
  return ungroup_elems_impl(quads, ids);
}

// Weld vertices within a threshold.
std::pair<std::vector<vec3f>, std::vector<int>> weld_vertices(
    const std::vector<vec3f>& positions, float threshold) {
  auto indices   = std::vector<int>(positions.size());
  auto welded    = std::vector<vec3f>{};
  auto grid      = make_hash_grid(threshold);
  auto neighbors = std::vector<int>{};
  for (auto vertex = 0; vertex < positions.size(); vertex++) {
    auto& position = positions[vertex];
    find_neighbors(grid, neighbors, position, threshold);
    if (neighbors.empty()) {
      welded.push_back(position);
      indices[vertex] = (int)welded.size() - 1;
      insert_vertex(grid, position);
    } else {
      indices[vertex] = neighbors.front();
    }
  }
  return {welded, indices};
}
std::pair<std::vector<vec3i>, std::vector<vec3f>> weld_triangles(
    const std::vector<vec3i>& triangles, const std::vector<vec3f>& positions,
    float threshold) {
  auto [wpositions, indices] = weld_vertices(positions, threshold);
  auto wtriangles            = triangles;
  for (auto& t : wtriangles) t = {indices[t.x], indices[t.y], indices[t.z]};
  return {wtriangles, wpositions};
}
std::pair<std::vector<vec4i>, std::vector<vec3f>> weld_quads(
    const std::vector<vec4i>& quads, const std::vector<vec3f>& positions,
    float threshold) {
  auto [wpositions, indices] = weld_vertices(positions, threshold);
  auto wquads                = quads;
  for (auto& q : wquads)
    q = {
        indices[q.x],
        indices[q.y],
        indices[q.z],
        indices[q.w],
    };
  return {wquads, wpositions};
}

// Merge shape elements
void merge_lines(std::vector<vec2i>& lines,
    const std::vector<vec2i>& merge_lines, int num_verts) {
  for (auto& l : merge_lines)
    lines.push_back({l.x + num_verts, l.y + num_verts});
}
void merge_triangles(std::vector<vec3i>& triangles,
    const std::vector<vec3i>& merge_triangles, int num_verts) {
  for (auto& t : merge_triangles)
    triangles.push_back({t.x + num_verts, t.y + num_verts, t.z + num_verts});
}
void merge_quads(std::vector<vec4i>& quads,
    const std::vector<vec4i>& merge_quads, int num_verts) {
  for (auto& q : merge_quads)
    quads.push_back(
        {q.x + num_verts, q.y + num_verts, q.z + num_verts, q.w + num_verts});
}
void merge_lines(std::vector<vec2i>& lines, std::vector<vec3f>& positions,
    std::vector<vec3f>& tangents, std::vector<vec2f>& texcoords,
    std::vector<float>& radius, const std::vector<vec2i>& merge_lines,
    const std::vector<vec3f>& merge_positions,
    const std::vector<vec3f>& merge_tangents,
    const std::vector<vec2f>& merge_texturecoords,
    const std::vector<float>& merge_radius) {
  auto merge_verts = (int)positions.size();
  for (auto& l : merge_lines)
    lines.push_back({l.x + merge_verts, l.y + merge_verts});
  positions.insert(
      positions.end(), merge_positions.begin(), merge_positions.end());
  tangents.insert(tangents.end(), merge_tangents.begin(), merge_tangents.end());
  texcoords.insert(
      texcoords.end(), merge_texturecoords.begin(), merge_texturecoords.end());
  radius.insert(radius.end(), merge_radius.begin(), merge_radius.end());
}
void merge_triangles(std::vector<vec3i>& triangles,
    std::vector<vec3f>& positions, std::vector<vec3f>& normals,
    std::vector<vec2f>& texcoords, const std::vector<vec3i>& merge_triangles,
    const std::vector<vec3f>& merge_positions,
    const std::vector<vec3f>& merge_normals,
    const std::vector<vec2f>& merge_texturecoords) {
  auto merge_verts = (int)positions.size();
  for (auto& t : merge_triangles)
    triangles.push_back(
        {t.x + merge_verts, t.y + merge_verts, t.z + merge_verts});
  positions.insert(
      positions.end(), merge_positions.begin(), merge_positions.end());
  normals.insert(normals.end(), merge_normals.begin(), merge_normals.end());
  texcoords.insert(
      texcoords.end(), merge_texturecoords.begin(), merge_texturecoords.end());
}
void merge_quads(std::vector<vec4i>& quads, std::vector<vec3f>& positions,
    std::vector<vec3f>& normals, std::vector<vec2f>& texcoords,
    const std::vector<vec4i>& merge_quads,
    const std::vector<vec3f>& merge_positions,
    const std::vector<vec3f>& merge_normals,
    const std::vector<vec2f>& merge_texturecoords) {
  auto merge_verts = (int)positions.size();
  for (auto& q : merge_quads)
    quads.push_back({q.x + merge_verts, q.y + merge_verts, q.z + merge_verts,
        q.w + merge_verts});
  positions.insert(
      positions.end(), merge_positions.begin(), merge_positions.end());
  normals.insert(normals.end(), merge_normals.begin(), merge_normals.end());
  texcoords.insert(
      texcoords.end(), merge_texturecoords.begin(), merge_texturecoords.end());
}

void merge_triangles_and_quads(std::vector<vec3i>& triangles,
    std::vector<vec4i>& quads, bool force_triangles) {
  if (quads.empty()) return;
  if (force_triangles) {
    auto qtriangles = quads_to_triangles(quads);
    triangles.insert(triangles.end(), qtriangles.begin(), qtriangles.end());
    quads = {};
  } else {
    auto tquads = triangles_to_quads(triangles);
    quads.insert(quads.end(), tquads.begin(), tquads.end());
    triangles = {};
  }
}

}  // namespace yocto::shape

// -----------------------------------------------------------------------------
// IMPLEMENTATION OF SHAPE SUBDIVISION
// -----------------------------------------------------------------------------
namespace yocto::shape {

// Subdivide lines.
template <typename T>
void subdivide_lines_impl(std::vector<vec2i>& lines, std::vector<T>& vert,
    const std::vector<vec2i>& lines_, const std::vector<T>& vert_, int level) {
  // initialization
  lines = lines_;
  vert  = vert_;
  // early exit
  if (lines.empty() || vert.empty()) return;
  // loop over levels
  for (auto l = 0; l < level; l++) {
    // sizes
    auto nverts = (int)vert.size();
    auto nlines = (int)lines.size();
    // create vertices
    auto tvert = std::vector<T>(nverts + nlines);
    for (auto i = 0; i < nverts; i++) tvert[i] = vert[i];
    for (auto i = 0; i < nlines; i++) {
      auto l            = lines[i];
      tvert[nverts + i] = (vert[l.x] + vert[l.y]) / 2;
    }
    // create lines
    auto tlines = std::vector<vec2i>(nlines * 2);
    for (auto i = 0; i < nlines; i++) {
      auto l            = lines[i];
      tlines[i * 2 + 0] = {l.x, nverts + i};
      tlines[i * 2 + 0] = {nverts + i, l.y};
    }
    swap(tlines, lines);
    swap(tvert, vert);
  }
}
template <typename T>
std::pair<std::vector<vec2i>, std::vector<T>> subdivide_lines_impl(
    const std::vector<vec2i>& lines, const std::vector<T>& vert, int level) {
  auto tess = std::pair<std::vector<vec2i>, std::vector<T>>{};
  subdivide_lines_impl(tess.first, tess.second, lines, vert, level);
  return tess;
}

// Subdivide triangle.
template <typename T>
void subdivide_triangles_impl(std::vector<vec3i>& triangles,
    std::vector<T>& vert, const std::vector<vec3i>& triangles_,
    const std::vector<T>& vert_, int level) {
  // initialization
  triangles = triangles_;
  vert      = vert_;
  // early exit
  if (triangles.empty() || vert.empty()) return;
  // loop over levels
  for (auto l = 0; l < level; l++) {
    // get edges
    auto emap  = make_edge_map(triangles);
    auto edges = get_edges(emap);
    // number of elements
    auto nverts = (int)vert.size();
    auto nedges = (int)edges.size();
    auto nfaces = (int)triangles.size();
    // create vertices
    auto tvert = std::vector<T>(nverts + nedges);
    for (auto i = 0; i < nverts; i++) tvert[i] = vert[i];
    for (auto i = 0; i < nedges; i++) {
      auto e            = edges[i];
      tvert[nverts + i] = (vert[e.x] + vert[e.y]) / 2;
    }
    // create triangles
    auto ttriangles = std::vector<vec3i>(nfaces * 4);
    for (auto i = 0; i < nfaces; i++) {
      auto t                = triangles[i];
      ttriangles[i * 4 + 0] = {t.x, nverts + edge_index(emap, {t.x, t.y}),
          nverts + edge_index(emap, {t.z, t.x})};
      ttriangles[i * 4 + 1] = {t.y, nverts + edge_index(emap, {t.y, t.z}),
          nverts + edge_index(emap, {t.x, t.y})};
      ttriangles[i * 4 + 2] = {t.z, nverts + edge_index(emap, {t.z, t.x}),
          nverts + edge_index(emap, {t.y, t.z})};
      ttriangles[i * 4 + 3] = {nverts + edge_index(emap, {t.x, t.y}),
          nverts + edge_index(emap, {t.y, t.z}),
          nverts + edge_index(emap, {t.z, t.x})};
    }
    swap(ttriangles, triangles);
    swap(tvert, vert);
  }
}
template <typename T>
std::pair<std::vector<vec3i>, std::vector<T>> subdivide_triangles_impl(
    const std::vector<vec3i>& triangles, const std::vector<T>& vert,
    int level) {
  auto tess = std::pair<std::vector<vec3i>, std::vector<T>>{};
  subdivide_triangles_impl(tess.first, tess.second, triangles, vert, level);
  return tess;
}

// Subdivide quads.
template <typename T>
void subdivide_quads_impl(std::vector<vec4i>& quads, std::vector<T>& vert,
    const std::vector<vec4i>& quads_, const std::vector<T>& vert_, int level) {
  // initialization
  quads = quads_;
  vert  = vert_;
  // early exit
  if (quads.empty() || vert.empty()) return;
  // loop over levels
  for (auto l = 0; l < level; l++) {
    // get edges
    auto emap  = make_edge_map(quads);
    auto edges = get_edges(emap);
    // number of elements
    auto nverts = (int)vert.size();
    auto nedges = (int)edges.size();
    auto nfaces = (int)quads.size();
    // create vertices
    auto tvert = std::vector<T>(nverts + nedges + nfaces);
    for (auto i = 0; i < nverts; i++) tvert[i] = vert[i];
    for (auto i = 0; i < nedges; i++) {
      auto e            = edges[i];
      tvert[nverts + i] = (vert[e.x] + vert[e.y]) / 2;
    }
    for (auto i = 0; i < nfaces; i++) {
      auto q = quads[i];
      if (q.z != q.w) {
        tvert[nverts + nedges + i] =
            (vert[q.x] + vert[q.y] + vert[q.z] + vert[q.w]) / 4;
      } else {
        tvert[nverts + nedges + i] = (vert[q.x] + vert[q.y] + vert[q.y]) / 3;
      }
    }
    // create quads
    auto tquads = std::vector<vec4i>(nfaces * 4);  // conservative allocation
    auto qi     = 0;
    for (auto i = 0; i < nfaces; i++) {
      auto q = quads[i];
      if (q.z != q.w) {
        tquads[qi++] = {q.x, nverts + edge_index(emap, {q.x, q.y}),
            nverts + nedges + i, nverts + edge_index(emap, {q.w, q.x})};
        tquads[qi++] = {q.y, nverts + edge_index(emap, {q.y, q.z}),
            nverts + nedges + i, nverts + edge_index(emap, {q.x, q.y})};
        tquads[qi++] = {q.z, nverts + edge_index(emap, {q.z, q.w}),
            nverts + nedges + i, nverts + edge_index(emap, {q.y, q.z})};
        tquads[qi++] = {q.w, nverts + edge_index(emap, {q.w, q.x}),
            nverts + nedges + i, nverts + edge_index(emap, {q.z, q.w})};
      } else {
        tquads[qi++] = {q.x, nverts + edge_index(emap, {q.x, q.y}),
            nverts + nedges + i, nverts + edge_index(emap, {q.z, q.x})};
        tquads[qi++] = {q.y, nverts + edge_index(emap, {q.y, q.z}),
            nverts + nedges + i, nverts + edge_index(emap, {q.x, q.y})};
        tquads[qi++] = {q.z, nverts + edge_index(emap, {q.z, q.x}),
            nverts + nedges + i, nverts + edge_index(emap, {q.y, q.z})};
      }
    }
    tquads.resize(qi);
    // done
    swap(tquads, quads);
    swap(tvert, vert);
  }
}
template <typename T>
std::pair<std::vector<vec4i>, std::vector<T>> subdivide_quads_impl(
    const std::vector<vec4i>& quads, const std::vector<T>& vert, int level) {
  auto tess = std::pair<std::vector<vec4i>, std::vector<T>>{};
  subdivide_quads_impl(tess.first, tess.second, quads, vert, level);
  return tess;
}

// Subdivide beziers.
template <typename T>
void subdivide_beziers_impl(std::vector<vec4i>& beziers, std::vector<T>& vert,
    const std::vector<vec4i>& beziers_, const std::vector<T>& vert_,
    int level) {
  // initialization
  beziers = beziers_;
  vert    = vert_;
  // early exit
  if (beziers.empty() || vert.empty()) return;
  // loop over levels
  for (auto l = 0; l < level; l++) {
    // get edges
    auto vmap     = std::unordered_map<int, int>();
    auto tvert    = std::vector<T>();
    auto tbeziers = std::vector<vec4i>();
    for (auto b : beziers) {
      if (vmap.find(b.x) == vmap.end()) {
        vmap[b.x] = (int)tvert.size();
        tvert.push_back(vert[b.x]);
      }
      if (vmap.find(b.w) == vmap.end()) {
        vmap[b.w] = (int)tvert.size();
        tvert.push_back(vert[b.w]);
      }
      auto bo = (int)tvert.size();
      tbeziers.push_back({vmap.at(b.x), bo + 0, bo + 1, bo + 2});
      tbeziers.push_back({bo + 2, bo + 3, bo + 4, vmap.at(b.w)});
      tvert.push_back(vert[b.x] / 2 + vert[b.y] / 2);
      tvert.push_back(vert[b.x] / 4 + vert[b.y] / 2 + vert[b.z] / 4);
      tvert.push_back(vert[b.x] / 8 + vert[b.y] * ((float)3 / (float)8) +
                      vert[b.z] * ((float)3 / (float)8) + vert[b.w] / 8);
      tvert.push_back(vert[b.y] / 4 + vert[b.z] / 2 + vert[b.w] / 4);
      tvert.push_back(vert[b.z] / 2 + vert[b.w] / 2);
    }

    // done
    swap(tbeziers, beziers);
    swap(tvert, vert);
  }
}
template <typename T>
std::pair<std::vector<vec4i>, std::vector<T>> subdivide_beziers_impl(
    const std::vector<vec4i>& beziers, const std::vector<T>& vert, int level) {
  auto tess = std::pair<std::vector<vec4i>, std::vector<T>>{};
  subdivide_beziers_impl(tess.first, tess.second, beziers, vert, level);
  return tess;
}

// Subdivide catmullclark.
template <typename T>
void subdivide_catmullclark_impl(std::vector<vec4i>& quads,
    std::vector<T>& vert, const std::vector<vec4i>& quads_,
    const std::vector<T>& vert_, int level, bool lock_boundary) {
  // initialization
  quads = quads_;
  vert  = vert_;
  // early exit
  if (quads.empty() || vert.empty()) return;
  // loop over levels
  for (auto l = 0; l < level; l++) {
    // get edges
    auto emap     = make_edge_map(quads);
    auto edges    = get_edges(emap);
    auto boundary = get_boundary(emap);
    // number of elements
    auto nverts    = (int)vert.size();
    auto nedges    = (int)edges.size();
    auto nboundary = (int)boundary.size();
    auto nfaces    = (int)quads.size();

    // split elements ------------------------------------
    // create vertices
    auto tvert = std::vector<T>(nverts + nedges + nfaces);
    for (auto i = 0; i < nverts; i++) tvert[i] = vert[i];
    for (auto i = 0; i < nedges; i++) {
      auto e            = edges[i];
      tvert[nverts + i] = (vert[e.x] + vert[e.y]) / 2;
    }
    for (auto i = 0; i < nfaces; i++) {
      auto q = quads[i];
      if (q.z != q.w) {
        tvert[nverts + nedges + i] =
            (vert[q.x] + vert[q.y] + vert[q.z] + vert[q.w]) / 4;
      } else {
        tvert[nverts + nedges + i] = (vert[q.x] + vert[q.y] + vert[q.y]) / 3;
      }
    }
    // create quads
    auto tquads = std::vector<vec4i>(nfaces * 4);  // conservative allocation
    auto qi     = 0;
    for (auto i = 0; i < nfaces; i++) {
      auto q = quads[i];
      if (q.z != q.w) {
        tquads[qi++] = {q.x, nverts + edge_index(emap, {q.x, q.y}),
            nverts + nedges + i, nverts + edge_index(emap, {q.w, q.x})};
        tquads[qi++] = {q.y, nverts + edge_index(emap, {q.y, q.z}),
            nverts + nedges + i, nverts + edge_index(emap, {q.x, q.y})};
        tquads[qi++] = {q.z, nverts + edge_index(emap, {q.z, q.w}),
            nverts + nedges + i, nverts + edge_index(emap, {q.y, q.z})};
        tquads[qi++] = {q.w, nverts + edge_index(emap, {q.w, q.x}),
            nverts + nedges + i, nverts + edge_index(emap, {q.z, q.w})};
      } else {
        tquads[qi++] = {q.x, nverts + edge_index(emap, {q.x, q.y}),
            nverts + nedges + i, nverts + edge_index(emap, {q.z, q.x})};
        tquads[qi++] = {q.y, nverts + edge_index(emap, {q.y, q.z}),
            nverts + nedges + i, nverts + edge_index(emap, {q.x, q.y})};
        tquads[qi++] = {q.z, nverts + edge_index(emap, {q.z, q.x}),
            nverts + nedges + i, nverts + edge_index(emap, {q.y, q.z})};
      }
    }
    tquads.resize(qi);

    // split boundary
    auto tboundary = std::vector<vec2i>(nboundary * 2);
    for (auto i = 0; i < nboundary; i++) {
      auto e               = boundary[i];
      tboundary[i * 2 + 0] = {e.x, nverts + edge_index(emap, e)};
      tboundary[i * 2 + 1] = {nverts + edge_index(emap, e), e.y};
    }

    // setup creases -----------------------------------
    auto tcrease_edges = std::vector<vec2i>();
    auto tcrease_verts = std::vector<int>();
    if (lock_boundary) {
      for (auto& b : tboundary) {
        tcrease_verts.push_back(b.x);
        tcrease_verts.push_back(b.y);
      }
    } else {
      for (auto& b : tboundary) tcrease_edges.push_back(b);
    }

    // define vertex valence ---------------------------
    auto tvert_val = std::vector<int>(tvert.size(), 2);
    for (auto& e : tboundary) {
      tvert_val[e.x] = (lock_boundary) ? 0 : 1;
      tvert_val[e.y] = (lock_boundary) ? 0 : 1;
    }

    // averaging pass ----------------------------------
    auto avert  = std::vector<T>(tvert.size(), T());
    auto acount = std::vector<int>(tvert.size(), 0);
    for (auto p : tcrease_verts) {
      if (tvert_val[p] != 0) continue;
      avert[p] += tvert[p];
      acount[p] += 1;
    }
    for (auto& e : tcrease_edges) {
      auto c = (tvert[e.x] + tvert[e.y]) / 2;
      for (auto vid : {e.x, e.y}) {
        if (tvert_val[vid] != 1) continue;
        avert[vid] += c;
        acount[vid] += 1;
      }
    }
    for (auto& q : tquads) {
      auto c = (tvert[q.x] + tvert[q.y] + tvert[q.z] + tvert[q.w]) / 4;
      for (auto vid : {q.x, q.y, q.z, q.w}) {
        if (tvert_val[vid] != 2) continue;
        avert[vid] += c;
        acount[vid] += 1;
      }
    }
    for (auto i = 0; i < tvert.size(); i++) avert[i] /= (float)acount[i];

    // correction pass ----------------------------------
    // p = p + (avg_p - p) * (4/avg_count)
    for (auto i = 0; i < tvert.size(); i++) {
      if (tvert_val[i] != 2) continue;
      avert[i] = tvert[i] + (avert[i] - tvert[i]) * (4 / (float)acount[i]);
    }
    tvert = avert;

    // done
    swap(tquads, quads);
    swap(tvert, vert);
  }
}
template <typename T>
std::pair<std::vector<vec4i>, std::vector<T>> subdivide_catmullclark_impl(
    const std::vector<vec4i>& quads, const std::vector<T>& vert, int level,
    bool lock_boundary) {
  auto tess = std::pair<std::vector<vec4i>, std::vector<T>>{};
  subdivide_catmullclark_impl(
      tess.first, tess.second, quads, vert, level, lock_boundary);
  return tess;
}

std::pair<std::vector<vec2i>, std::vector<float>> subdivide_lines(
    const std::vector<vec2i>& lines, const std::vector<float>& vert,
    int level) {
  return subdivide_lines_impl(lines, vert, level);
}
std::pair<std::vector<vec2i>, std::vector<vec2f>> subdivide_lines(
    const std::vector<vec2i>& lines, const std::vector<vec2f>& vert,
    int level) {
  return subdivide_lines_impl(lines, vert, level);
}
std::pair<std::vector<vec2i>, std::vector<vec3f>> subdivide_lines(
    const std::vector<vec2i>& lines, const std::vector<vec3f>& vert,
    int level) {
  return subdivide_lines_impl(lines, vert, level);
}
std::pair<std::vector<vec2i>, std::vector<vec4f>> subdivide_lines(
    const std::vector<vec2i>& lines, const std::vector<vec4f>& vert,
    int level) {
  return subdivide_lines_impl(lines, vert, level);
}

std::pair<std::vector<vec3i>, std::vector<float>> subdivide_triangles(
    const std::vector<vec3i>& triangles, const std::vector<float>& vert,
    int level) {
  return subdivide_triangles_impl(triangles, vert, level);
}
std::pair<std::vector<vec3i>, std::vector<vec2f>> subdivide_triangles(
    const std::vector<vec3i>& triangles, const std::vector<vec2f>& vert,
    int level) {
  return subdivide_triangles_impl(triangles, vert, level);
}
std::pair<std::vector<vec3i>, std::vector<vec3f>> subdivide_triangles(
    const std::vector<vec3i>& triangles, const std::vector<vec3f>& vert,
    int level) {
  return subdivide_triangles_impl(triangles, vert, level);
}
std::pair<std::vector<vec3i>, std::vector<vec4f>> subdivide_triangles(
    const std::vector<vec3i>& triangles, const std::vector<vec4f>& vert,
    int level) {
  return subdivide_triangles_impl(triangles, vert, level);
}

std::pair<std::vector<vec4i>, std::vector<float>> subdivide_quads(
    const std::vector<vec4i>& quads, const std::vector<float>& vert,
    int level) {
  return subdivide_quads_impl(quads, vert, level);
}
std::pair<std::vector<vec4i>, std::vector<vec2f>> subdivide_quads(
    const std::vector<vec4i>& quads, const std::vector<vec2f>& vert,
    int level) {
  return subdivide_quads_impl(quads, vert, level);
}
std::pair<std::vector<vec4i>, std::vector<vec3f>> subdivide_quads(
    const std::vector<vec4i>& quads, const std::vector<vec3f>& vert,
    int level) {
  return subdivide_quads_impl(quads, vert, level);
}
std::pair<std::vector<vec4i>, std::vector<vec4f>> subdivide_quads(
    const std::vector<vec4i>& quads, const std::vector<vec4f>& vert,
    int level) {
  return subdivide_quads_impl(quads, vert, level);
}

std::pair<std::vector<vec4i>, std::vector<float>> subdivide_beziers(
    const std::vector<vec4i>& beziers, const std::vector<float>& vert,
    int level) {
  return subdivide_beziers_impl(beziers, vert, level);
}
std::pair<std::vector<vec4i>, std::vector<vec2f>> subdivide_beziers(
    const std::vector<vec4i>& beziers, const std::vector<vec2f>& vert,
    int level) {
  return subdivide_beziers_impl(beziers, vert, level);
}
std::pair<std::vector<vec4i>, std::vector<vec3f>> subdivide_beziers(
    const std::vector<vec4i>& beziers, const std::vector<vec3f>& vert,
    int level) {
  return subdivide_beziers_impl(beziers, vert, level);
}
std::pair<std::vector<vec4i>, std::vector<vec4f>> subdivide_beziers(
    const std::vector<vec4i>& beziers, const std::vector<vec4f>& vert,
    int level) {
  return subdivide_beziers_impl(beziers, vert, level);
}

std::pair<std::vector<vec4i>, std::vector<float>> subdivide_catmullclark(
    const std::vector<vec4i>& quads, const std::vector<float>& vert, int level,
    bool lock_boundary) {
  return subdivide_catmullclark_impl(quads, vert, level, lock_boundary);
}
std::pair<std::vector<vec4i>, std::vector<vec2f>> subdivide_catmullclark(
    const std::vector<vec4i>& quads, const std::vector<vec2f>& vert, int level,
    bool lock_boundary) {
  return subdivide_catmullclark_impl(quads, vert, level, lock_boundary);
}
std::pair<std::vector<vec4i>, std::vector<vec3f>> subdivide_catmullclark(
    const std::vector<vec4i>& quads, const std::vector<vec3f>& vert, int level,
    bool lock_boundary) {
  return subdivide_catmullclark_impl(quads, vert, level, lock_boundary);
}
std::pair<std::vector<vec4i>, std::vector<vec4f>> subdivide_catmullclark(
    const std::vector<vec4i>& quads, const std::vector<vec4f>& vert, int level,
    bool lock_boundary) {
  return subdivide_catmullclark_impl(quads, vert, level, lock_boundary);
}

}  // namespace yocto::shape

// -----------------------------------------------------------------------------
// IMPLEMENTATION OF SHAPE SAMPLING
// -----------------------------------------------------------------------------
namespace yocto::shape {

// Pick a point in a point set uniformly.
int sample_points(int npoints, float re) { return sample_uniform(npoints, re); }
int sample_points(const std::vector<float>& cdf, float re) {
  return sample_discrete_cdf(cdf, re);
}
std::vector<float> sample_points_cdf(int npoints) {
  auto cdf = std::vector<float>(npoints);
  for (auto i = 0; i < cdf.size(); i++) cdf[i] = 1 + (i ? cdf[i - 1] : 0);
  return cdf;
}

// Pick a point on lines uniformly.
std::pair<int, float> sample_lines(
    const std::vector<float>& cdf, float re, float ru) {
  return {sample_discrete_cdf(cdf, re), ru};
}
std::vector<float> sample_lines_cdf(
    const std::vector<vec2i>& lines, const std::vector<vec3f>& positions) {
  auto cdf = std::vector<float>(lines.size());
  for (auto i = 0; i < cdf.size(); i++) {
    auto l = lines[i];
    auto w = line_length(positions[l.x], positions[l.y]);
    cdf[i] = w + (i ? cdf[i - 1] : 0);
  }
  return cdf;
}

// Pick a point on a triangle mesh uniformly.
std::pair<int, vec2f> sample_triangles(
    const std::vector<float>& cdf, float re, const vec2f& ruv) {
  return {sample_discrete_cdf(cdf, re), sample_triangle(ruv)};
}
std::vector<float> sample_triangles_cdf(
    const std::vector<vec3i>& triangles, const std::vector<vec3f>& positions) {
  auto cdf = std::vector<float>(triangles.size());
  for (auto i = 0; i < cdf.size(); i++) {
    auto t = triangles[i];
    auto w = triangle_area(positions[t.x], positions[t.y], positions[t.z]);
    cdf[i] = w + (i ? cdf[i - 1] : 0);
  }
  return cdf;
}

// Pick a point on a quad mesh uniformly.
std::pair<int, vec2f> sample_quads(
    const std::vector<float>& cdf, float re, const vec2f& ruv) {
  return {sample_discrete_cdf(cdf, re), ruv};
}
std::pair<int, vec2f> sample_quads(const std::vector<vec4i>& quads,
    const std::vector<float>& cdf, float re, const vec2f& ruv) {
  auto element = sample_discrete_cdf(cdf, re);
  if (quads[element].z == quads[element].w) {
    return {element, sample_triangle(ruv)};
  } else {
    return {element, ruv};
  }
}
std::vector<float> sample_quads_cdf(
    const std::vector<vec4i>& quads, const std::vector<vec3f>& positions) {
  auto cdf = std::vector<float>(quads.size());
  for (auto i = 0; i < cdf.size(); i++) {
    auto q = quads[i];
    auto w = quad_area(
        positions[q.x], positions[q.y], positions[q.z], positions[q.w]);
    cdf[i] = w + (i ? cdf[i - 1] : 0);
  }
  return cdf;
}

// Samples a set of points over a triangle mesh uniformly. The rng function
// takes the point index and returns vec3f numbers uniform directibuted in
// [0,1]^3. unorm and texcoord are optional.
void sample_triangles(std::vector<vec3f>& sampled_positions,
    std::vector<vec3f>& sampled_normals, std::vector<vec2f>& sampled_texcoords,
    const std::vector<vec3i>& triangles, const std::vector<vec3f>& positions,
    const std::vector<vec3f>& normals, const std::vector<vec2f>& texcoords,
    int npoints, int seed) {
  sampled_positions.resize(npoints);
  sampled_normals.resize(npoints);
  sampled_texcoords.resize(npoints);
  auto cdf = sample_triangles_cdf(triangles, positions);
  auto rng = make_rng(seed);
  for (auto i = 0; i < npoints; i++) {
    auto  sample         = sample_triangles(cdf, rand1f(rng), rand2f(rng));
    auto& t              = triangles[sample.first];
    auto  uv             = sample.second;
    sampled_positions[i] = interpolate_triangle(
        positions[t.x], positions[t.y], positions[t.z], uv);
    if (!sampled_normals.empty()) {
      sampled_normals[i] = normalize(
          interpolate_triangle(normals[t.x], normals[t.y], normals[t.z], uv));
    } else {
      sampled_normals[i] = triangle_normal(
          positions[t.x], positions[t.y], positions[t.z]);
    }
    if (!sampled_texcoords.empty()) {
      sampled_texcoords[i] = interpolate_triangle(
          texcoords[t.x], texcoords[t.y], texcoords[t.z], uv);
    } else {
      sampled_texcoords[i] = zero2f;
    }
  }
}

// Samples a set of points over a triangle mesh uniformly. The rng function
// takes the point index and returns vec3f numbers uniform directibuted in
// [0,1]^3. unorm and texcoord are optional.
void sample_quads(std::vector<vec3f>& sampled_positions,
    std::vector<vec3f>& sampled_normals, std::vector<vec2f>& sampled_texcoords,
    const std::vector<vec4i>& quads, const std::vector<vec3f>& positions,
    const std::vector<vec3f>& normals, const std::vector<vec2f>& texcoords,
    int npoints, int seed) {
  sampled_positions.resize(npoints);
  sampled_normals.resize(npoints);
  sampled_texcoords.resize(npoints);
  auto cdf = sample_quads_cdf(quads, positions);
  auto rng = make_rng(seed);
  for (auto i = 0; i < npoints; i++) {
    auto  sample         = sample_quads(cdf, rand1f(rng), rand2f(rng));
    auto& q              = quads[sample.first];
    auto  uv             = sample.second;
    sampled_positions[i] = interpolate_quad(
        positions[q.x], positions[q.y], positions[q.z], positions[q.w], uv);
    if (!sampled_normals.empty()) {
      sampled_normals[i] = normalize(interpolate_quad(
          normals[q.x], normals[q.y], normals[q.z], normals[q.w], uv));
    } else {
      sampled_normals[i] = quad_normal(
          positions[q.x], positions[q.y], positions[q.z], positions[q.w]);
    }
    if (!sampled_texcoords.empty()) {
      sampled_texcoords[i] = interpolate_quad(
          texcoords[q.x], texcoords[q.y], texcoords[q.z], texcoords[q.w], uv);
    } else {
      sampled_texcoords[i] = zero2f;
    }
  }
}

}  // namespace yocto::shape

// -----------------------------------------------------------------------------
// SHAPE GEODESICS
// -----------------------------------------------------------------------------
namespace yocto::shape {

static inline void connect_nodes(
    geodesic_solver& solver, int a, int b, float length) {
  solver.graph[a].push_back({b, length});
  solver.graph[b].push_back({a, length});
}

static inline float opposite_nodes_arc_length(geodesic_solver& solver,
    const std::vector<vec3f>& positions, int a, int c, const vec2i& edge) {
  // Triangles (a, b, d) and (b, d, c) are connected by (b, d) edge
  // Nodes a and c must be connected.

  int  b  = edge.x;
  int  d  = edge.y;
  auto ba = positions[a] - positions[b];
  auto bc = positions[c] - positions[b];
  auto bd = positions[d] - positions[b];

  float cos_alpha = dot(normalize(ba), normalize(bd));
  float cos_beta  = dot(normalize(bc), normalize(bd));
  float sin_alpha = sqrt(max(0.0f, 1 - cos_alpha * cos_alpha));
  float sin_beta  = sqrt(max(0.0f, 1 - cos_beta * cos_beta));

  // cos(alpha + beta)
  float cos_alpha_beta = cos_alpha * cos_beta - sin_alpha * sin_beta;
  if (cos_alpha_beta <= -1) return flt_max;

  // law of cosines (generalized Pythagorean theorem)
  float len = dot(ba, ba) + dot(bc, bc) -
              length(ba) * length(bc) * 2 * cos_alpha_beta;

  if (len <= 0)
    return flt_max;
  else
    return sqrtf(len);
}

static inline void connect_opposite_nodes(geodesic_solver& solver,
    const std::vector<vec3f>& positions, const vec3i& tr0, const vec3i& tr1,
    const vec2i& edge) {
  auto opposite_vertex = [](const vec3i& tr, const vec2i& edge) -> int {
    for (int i = 0; i < 3; ++i) {
      if (tr[i] != edge.x && tr[i] != edge.y) return tr[i];
    }
    return -1;
  };

  int v0 = opposite_vertex(tr0, edge);
  int v1 = opposite_vertex(tr1, edge);
  if (v0 == -1 || v1 == -1) return;
  auto length = opposite_nodes_arc_length(solver, positions, v0, v1, edge);
  connect_nodes(solver, v0, v1, length);
}

geodesic_solver make_geodesic_solver(const std::vector<vec3i>& triangles,
    const std::vector<vec3i>&                                  adjacencies,
    const std::vector<vec3f>&                                  positions) {
  auto solver = geodesic_solver{};
  solver.graph.resize(positions.size());
  for (int face = 0; face < triangles.size(); face++) {
    for (int k = 0; k < 3; k++) {
      auto a = triangles[face][k];
      auto b = triangles[face][(k + 1) % 3];

      // connect mesh edges
      auto len = length(positions[a] - positions[b]);
      if (a < b) connect_nodes(solver, a, b, len);

      // connect opposite nodes
      auto neighbor = adjacencies[face][k];
      if (face < neighbor) {
        connect_opposite_nodes(
            solver, positions, triangles[face], triangles[neighbor], {a, b});
      }
    }
  }
  return solver;
}

// `update` is a function that is executed during expansion, every time a node
// is put into queue. `exit` is a function that tells whether to expand the
// current node or perform early exit.
template <typename Update, typename Exit>
void visit_geodesic_graph(std::vector<float>& field,
    const geodesic_solver& solver, const std::vector<int>& sources,
    Update&& update, Exit&& exit) {
  /*
     This algortithm uses the heuristic Small Label Fisrt and Large Label Last
     https://en.wikipedia.org/wiki/Shortest_Path_Faster_Algorithm

     Large Label Last (LLL): When extracting nodes from the queue, pick the
     front one. If it weights more than the average weight of the queue, put
     on the back and check the next node. Continue this way.
     Sometimes average_weight is less than every value due to floating point
     errors (doesn't happen with double precision).

     Small Label First (SLF): When adding a new node to queue, instead of
     always pushing it to the end of the queue, if it weights less than the
     front node of the queue, it is put on front. Otherwise the node is put at
     the end of the queue.
  */

  auto in_queue = std::vector<bool>(solver.graph.size(), false);

  // setup queue
  auto queue = std::deque<int>();
  for (auto source : sources) {
    in_queue[source] = true;
    queue.push_back(source);
  }

  // Cumulative weights of elements in queue. Used to keep track of the
  // average weight of the queue.
  double cumulative_weight = 0.0;

  while (!queue.empty()) {
    auto node           = queue.front();
    auto average_weight = (float)cumulative_weight / queue.size();

    // Large Label Last (see comment at the beginning)
    for (auto tries = 0; tries < queue.size() + 1; tries++) {
      if (field[node] <= average_weight) break;
      queue.pop_front();
      queue.push_back(node);
      node = queue.front();
    }

    // Remove node from queue.
    queue.pop_front();
    in_queue[node] = false;
    cumulative_weight -= field[node];

    // Check early exit condition.
    if (exit(node)) continue;

    for (int i = 0; i < solver.graph[node].size(); i++) {
      // Distance of neighbor through this node
      auto new_distance = field[node] + solver.graph[node][i].length;
      auto neighbor     = solver.graph[node][i].node;

      auto old_distance = field[neighbor];
      if (new_distance >= old_distance) continue;

      if (in_queue[neighbor]) {
        // If neighbor already in queue, don't add it.
        // Just update cumulative weight.
        cumulative_weight += new_distance - old_distance;
      } else {
        // If neighbor not in queue, add node to queue using Small Label
        // First (see comment at the beginning).
        if (queue.empty() || (new_distance < field[queue.front()]))
          queue.push_front(neighbor);
        else
          queue.push_back(neighbor);

        // Update queue information.
        in_queue[neighbor] = true;
        cumulative_weight += new_distance;
      }

      // Update distance of neighbor.
      field[neighbor] = new_distance;
      update(node, neighbor, new_distance);
    }
  }
}

// Compute geodesic distances
void update_geodesic_distances(std::vector<float>& distances,
    const geodesic_solver& solver, const std::vector<int>& sources,
    float max_distance) {
  auto update = [](int node, int neighbor, float new_distance) {};
  auto exit   = [&](int node) { return distances[node] > max_distance; };
  visit_geodesic_graph(distances, solver, sources, update, exit);
}

std::vector<float> compute_geodesic_distances(const geodesic_solver& solver,
    const std::vector<int>& sources, float max_distance) {
  auto distances = std::vector<float>(solver.graph.size(), flt_max);
  for (auto source : sources) distances[source] = 0.0f;
  update_geodesic_distances(distances, solver, sources, max_distance);
  return distances;
}

// Compute all shortest paths from source vertices to any other vertex.
// Paths are implicitly represented: each node is assigned its previous node in
// the path. Graph search early exits when reching end_vertex.
std::vector<int> compute_geodesic_paths(const geodesic_solver& solver,
    const std::vector<int>& sources, int end_vertex) {
  auto parents   = std::vector<int>(solver.graph.size(), -1);
  auto distances = std::vector<float>(solver.graph.size(), flt_max);
  auto update    = [&parents](int node, int neighbor, float new_distance) {
    parents[neighbor] = node;
  };
  auto exit = [end_vertex](int node) { return node == end_vertex; };
  for (auto source : sources) distances[source] = 0.0f;
  visit_geodesic_graph(distances, solver, sources, update, exit);
  return parents;
}

// Sample vertices with a Poisson distribution using geodesic distances
// Sampling strategy is farthest point sampling (FPS): at every step
// take the farthers point from current sampled set until done.
std::vector<int> sample_vertices_poisson(
    const geodesic_solver& solver, int num_samples) {
  auto verts = std::vector<int>{};
  verts.reserve(num_samples);
  auto distances = std::vector<float>(solver.graph.size(), flt_max);
  while (true) {
    auto max_index =
        (int)(std::max_element(distances.begin(), distances.end()) -
              distances.begin());
    verts.push_back(max_index);
    if (verts.size() >= num_samples) break;
    distances[max_index] = 0.0f;
    update_geodesic_distances(distances, solver, {max_index}, flt_max);
  }
  return verts;
}

// Compute the distance field needed to compute a voronoi diagram
std::vector<std::vector<float>> compute_voronoi_fields(
    const geodesic_solver& solver, const std::vector<int>& generators) {
  auto fields = std::vector<std::vector<float>>(generators.size());

  // Find max distance from a generator to set an early exit condition for the
  // following distance field computations. This optimization makes computation
  // time weakly dependant on the number of generators.
  auto total = compute_geodesic_distances(solver, generators);
  auto max   = *std::max_element(total.begin(), total.end());
  // @Speed: use parallel_for
  for (int i = 0; i < generators.size(); ++i) {
    fields[i]                = std::vector<float>(solver.graph.size(), flt_max);
    fields[i][generators[i]] = 0;
    fields[i] = compute_geodesic_distances(solver, {generators[i]}, max);
  };
  return fields;
}

std::vector<vec3f> colors_from_field(const std::vector<float>& field,
    float scale, const vec3f& c0, const vec3f& c1) {
  auto colors = std::vector<vec3f>{field.size()};
  for (auto i = 0; i < colors.size(); i++) {
    colors[i] = ((int64_t)(field[i] * scale)) % 2 ? c0 : c1;
  }
  return colors;
}

}  // namespace yocto::shape

// -----------------------------------------------------------------------------
// IMPLEMENTATION OF INTEGRAL PATHS
// -----------------------------------------------------------------------------
namespace yocto::shape {

namespace integral_paths {

static int adjacent_face(const std::vector<vec3i>& triangles,
    const std::vector<vec3i>& adjacency, int face, const vec2i& edge) {
  // Given a face and an edge, return the adjacent face
  for (int k = 0; k < 3; ++k) {
    auto x = triangles[face][k];
    auto y = triangles[face][k < 2 ? k + 1 : 0];
    auto e = vec2i{x, y};
    if (e == edge) return adjacency[face][k];
    if (vec2i{e.y, e.x} == edge) return adjacency[face][k];
  }
  return -1;
}

static std::vector<int> get_face_ring(const std::vector<vec3i>& triangles,
    const std::vector<vec3i>& adjacency, int face, int vertex) {
  auto contains = [](const vec3i& v, int a) -> bool {
    return v.x == a || v.y == a || v.z == a;
  };
  auto containsv = [](const std::vector<int>& v, int a) -> bool {
    return find(v.begin(), v.end(), a) != v.end();
  };

  auto result = std::vector<int>();
  result.reserve(12);

  if (face == -1) {
    for (int i = 0; i < triangles.size(); ++i) {
      if (contains(triangles[i], vertex)) {
        face = i;
        break;
      }
    }
  }

  auto queue = std::vector<int>();
  queue.push_back(face);

  while (!queue.empty()) {
    int f = queue.back();
    queue.pop_back();
    result.push_back(f);

    for (int k = 0; k < 3; ++k) {
      auto edge = vec2i{triangles[f][k], triangles[f][(k + 1) % 3]};
      if (edge.x == vertex || edge.y == vertex) {
        int neighbor_face = adjacency[f][k];
        if (neighbor_face == -1) continue;
        if (!containsv(result, neighbor_face)) {
          queue.push_back(neighbor_face);
          result.push_back(neighbor_face);
        }
      }
    }
  }

  return result;
}

static float step_from_point_to_edge(const vec3f& right, const vec3f& left,
    const vec3f& direction, float epsilon = 0.0001f) {
  auto normal            = cross(right, left);
  auto inverse_transform = mat3f{right - left, left, normal};
  auto transform         = inverse(inverse_transform);
  auto dir               = transform * direction;
  return clamp(1 - dir.x / dir.y, 0.0f + epsilon, 1.0f - epsilon);
}
static std::pair<float, bool> step_from_edge_to_edge(const vec3f& point,
    const vec3f& a, const vec3f& b, const vec3f& c, const vec3f& direction,
    float epsilon = 0.0001f) {
  //      b
  //     /\
  //    /  \
  //   /    \
  //  /______\
  // c        a

  auto right  = a - point;
  auto left   = c - point;
  auto front  = b - point;
  auto normal = triangle_normal(a, b, c);

  auto right_side = dot(cross(direction, front), normal) > 0;
  if (right_side) {
    auto below_edge = dot(cross(direction, right), normal) > 0;
    if (below_edge) return {epsilon, true};

    auto x = step_from_point_to_edge(right, front, direction);
    return {x, true};
  } else {
    auto below_edge = dot(cross(left, direction), normal) > 0;
    if (below_edge) return {1.0f - epsilon, false};

    auto x = step_from_point_to_edge(front, left, direction);
    return {x, false};
  }
}

static surface_path::vertex step_from_point(const std::vector<vec3i>& triangles,
    const std::vector<vec3f>& positions, const std::vector<vec3i>& adjacency,
    const std::vector<int>& tags, const std::vector<float>& field, int vertex,
    int start_face, int tag = -1, float epsilon = 0.0001f) {
  auto opposite_edge = [](int vertex, const vec3i& tr) -> vec2i {
    for (int i = 0; i < 3; ++i) {
      if (tr[i] == vertex) return {tr[(i + 1) % 3], tr[(i + 2) % 3]};
    }
    return {-1, -1};
  };
  auto is_direction_inbetween = [](const vec3f& right, const vec3f& left,
                                    const vec3f& direction,
                                    float        threshold = 0) -> bool {
    auto normal      = cross(right, left);
    auto cross_right = cross(right, direction);
    auto cross_left  = cross(direction, left);
    return dot(cross_right, normal) > threshold &&
           dot(cross_left, normal) > threshold;
  };

  auto triangle_fan = get_face_ring(triangles, adjacency, start_face, vertex);

  auto best_alignment = 0.0;
  auto fallback_lerp  = surface_path::vertex{vec2i{-1, -1}, -1, 0};

  for (auto i = 0; i < triangle_fan.size(); ++i) {
    auto face = triangle_fan[i];
    if (tag != -1 && tags[face] != tag) continue;
    auto edge = opposite_edge(vertex, triangles[face]);
    if (edge == vec2i{-1, -1}) throw std::runtime_error("edge is {-1, -1}\n");

    auto a         = positions[vertex];
    auto b         = positions[edge.x];
    auto c         = positions[edge.y];
    auto left      = c - a;
    auto right     = b - a;
    auto direction = normalize(
        compute_gradient(triangles[face], positions, field));

    auto right_dot = dot(normalize(right), direction);
    auto left_dot  = dot(normalize(left), direction);

    // Look for the most aligned edge with the gradient
    // direction. If no other face is suitable for selection, we return the
    // most aligned edge as result (fallback_lerp)
    if (max(right_dot, left_dot) > best_alignment) {
      best_alignment = max(right_dot, left_dot);
      auto alpha     = right_dot > left_dot ? 0.0f + epsilon : 1.0f - epsilon;
      fallback_lerp  = surface_path::vertex{edge, face, alpha};
    }

    // Check if gradient direction is in between ac and ab.
    if (is_direction_inbetween(right, left, direction)) {
      auto alpha = step_from_point_to_edge(right, left, direction);
      return surface_path::vertex{edge, face, alpha};
    }
  }

  return fallback_lerp;
}

surface_path integrate_field(const std::vector<vec3i>& triangles,
    const std::vector<vec3f>& positions, const std::vector<vec3i>& adjacency,
    const std::vector<int>& tags, int tag, const std::vector<float>& field,
    int from) {
  auto opposite_vertex = [](const vec3i& tr, const vec2i& edge) -> int {
    for (int i = 0; i < 3; ++i) {
      if (tr[i] != edge.x && tr[i] != edge.y) return tr[i];
    }
    return -1;
  };
  auto find_index = [](const vec3i& v, int x) -> int {
    if (v.x == x) return 0;
    if (v.y == x) return 1;
    if (v.z == x) return 2;
    return -1;
  };

  // trace function
  auto lerps = std::vector<surface_path::vertex>();
  auto lerp  = step_from_point(
      triangles, positions, adjacency, tags, field, from, -1, tag);
  if (lerp.face == -1) return {};
  lerps.push_back(lerp);

  const int num_steps = 10000;

  for (auto i = 0; i < num_steps; i++) {
    auto [old_edge, old_face, old_alpha] = lerps.back();
    if (old_face == -1) throw std::runtime_error("programmer error");
    auto point = (1.0f - old_alpha) * positions[old_edge.x] +
                 old_alpha * positions[old_edge.y];

    auto face = adjacent_face(triangles, adjacency, old_face, old_edge);
    if (face == -1) {
      lerps.push_back({vec2i{-1, -1}, face, 0});
      return surface_path{from, -1, lerps};
    }

    if (tags[face] != tag) {
      auto k  = find_index(triangles[face], old_edge.x);
      auto to = triangles[face][(k + 1) % 3];

      // @Hack!: We store the tag of the reached region in edge.x
      auto edge = vec2i{to, tags[face]};
      lerps.push_back({edge, face, 0});
      return surface_path{from, to, lerps};
    }

    auto direction = normalize(
        compute_gradient(triangles[face], positions, field));

    auto front_idx = opposite_vertex(triangles[face], old_edge);
    if (front_idx == -1) {
      throw std::runtime_error("programmer error");
      return {};
    }

    if (old_alpha < 0 || old_alpha > 1)
      throw std::runtime_error("programmer error");

    auto& a = positions[old_edge.x];
    auto& b = positions[front_idx];
    auto& c = positions[old_edge.y];

    auto [x, step_right] = step_from_edge_to_edge(point, a, b, c, direction);

    auto edge = old_edge;
    if (step_right) {
      point  = (1 - x) * a + x * b;
      edge.y = front_idx;
    } else {
      point  = (1 - x) * b + x * c;
      edge.x = front_idx;
    }
    lerps.push_back({edge, face, x});
  }

  throw std::runtime_error("integral path ended nowhere");
  return surface_path{from, 0, lerps};
}

surface_path integrate_field(const std::vector<vec3i>& triangles,
    const std::vector<vec3f>& positions, const std::vector<vec3i>& adjacency,
    const std::vector<int>& tags, int tag, const std::vector<float>& field,
    int from, int to) {
  auto opposite_vertex = [](const vec3i& tr, const vec2i& edge) -> int {
    for (int i = 0; i < 3; ++i) {
      if (tr[i] != edge.x && tr[i] != edge.y) return tr[i];
    }
    return -1;
  };
  auto contains = [](const vec3i& v, int a) -> bool {
    return v.x == a || v.y == a || v.z == a;
  };

  auto lerps = std::vector<surface_path::vertex>();

  lerps.push_back(
      step_from_point(triangles, positions, adjacency, tags, field, from, -1));

  const int num_steps = 10000;

  for (int i = 0; i < num_steps; i++) {
    auto [old_edge, old_face, old_alpha] = lerps.back();
    vec3f point = (1.0f - old_alpha) * positions[old_edge.x] +
                  old_alpha * positions[old_edge.y];

    auto face = adjacent_face(triangles, adjacency, old_face, old_edge);
    if (face == -1) break;

    if (contains(triangles[face], to)) {
      for (int k = 0; k < 3; ++k) {
        auto edge = vec2i{triangles[face][k], triangles[face][(k + 1) % 3]};
        if (edge.x == to) {
          lerps.push_back({edge, face, 0});
          return {from, to, lerps};
        }
      }
    }
    auto direction = normalize(
        compute_gradient(triangles[face], positions, field));

    int front_idx = opposite_vertex(triangles[face], old_edge);
    if (front_idx == -1) {
      throw std::runtime_error("programmer error: front_idx is -1");
      break;
    }

    if (old_alpha < 0 || old_alpha > 1)
      throw std::runtime_error("programmer error");
    if (old_alpha == 0 || old_alpha == 1) {
      int  vertex = old_alpha == 0 ? old_edge.x : old_edge.y;
      auto lerp   = step_from_point(
          triangles, positions, adjacency, tags, field, vertex, old_face);
      lerps.push_back(lerp);
      if (lerp.alpha == 0 && lerp.edge.x == to) break;
      if (lerp.alpha == 1 && lerp.edge.y == to) break;
      continue;
    }

    auto& a = positions[old_edge.x];
    auto& b = positions[front_idx];
    auto& c = positions[old_edge.y];

    auto [x, step_right] = step_from_edge_to_edge(point, a, b, c, direction);

    auto edge = old_edge;
    if (step_right) {
      point  = (1 - x) * a + x * b;
      edge.y = front_idx;
    } else {
      point  = (1 - x) * b + x * c;
      edge.x = front_idx;
    }
    if (opposite_vertex(triangles[face], edge) == -1) {
      throw std::runtime_error("opposite vertex == -1");
    }

    lerps.push_back({edge, face, x});
    if (x == 0 && edge.x == to) break;
    if (x == 1 && edge.y == to) break;
  }

  return {from, to, lerps};
}

std::vector<vec3f> make_positions_from_path(
    const surface_path& path, const std::vector<vec3f>& mesh_positions) {
  if (path.vertices.empty()) return {};

  auto positions = std::vector<vec3f>();
  positions.reserve(path.vertices.size() + 1);
  positions.push_back(mesh_positions[path.start]);

  for (int i = 0; i < path.vertices.size() - 1; ++i) {
    auto [edge, face, x] = path.vertices[i];
    auto p0              = mesh_positions[edge.x];
    auto p1              = mesh_positions[edge.y];
    auto position        = (1 - x) * p0 + x * p1;
    positions.push_back(position);
  }

  if (path.end != -1) {
    positions.push_back(mesh_positions[path.end]);
  }
  return positions;
}

}  // namespace integral_paths

// Trace integral path following the gradient of a scalar field
surface_path integrate_field(const std::vector<vec3i>& triangles,
    const std::vector<vec3f>& positions, const std::vector<vec3i>& adjacency,
    const std::vector<int>& tags, int tag, const std::vector<float>& field,
    int from) {
  return integral_paths::integrate_field(
      triangles, positions, adjacency, tags, tag, field, from);
}
surface_path integrate_field(const std::vector<vec3i>& triangles,
    const std::vector<vec3f>& positions, const std::vector<vec3i>& adjacency,
    const std::vector<int>& tags, int tag, const std::vector<float>& field,
    int from, int to) {
  return integral_paths::integrate_field(
      triangles, positions, adjacency, tags, tag, field, from, to);
}

std::vector<vec3f> make_positions_from_path(
    const surface_path& path, const std::vector<vec3f>& mesh_positions) {
  return integral_paths::make_positions_from_path(path, mesh_positions);
}

vec3f compute_gradient(const vec3i& triangle,
    const std::vector<vec3f>& positions, const std::vector<float>& field) {
  auto xy     = positions[triangle.y] - positions[triangle.x];
  auto yz     = positions[triangle.z] - positions[triangle.y];
  auto zx     = positions[triangle.x] - positions[triangle.z];
  auto normal = normalize(cross(zx, xy));
  auto result = zero3f;
  result += field[triangle.x] * cross(normal, yz);
  result += field[triangle.y] * cross(normal, zx);
  result += field[triangle.z] * cross(normal, xy);
  return result;
}

}  // namespace yocto::shape

// -----------------------------------------------------------------------------
// IMPLEMENTATION OF SHAPE EXAMPLES
// -----------------------------------------------------------------------------
namespace yocto::shape {

// Make a quad.
void make_rect(std::vector<vec4i>& quads, std::vector<vec3f>& positions,
    std::vector<vec3f>& normals, std::vector<vec2f>& texcoords,
    const vec2i& steps, const vec2f& scale, const vec2f& uvscale) {
  positions.resize((steps.x + 1) * (steps.y + 1));
  normals.resize((steps.x + 1) * (steps.y + 1));
  texcoords.resize((steps.x + 1) * (steps.y + 1));
  for (auto j = 0; j <= steps.y; j++) {
    for (auto i = 0; i <= steps.x; i++) {
      auto uv = vec2f{i / (float)steps.x, j / (float)steps.y};
      positions[j * (steps.x + 1) + i] = {
          (2 * uv.x - 1) * scale.x, (2 * uv.y - 1) * scale.y, 0};
      normals[j * (steps.x + 1) + i]   = {0, 0, 1};
      texcoords[j * (steps.x + 1) + i] = vec2f{uv.x, 1 - uv.y} * uvscale;
    }
  }

  quads.resize(steps.x * steps.y);
  for (auto j = 0; j < steps.y; j++) {
    for (auto i = 0; i < steps.x; i++) {
      quads[j * steps.x + i] = {j * (steps.x + 1) + i,
          j * (steps.x + 1) + i + 1, (j + 1) * (steps.x + 1) + i + 1,
          (j + 1) * (steps.x + 1) + i};
    }
  }
}

void make_bulged_rect(std::vector<vec4i>& quads, std::vector<vec3f>& positions,
    std::vector<vec3f>& normals, std::vector<vec2f>& texcoords,
    const vec2i& steps, const vec2f& scale, const vec2f& uvscale,
    float height) {
  make_rect(quads, positions, normals, texcoords, steps, scale, uvscale);
  if (height) {
    height      = min(height, min(scale));
    auto radius = (1 + height * height) / (2 * height);
    auto center = vec3f{0, 0, -radius + height};
    for (auto i = 0; i < positions.size(); i++) {
      auto pn      = normalize(positions[i] - center);
      positions[i] = center + pn * radius;
      normals[i]   = pn;
    }
  }
}

void make_recty(std::vector<vec4i>& quads, std::vector<vec3f>& positions,
    std::vector<vec3f>& normals, std::vector<vec2f>& texcoords,
    const vec2i& steps, const vec2f& scale, const vec2f& uvscale) {
  make_rect(quads, positions, normals, texcoords, steps, scale, uvscale);
  for (auto& p : positions) {
    std::swap(p.y, p.z);
    p.z = -p.z;
  }
  for (auto& n : normals) std::swap(n.y, n.z);
}

// Make a cube.
void make_box(std::vector<vec4i>& quads, std::vector<vec3f>& positions,
    std::vector<vec3f>& normals, std::vector<vec2f>& texcoords,
    const vec3i& steps, const vec3f& scale, const vec3f& uvscale) {
  quads.clear();
  positions.clear();
  normals.clear();
  texcoords.clear();
  auto qquads         = std::vector<vec4i>{};
  auto qpositions     = std::vector<vec3f>{};
  auto qnormals       = std::vector<vec3f>{};
  auto qtexturecoords = std::vector<vec2f>{};
  // + z
  make_rect(qquads, qpositions, qnormals, qtexturecoords, {steps.x, steps.y},
      {scale.x, scale.y}, {uvscale.x, uvscale.y});
  for (auto& p : qpositions) p = {p.x, p.y, scale.z};
  for (auto& n : qnormals) n = {0, 0, 1};
  merge_quads(quads, positions, normals, texcoords, qquads, qpositions,
      qnormals, qtexturecoords);
  // - z
  make_rect(qquads, qpositions, qnormals, qtexturecoords, {steps.x, steps.y},
      {scale.x, scale.y}, {uvscale.x, uvscale.y});
  for (auto& p : qpositions) p = {-p.x, p.y, -scale.z};
  for (auto& n : qnormals) n = {0, 0, -1};
  merge_quads(quads, positions, normals, texcoords, qquads, qpositions,
      qnormals, qtexturecoords);
  // + x
  make_rect(qquads, qpositions, qnormals, qtexturecoords, {steps.z, steps.y},
      {scale.z, scale.y}, {uvscale.z, uvscale.y});
  for (auto& p : qpositions) p = {scale.x, p.y, -p.x};
  for (auto& n : qnormals) n = {1, 0, 0};
  merge_quads(quads, positions, normals, texcoords, qquads, qpositions,
      qnormals, qtexturecoords);
  // - x
  make_rect(qquads, qpositions, qnormals, qtexturecoords, {steps.z, steps.y},
      {scale.z, scale.y}, {uvscale.z, uvscale.y});
  for (auto& p : qpositions) p = {-scale.x, p.y, p.x};
  for (auto& n : qnormals) n = {-1, 0, 0};
  merge_quads(quads, positions, normals, texcoords, qquads, qpositions,
      qnormals, qtexturecoords);
  // + y
  make_rect(qquads, qpositions, qnormals, qtexturecoords, {steps.x, steps.z},
      {scale.x, scale.z}, {uvscale.x, uvscale.z});
  for (auto i = 0; i < qpositions.size(); i++) {
    qpositions[i] = {qpositions[i].x, scale.y, -qpositions[i].y};
    qnormals[i]   = {0, 1, 0};
  }
  merge_quads(quads, positions, normals, texcoords, qquads, qpositions,
      qnormals, qtexturecoords);
  // - y
  make_rect(qquads, qpositions, qnormals, qtexturecoords, {steps.x, steps.z},
      {scale.x, scale.z}, {uvscale.x, uvscale.z});
  for (auto i = 0; i < qpositions.size(); i++) {
    qpositions[i] = {qpositions[i].x, -scale.y, qpositions[i].y};
    qnormals[i]   = {0, -1, 0};
  }
  merge_quads(quads, positions, normals, texcoords, qquads, qpositions,
      qnormals, qtexturecoords);
}

void make_rounded_box(std::vector<vec4i>& quads, std::vector<vec3f>& positions,
    std::vector<vec3f>& normals, std::vector<vec2f>& texcoords,
    const vec3i& steps, const vec3f& scale, const vec3f& uvscale,
    float radius) {
  make_box(quads, positions, normals, texcoords, steps, scale, uvscale);
  if (radius) {
    radius = min(radius, min(scale));
    auto c = scale - radius;
    for (auto i = 0; i < positions.size(); i++) {
      auto pc = vec3f{
          abs(positions[i].x), abs(positions[i].y), abs(positions[i].z)};
      auto ps = vec3f{positions[i].x < 0 ? -1.0f : 1.0f,
          positions[i].y < 0 ? -1.0f : 1.0f, positions[i].z < 0 ? -1.0f : 1.0f};
      if (pc.x >= c.x && pc.y >= c.y && pc.z >= c.z) {
        auto pn      = normalize(pc - c);
        positions[i] = c + radius * pn;
        normals[i]   = pn;
      } else if (pc.x >= c.x && pc.y >= c.y) {
        auto pn      = normalize((pc - c) * vec3f{1, 1, 0});
        positions[i] = {c.x + radius * pn.x, c.y + radius * pn.y, pc.z};
        normals[i]   = pn;
      } else if (pc.x >= c.x && pc.z >= c.z) {
        auto pn      = normalize((pc - c) * vec3f{1, 0, 1});
        positions[i] = {c.x + radius * pn.x, pc.y, c.z + radius * pn.z};
        normals[i]   = pn;
      } else if (pc.y >= c.y && pc.z >= c.z) {
        auto pn      = normalize((pc - c) * vec3f{0, 1, 1});
        positions[i] = {pc.x, c.y + radius * pn.y, c.z + radius * pn.z};
        normals[i]   = pn;
      } else {
        continue;
      }
      positions[i] *= ps;
      normals[i] *= ps;
    }
  }
}

void make_rect_stack(std::vector<vec4i>& quads, std::vector<vec3f>& positions,
    std::vector<vec3f>& normals, std::vector<vec2f>& texcoords,
    const vec3i& steps, const vec3f& scale, const vec2f& uvscale) {
  auto qquads         = std::vector<vec4i>{};
  auto qpositions     = std::vector<vec3f>{};
  auto qnormals       = std::vector<vec3f>{};
  auto qtexturecoords = std::vector<vec2f>{};
  for (auto i = 0; i <= steps.z; i++) {
    make_rect(qquads, qpositions, qnormals, qtexturecoords, {steps.x, steps.y},
        {scale.x, scale.y}, uvscale);
    for (auto& p : qpositions) p.z = (-1 + 2 * (float)i / steps.z) * scale.z;
    merge_quads(quads, positions, normals, texcoords, qquads, qpositions,
        qnormals, qtexturecoords);
  }
}

void make_floor(std::vector<vec4i>& quads, std::vector<vec3f>& positions,
    std::vector<vec3f>& normals, std::vector<vec2f>& texcoords,
    const vec2i& steps, const vec2f& scale, const vec2f& uvscale) {
  make_rect(quads, positions, normals, texcoords, steps, scale, uvscale);
  for (auto& p : positions) {
    std::swap(p.y, p.z);
    p.z = -p.z;
  }
  for (auto& n : normals) std::swap(n.y, n.z);
}

void make_bent_floor(std::vector<vec4i>& quads, std::vector<vec3f>& positions,
    std::vector<vec3f>& normals, std::vector<vec2f>& texcoords,
    const vec2i& steps, const vec2f& scale, const vec2f& uvscale,
    float radius) {
  make_floor(quads, positions, normals, texcoords, steps, scale, uvscale);
  if (radius) {
    radius     = min(radius, scale.y);
    auto start = (scale.y - radius) / 2;
    auto end   = start + radius;
    for (auto i = 0; i < positions.size(); i++) {
      if (positions[i].z < -end) {
        positions[i] = {positions[i].x, -positions[i].z - end + radius, -end};
        normals[i]   = {0, 0, 1};
      } else if (positions[i].z < -start && positions[i].z >= -end) {
        auto phi     = (pif / 2) * (-positions[i].z - start) / radius;
        positions[i] = {positions[i].x, -cos(phi) * radius + radius,
            -sin(phi) * radius - start};
        normals[i]   = {0, cos(phi), sin(phi)};
      } else {
      }
    }
  }
}

// Generate a sphere
void make_sphere(std::vector<vec4i>& quads, std::vector<vec3f>& positions,
    std::vector<vec3f>& normals, std::vector<vec2f>& texcoords, int steps,
    float scale, float uvscale) {
  make_box(quads, positions, normals, texcoords, {steps, steps, steps},
      {scale, scale, scale}, {uvscale, uvscale, uvscale});
  for (auto& p : positions) p = normalize(p) * scale;
  normals = positions;
  for (auto& n : normals) n = normalize(n);
}

// Generate a uvsphere
void make_uvsphere(std::vector<vec4i>& quads, std::vector<vec3f>& positions,
    std::vector<vec3f>& normals, std::vector<vec2f>& texcoords,
    const vec2i& steps, float scale, const vec2f& uvscale) {
  make_rect(quads, positions, normals, texcoords, steps, {1, 1}, {1, 1});
  for (auto i = 0; i < positions.size(); i++) {
    auto uv      = texcoords[i];
    auto a       = vec2f{2 * pif * uv.x, pif * (1 - uv.y)};
    positions[i] = vec3f{cos(a.x) * sin(a.y), sin(a.x) * sin(a.y), cos(a.y)} *
                   scale;
    normals[i]   = normalize(positions[i]);
    texcoords[i] = uv * uvscale;
  }
}

void make_capped_uvsphere(std::vector<vec4i>& quads,
    std::vector<vec3f>& positions, std::vector<vec3f>& normals,
    std::vector<vec2f>& texcoords, const vec2i& steps, float scale,
    const vec2f& uvscale, float cap) {
  make_uvsphere(quads, positions, normals, texcoords, steps, scale, uvscale);
  if (cap) {
    cap        = min(cap, scale / 2);
    auto zflip = (scale - cap);
    for (auto i = 0; i < positions.size(); i++) {
      if (positions[i].z > zflip) {
        positions[i].z = 2 * zflip - positions[i].z;
        normals[i].x   = -normals[i].x;
        normals[i].y   = -normals[i].y;
      } else if (positions[i].z < -zflip) {
        positions[i].z = 2 * (-zflip) - positions[i].z;
        normals[i].x   = -normals[i].x;
        normals[i].y   = -normals[i].y;
      }
    }
  }
}

// Generate a disk
void make_disk(std::vector<vec4i>& quads, std::vector<vec3f>& positions,
    std::vector<vec3f>& normals, std::vector<vec2f>& texcoords, int steps,
    float scale, float uvscale) {
  make_rect(quads, positions, normals, texcoords, {steps, steps}, {1, 1},
      {uvscale, uvscale});
  for (auto i = 0; i < positions.size(); i++) {
    // Analytical Methods for Squaring the Disc, by C. Fong
    // https://arxiv.org/abs/1509.06344
    auto xy = vec2f{positions[i].x, positions[i].y};
    auto uv = vec2f{
        xy.x * sqrt(1 - xy.y * xy.y / 2), xy.y * sqrt(1 - xy.x * xy.x / 2)};
    positions[i] = vec3f{uv.x, uv.y, 0} * scale;
  }
}

void make_bulged_disk(std::vector<vec4i>& quads, std::vector<vec3f>& positions,
    std::vector<vec3f>& normals, std::vector<vec2f>& texcoords, int steps,
    float scale, float uvscale, float height) {
  make_disk(quads, positions, normals, texcoords, steps, scale, uvscale);
  if (height) {
    height      = min(height, scale);
    auto radius = (1 + height * height) / (2 * height);
    auto center = vec3f{0, 0, -radius + height};
    for (auto i = 0; i < positions.size(); i++) {
      auto pn      = normalize(positions[i] - center);
      positions[i] = center + pn * radius;
      normals[i]   = pn;
    }
  }
}

// Generate a uvdisk
void make_uvdisk(std::vector<vec4i>& quads, std::vector<vec3f>& positions,
    std::vector<vec3f>& normals, std::vector<vec2f>& texcoords,
    const vec2i& steps, float scale, const vec2f& uvscale) {
  make_rect(quads, positions, normals, texcoords, steps, {1, 1}, {1, 1});
  for (auto i = 0; i < positions.size(); i++) {
    auto uv      = texcoords[i];
    auto phi     = 2 * pif * uv.x;
    positions[i] = vec3f{cos(phi) * uv.y, sin(phi) * uv.y, 0} * scale;
    normals[i]   = {0, 0, 1};
    texcoords[i] = uv * uvscale;
  }
}

// Generate a uvcylinder
void make_uvcylinder(std::vector<vec4i>& quads, std::vector<vec3f>& positions,
    std::vector<vec3f>& normals, std::vector<vec2f>& texcoords,
    const vec3i& steps, const vec2f& scale, const vec3f& uvscale) {
  auto qquads     = std::vector<vec4i>{};
  auto qpositions = std::vector<vec3f>{};
  auto qnormals   = std::vector<vec3f>{};
  auto qtexcoords = std::vector<vec2f>{};
  // side
  make_rect(qquads, qpositions, qnormals, qtexcoords, {steps.x, steps.y},
      {1, 1}, {1, 1});
  for (auto i = 0; i < qpositions.size(); i++) {
    auto uv       = qtexcoords[i];
    auto phi      = 2 * pif * uv.x;
    qpositions[i] = {
        cos(phi) * scale.x, sin(phi) * scale.x, (2 * uv.y - 1) * scale.y};
    qnormals[i]   = {cos(phi), sin(phi), 0};
    qtexcoords[i] = uv * vec2f{uvscale.x, uvscale.y};
  }
  merge_quads(quads, positions, normals, texcoords, qquads, qpositions,
      qnormals, qtexcoords);
  // top
  make_rect(qquads, qpositions, qnormals, qtexcoords, {steps.x, steps.z},
      {1, 1}, {1, 1});
  for (auto i = 0; i < qpositions.size(); i++) {
    auto uv         = qtexcoords[i];
    auto phi        = 2 * pif * uv.x;
    qpositions[i]   = {cos(phi) * uv.y * scale.x, sin(phi) * uv.y * scale.x, 0};
    qnormals[i]     = {0, 0, 1};
    qtexcoords[i]   = uv * vec2f{uvscale.x, uvscale.z};
    qpositions[i].z = scale.y;
  }
  merge_quads(quads, positions, normals, texcoords, qquads, qpositions,
      qnormals, qtexcoords);
  // bottom
  make_rect(qquads, qpositions, qnormals, qtexcoords, {steps.x, steps.z},
      {1, 1}, {1, 1});
  for (auto i = 0; i < qpositions.size(); i++) {
    auto uv         = qtexcoords[i];
    auto phi        = 2 * pif * uv.x;
    qpositions[i]   = {cos(phi) * uv.y * scale.x, sin(phi) * uv.y * scale.x, 0};
    qnormals[i]     = {0, 0, 1};
    qtexcoords[i]   = uv * vec2f{uvscale.x, uvscale.z};
    qpositions[i].z = -scale.y;
    qnormals[i]     = -qnormals[i];
  }
  for (auto i = 0; i < qquads.size(); i++) swap(qquads[i].x, qquads[i].z);
  merge_quads(quads, positions, normals, texcoords, qquads, qpositions,
      qnormals, qtexcoords);
}

// Generate a uvcylinder
void make_rounded_uvcylinder(std::vector<vec4i>& quads,
    std::vector<vec3f>& positions, std::vector<vec3f>& normals,
    std::vector<vec2f>& texcoords, const vec3i& steps, const vec2f& scale,
    const vec3f& uvscale, float radius) {
  make_uvcylinder(quads, positions, normals, texcoords, steps, scale, uvscale);
  if (radius) {
    radius = min(radius, min(scale));
    auto c = scale - radius;
    for (auto i = 0; i < positions.size(); i++) {
      auto phi = atan2(positions[i].y, positions[i].x);
      auto r   = length(vec2f{positions[i].x, positions[i].y});
      auto z   = positions[i].z;
      auto pc  = vec2f{r, abs(z)};
      auto ps  = (z < 0) ? -1.0f : 1.0f;
      if (pc.x >= c.x && pc.y >= c.y) {
        auto pn      = normalize(pc - c);
        positions[i] = {cos(phi) * (c.x + radius * pn.x),
            sin(phi) * (c.x + radius * pn.x), ps * (c.y + radius * pn.y)};
        normals[i]   = {cos(phi) * pn.x, sin(phi) * pn.x, ps * pn.y};
      } else {
        continue;
      }
    }
  }
}

// Make a quad.
void make_yrect(std::vector<vec4i>& quads, std::vector<vec3f>& positions,
    std::vector<vec3f>& normals, std::vector<vec2f>& texcoords,
    const vec2i& steps, const vec2f& scale, const vec2f& uvscale) {
  make_rect(quads, positions, normals, texcoords, steps, scale, uvscale);
  for (auto& p : positions) {
    std::swap(p.y, p.z);
    p.z = -p.z;
  }
  for (auto& n : normals) std::swap(n.y, n.z);
}

void make_bulged_yrect(std::vector<vec4i>& quads, std::vector<vec3f>& positions,
    std::vector<vec3f>& normals, std::vector<vec2f>& texcoords,
    const vec2i& steps, const vec2f& scale, const vec2f& uvscale,
    float height) {
  make_bulged_rect(
      quads, positions, normals, texcoords, steps, scale, uvscale, height);
  for (auto& p : positions) {
    std::swap(p.y, p.z);
    p.z = -p.z;
  }
  for (auto& n : normals) std::swap(n.y, n.z);
}

// Generate lines set along a quad.
void make_lines(std::vector<vec2i>& lines, std::vector<vec3f>& positions,
    std::vector<vec3f>& normals, std::vector<vec2f>& texcoords,
    std::vector<float>& radius, const vec2i& steps, const vec2f& size,
    const vec2f& uvscale, const vec2f& rad) {
  auto nverts = (steps.x + 1) * steps.y;
  auto nlines = steps.x * steps.y;
  auto vid    = [steps](int i, int j) { return j * (steps.x + 1) + i; };
  auto fid    = [steps](int i, int j) { return j * steps.x + i; };

  positions.resize(nverts);
  normals.resize(nverts);
  texcoords.resize(nverts);
  radius.resize(nverts);
  if (steps.y > 1) {
    for (auto j = 0; j < steps.y; j++) {
      for (auto i = 0; i <= steps.x; i++) {
        auto uv = vec2f{
            i / (float)steps.x, j / (float)(steps.y > 1 ? steps.y - 1 : 1)};
        positions[vid(i, j)] = {
            (uv.x - 0.5f) * size.x, (uv.y - 0.5f) * size.y, 0};
        normals[vid(i, j)]   = {1, 0, 0};
        texcoords[vid(i, j)] = uv * uvscale;
      }
    }
  } else {
    for (auto i = 0; i <= steps.x; i++) {
      auto uv              = vec2f{i / (float)steps.x, 0};
      positions[vid(i, 0)] = {(uv.x - 0.5f) * size.x, 0, 0};
      normals[vid(i, 0)]   = {1, 0, 0};
      texcoords[vid(i, 0)] = uv * uvscale;
    }
  }

  lines.resize(nlines);
  for (int j = 0; j < steps.y; j++) {
    for (int i = 0; i < steps.x; i++) {
      lines[fid(i, j)] = {vid(i, j), vid(i + 1, j)};
    }
  }
}

// Generate a point at the origin.
void make_point(std::vector<int>& points, std::vector<vec3f>& positions,
    std::vector<vec3f>& normals, std::vector<vec2f>& texcoords,
    std::vector<float>& radius, float point_radius) {
  points    = {0};
  positions = {{0, 0, 0}};
  normals   = {{0, 0, 1}};
  texcoords = {{0, 0}};
  radius    = {point_radius};
}

// Generate a point set with points placed at the origin with texcoords
// varying along u.
void make_points(std::vector<int>& points, std::vector<vec3f>& positions,
    std::vector<vec3f>& normals, std::vector<vec2f>& texcoords,
    std::vector<float>& radius, int num, float uvscale, float point_radius) {
  points.resize(num);
  for (auto i = 0; i < num; i++) points[i] = i;
  positions.assign(num, {0, 0, 0});
  normals.assign(num, {0, 0, 1});
  texcoords.assign(num, {0, 0});
  radius.assign(num, point_radius);
  for (auto i = 0; i < texcoords.size(); i++)
    texcoords[i] = {(float)i / (float)num, 0};
}

// Generate a point set.
void make_random_points(std::vector<int>& points, std::vector<vec3f>& positions,
    std::vector<vec3f>& normals, std::vector<vec2f>& texcoords,
    std::vector<float>& radius, int num, const vec3f& size, float uvscale,
    float point_radius, uint64_t seed) {
  make_points(points, positions, normals, texcoords, radius, num, uvscale,
      point_radius);
  auto rng = make_rng(seed);
  for (auto i = 0; i < positions.size(); i++) {
    positions[i] = (rand3f(rng) - vec3f{0.5f, 0.5f, 0.5f}) * size;
  }
}

// Make a bezier circle. Returns bezier, pos.
void make_bezier_circle(
    std::vector<vec4i>& beziers, std::vector<vec3f>& positions, float size) {
  // constant from http://spencermortensen.com/articles/bezier-circle/
  const auto  c          = 0.551915024494f;
  static auto circle_pos = std::vector<vec3f>{{1, 0, 0}, {1, c, 0}, {c, 1, 0},
      {0, 1, 0}, {-c, 1, 0}, {-1, c, 0}, {-1, 0, 0}, {-1, -c, 0}, {-c, -1, 0},
      {0, -1, 0}, {c, -1, 0}, {1, -c, 0}};
  static auto circle_beziers = std::vector<vec4i>{
      {0, 1, 2, 3}, {3, 4, 5, 6}, {6, 7, 8, 9}, {9, 10, 11, 0}};
  positions = circle_pos;
  beziers   = circle_beziers;
  for (auto& p : positions) p *= size;
}

// Make fvquad
void make_fvrect(std::vector<vec4i>& quadspos, std::vector<vec4i>& quadsnorm,
    std::vector<vec4i>& quadstexcoord, std::vector<vec3f>& positions,
    std::vector<vec3f>& normals, std::vector<vec2f>& texcoords,
    const vec2i& steps, const vec2f& size, const vec2f& uvscale) {
  make_rect(quadspos, positions, normals, texcoords, steps, size, uvscale);
  quadsnorm     = quadspos;
  quadstexcoord = quadspos;
}
void make_fvbox(std::vector<vec4i>& quadspos, std::vector<vec4i>& quadsnorm,
    std::vector<vec4i>& quadstexcoord, std::vector<vec3f>& positions,
    std::vector<vec3f>& normals, std::vector<vec2f>& texcoords,
    const vec3i& steps, const vec3f& size, const vec3f& uvscale) {
  make_box(quadspos, positions, normals, texcoords, steps, size, uvscale);
  quadsnorm                     = quadspos;
  quadstexcoord                 = quadspos;
  std::tie(quadspos, positions) = weld_quads(
      quadspos, positions, 0.1f * min(size) / max(steps));
}
void make_fvsphere(std::vector<vec4i>& quadspos, std::vector<vec4i>& quadsnorm,
    std::vector<vec4i>& quadstexcoord, std::vector<vec3f>& positions,
    std::vector<vec3f>& normals, std::vector<vec2f>& texcoords, int steps,
    float size, float uvscale) {
  make_fvbox(quadspos, quadsnorm, quadstexcoord, positions, normals, texcoords,
      {steps, steps, steps}, {size, size, size}, {uvscale, uvscale, uvscale});
  quadsnorm = quadspos;
  normals   = positions;
  for (auto& n : normals) n = normalize(n);
}

// Predefined meshes
void make_monkey(
    std::vector<vec4i>& quads, std::vector<vec3f>& positions, float scale) {
  static const auto suzanne_positions = std::vector<vec3f>{
      {0.4375, 0.1640625, 0.765625}, {-0.4375, 0.1640625, 0.765625},
      {0.5, 0.09375, 0.6875}, {-0.5, 0.09375, 0.6875},
      {0.546875, 0.0546875, 0.578125}, {-0.546875, 0.0546875, 0.578125},
      {0.3515625, -0.0234375, 0.6171875}, {-0.3515625, -0.0234375, 0.6171875},
      {0.3515625, 0.03125, 0.71875}, {-0.3515625, 0.03125, 0.71875},
      {0.3515625, 0.1328125, 0.78125}, {-0.3515625, 0.1328125, 0.78125},
      {0.2734375, 0.1640625, 0.796875}, {-0.2734375, 0.1640625, 0.796875},
      {0.203125, 0.09375, 0.7421875}, {-0.203125, 0.09375, 0.7421875},
      {0.15625, 0.0546875, 0.6484375}, {-0.15625, 0.0546875, 0.6484375},
      {0.078125, 0.2421875, 0.65625}, {-0.078125, 0.2421875, 0.65625},
      {0.140625, 0.2421875, 0.7421875}, {-0.140625, 0.2421875, 0.7421875},
      {0.2421875, 0.2421875, 0.796875}, {-0.2421875, 0.2421875, 0.796875},
      {0.2734375, 0.328125, 0.796875}, {-0.2734375, 0.328125, 0.796875},
      {0.203125, 0.390625, 0.7421875}, {-0.203125, 0.390625, 0.7421875},
      {0.15625, 0.4375, 0.6484375}, {-0.15625, 0.4375, 0.6484375},
      {0.3515625, 0.515625, 0.6171875}, {-0.3515625, 0.515625, 0.6171875},
      {0.3515625, 0.453125, 0.71875}, {-0.3515625, 0.453125, 0.71875},
      {0.3515625, 0.359375, 0.78125}, {-0.3515625, 0.359375, 0.78125},
      {0.4375, 0.328125, 0.765625}, {-0.4375, 0.328125, 0.765625},
      {0.5, 0.390625, 0.6875}, {-0.5, 0.390625, 0.6875},
      {0.546875, 0.4375, 0.578125}, {-0.546875, 0.4375, 0.578125},
      {0.625, 0.2421875, 0.5625}, {-0.625, 0.2421875, 0.5625},
      {0.5625, 0.2421875, 0.671875}, {-0.5625, 0.2421875, 0.671875},
      {0.46875, 0.2421875, 0.7578125}, {-0.46875, 0.2421875, 0.7578125},
      {0.4765625, 0.2421875, 0.7734375}, {-0.4765625, 0.2421875, 0.7734375},
      {0.4453125, 0.3359375, 0.78125}, {-0.4453125, 0.3359375, 0.78125},
      {0.3515625, 0.375, 0.8046875}, {-0.3515625, 0.375, 0.8046875},
      {0.265625, 0.3359375, 0.8203125}, {-0.265625, 0.3359375, 0.8203125},
      {0.2265625, 0.2421875, 0.8203125}, {-0.2265625, 0.2421875, 0.8203125},
      {0.265625, 0.15625, 0.8203125}, {-0.265625, 0.15625, 0.8203125},
      {0.3515625, 0.2421875, 0.828125}, {-0.3515625, 0.2421875, 0.828125},
      {0.3515625, 0.1171875, 0.8046875}, {-0.3515625, 0.1171875, 0.8046875},
      {0.4453125, 0.15625, 0.78125}, {-0.4453125, 0.15625, 0.78125},
      {0.0, 0.4296875, 0.7421875}, {0.0, 0.3515625, 0.8203125},
      {0.0, -0.6796875, 0.734375}, {0.0, -0.3203125, 0.78125},
      {0.0, -0.1875, 0.796875}, {0.0, -0.7734375, 0.71875},
      {0.0, 0.40625, 0.6015625}, {0.0, 0.5703125, 0.5703125},
      {0.0, 0.8984375, -0.546875}, {0.0, 0.5625, -0.8515625},
      {0.0, 0.0703125, -0.828125}, {0.0, -0.3828125, -0.3515625},
      {0.203125, -0.1875, 0.5625}, {-0.203125, -0.1875, 0.5625},
      {0.3125, -0.4375, 0.5703125}, {-0.3125, -0.4375, 0.5703125},
      {0.3515625, -0.6953125, 0.5703125}, {-0.3515625, -0.6953125, 0.5703125},
      {0.3671875, -0.890625, 0.53125}, {-0.3671875, -0.890625, 0.53125},
      {0.328125, -0.9453125, 0.5234375}, {-0.328125, -0.9453125, 0.5234375},
      {0.1796875, -0.96875, 0.5546875}, {-0.1796875, -0.96875, 0.5546875},
      {0.0, -0.984375, 0.578125}, {0.4375, -0.140625, 0.53125},
      {-0.4375, -0.140625, 0.53125}, {0.6328125, -0.0390625, 0.5390625},
      {-0.6328125, -0.0390625, 0.5390625}, {0.828125, 0.1484375, 0.4453125},
      {-0.828125, 0.1484375, 0.4453125}, {0.859375, 0.4296875, 0.59375},
      {-0.859375, 0.4296875, 0.59375}, {0.7109375, 0.484375, 0.625},
      {-0.7109375, 0.484375, 0.625}, {0.4921875, 0.6015625, 0.6875},
      {-0.4921875, 0.6015625, 0.6875}, {0.3203125, 0.7578125, 0.734375},
      {-0.3203125, 0.7578125, 0.734375}, {0.15625, 0.71875, 0.7578125},
      {-0.15625, 0.71875, 0.7578125}, {0.0625, 0.4921875, 0.75},
      {-0.0625, 0.4921875, 0.75}, {0.1640625, 0.4140625, 0.7734375},
      {-0.1640625, 0.4140625, 0.7734375}, {0.125, 0.3046875, 0.765625},
      {-0.125, 0.3046875, 0.765625}, {0.203125, 0.09375, 0.7421875},
      {-0.203125, 0.09375, 0.7421875}, {0.375, 0.015625, 0.703125},
      {-0.375, 0.015625, 0.703125}, {0.4921875, 0.0625, 0.671875},
      {-0.4921875, 0.0625, 0.671875}, {0.625, 0.1875, 0.6484375},
      {-0.625, 0.1875, 0.6484375}, {0.640625, 0.296875, 0.6484375},
      {-0.640625, 0.296875, 0.6484375}, {0.6015625, 0.375, 0.6640625},
      {-0.6015625, 0.375, 0.6640625}, {0.4296875, 0.4375, 0.71875},
      {-0.4296875, 0.4375, 0.71875}, {0.25, 0.46875, 0.7578125},
      {-0.25, 0.46875, 0.7578125}, {0.0, -0.765625, 0.734375},
      {0.109375, -0.71875, 0.734375}, {-0.109375, -0.71875, 0.734375},
      {0.1171875, -0.8359375, 0.7109375}, {-0.1171875, -0.8359375, 0.7109375},
      {0.0625, -0.8828125, 0.6953125}, {-0.0625, -0.8828125, 0.6953125},
      {0.0, -0.890625, 0.6875}, {0.0, -0.1953125, 0.75},
      {0.0, -0.140625, 0.7421875}, {0.1015625, -0.1484375, 0.7421875},
      {-0.1015625, -0.1484375, 0.7421875}, {0.125, -0.2265625, 0.75},
      {-0.125, -0.2265625, 0.75}, {0.0859375, -0.2890625, 0.7421875},
      {-0.0859375, -0.2890625, 0.7421875}, {0.3984375, -0.046875, 0.671875},
      {-0.3984375, -0.046875, 0.671875}, {0.6171875, 0.0546875, 0.625},
      {-0.6171875, 0.0546875, 0.625}, {0.7265625, 0.203125, 0.6015625},
      {-0.7265625, 0.203125, 0.6015625}, {0.7421875, 0.375, 0.65625},
      {-0.7421875, 0.375, 0.65625}, {0.6875, 0.4140625, 0.7265625},
      {-0.6875, 0.4140625, 0.7265625}, {0.4375, 0.546875, 0.796875},
      {-0.4375, 0.546875, 0.796875}, {0.3125, 0.640625, 0.8359375},
      {-0.3125, 0.640625, 0.8359375}, {0.203125, 0.6171875, 0.8515625},
      {-0.203125, 0.6171875, 0.8515625}, {0.1015625, 0.4296875, 0.84375},
      {-0.1015625, 0.4296875, 0.84375}, {0.125, -0.1015625, 0.8125},
      {-0.125, -0.1015625, 0.8125}, {0.2109375, -0.4453125, 0.7109375},
      {-0.2109375, -0.4453125, 0.7109375}, {0.25, -0.703125, 0.6875},
      {-0.25, -0.703125, 0.6875}, {0.265625, -0.8203125, 0.6640625},
      {-0.265625, -0.8203125, 0.6640625}, {0.234375, -0.9140625, 0.6328125},
      {-0.234375, -0.9140625, 0.6328125}, {0.1640625, -0.9296875, 0.6328125},
      {-0.1640625, -0.9296875, 0.6328125}, {0.0, -0.9453125, 0.640625},
      {0.0, 0.046875, 0.7265625}, {0.0, 0.2109375, 0.765625},
      {0.328125, 0.4765625, 0.7421875}, {-0.328125, 0.4765625, 0.7421875},
      {0.1640625, 0.140625, 0.75}, {-0.1640625, 0.140625, 0.75},
      {0.1328125, 0.2109375, 0.7578125}, {-0.1328125, 0.2109375, 0.7578125},
      {0.1171875, -0.6875, 0.734375}, {-0.1171875, -0.6875, 0.734375},
      {0.078125, -0.4453125, 0.75}, {-0.078125, -0.4453125, 0.75},
      {0.0, -0.4453125, 0.75}, {0.0, -0.328125, 0.7421875},
      {0.09375, -0.2734375, 0.78125}, {-0.09375, -0.2734375, 0.78125},
      {0.1328125, -0.2265625, 0.796875}, {-0.1328125, -0.2265625, 0.796875},
      {0.109375, -0.1328125, 0.78125}, {-0.109375, -0.1328125, 0.78125},
      {0.0390625, -0.125, 0.78125}, {-0.0390625, -0.125, 0.78125},
      {0.0, -0.203125, 0.828125}, {0.046875, -0.1484375, 0.8125},
      {-0.046875, -0.1484375, 0.8125}, {0.09375, -0.15625, 0.8125},
      {-0.09375, -0.15625, 0.8125}, {0.109375, -0.2265625, 0.828125},
      {-0.109375, -0.2265625, 0.828125}, {0.078125, -0.25, 0.8046875},
      {-0.078125, -0.25, 0.8046875}, {0.0, -0.2890625, 0.8046875},
      {0.2578125, -0.3125, 0.5546875}, {-0.2578125, -0.3125, 0.5546875},
      {0.1640625, -0.2421875, 0.7109375}, {-0.1640625, -0.2421875, 0.7109375},
      {0.1796875, -0.3125, 0.7109375}, {-0.1796875, -0.3125, 0.7109375},
      {0.234375, -0.25, 0.5546875}, {-0.234375, -0.25, 0.5546875},
      {0.0, -0.875, 0.6875}, {0.046875, -0.8671875, 0.6875},
      {-0.046875, -0.8671875, 0.6875}, {0.09375, -0.8203125, 0.7109375},
      {-0.09375, -0.8203125, 0.7109375}, {0.09375, -0.7421875, 0.7265625},
      {-0.09375, -0.7421875, 0.7265625}, {0.0, -0.78125, 0.65625},
      {0.09375, -0.75, 0.6640625}, {-0.09375, -0.75, 0.6640625},
      {0.09375, -0.8125, 0.640625}, {-0.09375, -0.8125, 0.640625},
      {0.046875, -0.8515625, 0.6328125}, {-0.046875, -0.8515625, 0.6328125},
      {0.0, -0.859375, 0.6328125}, {0.171875, 0.21875, 0.78125},
      {-0.171875, 0.21875, 0.78125}, {0.1875, 0.15625, 0.7734375},
      {-0.1875, 0.15625, 0.7734375}, {0.3359375, 0.4296875, 0.7578125},
      {-0.3359375, 0.4296875, 0.7578125}, {0.2734375, 0.421875, 0.7734375},
      {-0.2734375, 0.421875, 0.7734375}, {0.421875, 0.3984375, 0.7734375},
      {-0.421875, 0.3984375, 0.7734375}, {0.5625, 0.3515625, 0.6953125},
      {-0.5625, 0.3515625, 0.6953125}, {0.5859375, 0.2890625, 0.6875},
      {-0.5859375, 0.2890625, 0.6875}, {0.578125, 0.1953125, 0.6796875},
      {-0.578125, 0.1953125, 0.6796875}, {0.4765625, 0.1015625, 0.71875},
      {-0.4765625, 0.1015625, 0.71875}, {0.375, 0.0625, 0.7421875},
      {-0.375, 0.0625, 0.7421875}, {0.2265625, 0.109375, 0.78125},
      {-0.2265625, 0.109375, 0.78125}, {0.1796875, 0.296875, 0.78125},
      {-0.1796875, 0.296875, 0.78125}, {0.2109375, 0.375, 0.78125},
      {-0.2109375, 0.375, 0.78125}, {0.234375, 0.359375, 0.7578125},
      {-0.234375, 0.359375, 0.7578125}, {0.1953125, 0.296875, 0.7578125},
      {-0.1953125, 0.296875, 0.7578125}, {0.2421875, 0.125, 0.7578125},
      {-0.2421875, 0.125, 0.7578125}, {0.375, 0.0859375, 0.7265625},
      {-0.375, 0.0859375, 0.7265625}, {0.4609375, 0.1171875, 0.703125},
      {-0.4609375, 0.1171875, 0.703125}, {0.546875, 0.2109375, 0.671875},
      {-0.546875, 0.2109375, 0.671875}, {0.5546875, 0.28125, 0.671875},
      {-0.5546875, 0.28125, 0.671875}, {0.53125, 0.3359375, 0.6796875},
      {-0.53125, 0.3359375, 0.6796875}, {0.4140625, 0.390625, 0.75},
      {-0.4140625, 0.390625, 0.75}, {0.28125, 0.3984375, 0.765625},
      {-0.28125, 0.3984375, 0.765625}, {0.3359375, 0.40625, 0.75},
      {-0.3359375, 0.40625, 0.75}, {0.203125, 0.171875, 0.75},
      {-0.203125, 0.171875, 0.75}, {0.1953125, 0.2265625, 0.75},
      {-0.1953125, 0.2265625, 0.75}, {0.109375, 0.4609375, 0.609375},
      {-0.109375, 0.4609375, 0.609375}, {0.1953125, 0.6640625, 0.6171875},
      {-0.1953125, 0.6640625, 0.6171875}, {0.3359375, 0.6875, 0.59375},
      {-0.3359375, 0.6875, 0.59375}, {0.484375, 0.5546875, 0.5546875},
      {-0.484375, 0.5546875, 0.5546875}, {0.6796875, 0.453125, 0.4921875},
      {-0.6796875, 0.453125, 0.4921875}, {0.796875, 0.40625, 0.4609375},
      {-0.796875, 0.40625, 0.4609375}, {0.7734375, 0.1640625, 0.375},
      {-0.7734375, 0.1640625, 0.375}, {0.6015625, 0.0, 0.4140625},
      {-0.6015625, 0.0, 0.4140625}, {0.4375, -0.09375, 0.46875},
      {-0.4375, -0.09375, 0.46875}, {0.0, 0.8984375, 0.2890625},
      {0.0, 0.984375, -0.078125}, {0.0, -0.1953125, -0.671875},
      {0.0, -0.4609375, 0.1875}, {0.0, -0.9765625, 0.4609375},
      {0.0, -0.8046875, 0.34375}, {0.0, -0.5703125, 0.3203125},
      {0.0, -0.484375, 0.28125}, {0.8515625, 0.234375, 0.0546875},
      {-0.8515625, 0.234375, 0.0546875}, {0.859375, 0.3203125, -0.046875},
      {-0.859375, 0.3203125, -0.046875}, {0.7734375, 0.265625, -0.4375},
      {-0.7734375, 0.265625, -0.4375}, {0.4609375, 0.4375, -0.703125},
      {-0.4609375, 0.4375, -0.703125}, {0.734375, -0.046875, 0.0703125},
      {-0.734375, -0.046875, 0.0703125}, {0.59375, -0.125, -0.1640625},
      {-0.59375, -0.125, -0.1640625}, {0.640625, -0.0078125, -0.4296875},
      {-0.640625, -0.0078125, -0.4296875}, {0.3359375, 0.0546875, -0.6640625},
      {-0.3359375, 0.0546875, -0.6640625}, {0.234375, -0.3515625, 0.40625},
      {-0.234375, -0.3515625, 0.40625}, {0.1796875, -0.4140625, 0.2578125},
      {-0.1796875, -0.4140625, 0.2578125}, {0.2890625, -0.7109375, 0.3828125},
      {-0.2890625, -0.7109375, 0.3828125}, {0.25, -0.5, 0.390625},
      {-0.25, -0.5, 0.390625}, {0.328125, -0.9140625, 0.3984375},
      {-0.328125, -0.9140625, 0.3984375}, {0.140625, -0.7578125, 0.3671875},
      {-0.140625, -0.7578125, 0.3671875}, {0.125, -0.5390625, 0.359375},
      {-0.125, -0.5390625, 0.359375}, {0.1640625, -0.9453125, 0.4375},
      {-0.1640625, -0.9453125, 0.4375}, {0.21875, -0.28125, 0.4296875},
      {-0.21875, -0.28125, 0.4296875}, {0.2109375, -0.2265625, 0.46875},
      {-0.2109375, -0.2265625, 0.46875}, {0.203125, -0.171875, 0.5},
      {-0.203125, -0.171875, 0.5}, {0.2109375, -0.390625, 0.1640625},
      {-0.2109375, -0.390625, 0.1640625}, {0.296875, -0.3125, -0.265625},
      {-0.296875, -0.3125, -0.265625}, {0.34375, -0.1484375, -0.5390625},
      {-0.34375, -0.1484375, -0.5390625}, {0.453125, 0.8671875, -0.3828125},
      {-0.453125, 0.8671875, -0.3828125}, {0.453125, 0.9296875, -0.0703125},
      {-0.453125, 0.9296875, -0.0703125}, {0.453125, 0.8515625, 0.234375},
      {-0.453125, 0.8515625, 0.234375}, {0.4609375, 0.5234375, 0.4296875},
      {-0.4609375, 0.5234375, 0.4296875}, {0.7265625, 0.40625, 0.3359375},
      {-0.7265625, 0.40625, 0.3359375}, {0.6328125, 0.453125, 0.28125},
      {-0.6328125, 0.453125, 0.28125}, {0.640625, 0.703125, 0.0546875},
      {-0.640625, 0.703125, 0.0546875}, {0.796875, 0.5625, 0.125},
      {-0.796875, 0.5625, 0.125}, {0.796875, 0.6171875, -0.1171875},
      {-0.796875, 0.6171875, -0.1171875}, {0.640625, 0.75, -0.1953125},
      {-0.640625, 0.75, -0.1953125}, {0.640625, 0.6796875, -0.4453125},
      {-0.640625, 0.6796875, -0.4453125}, {0.796875, 0.5390625, -0.359375},
      {-0.796875, 0.5390625, -0.359375}, {0.6171875, 0.328125, -0.5859375},
      {-0.6171875, 0.328125, -0.5859375}, {0.484375, 0.0234375, -0.546875},
      {-0.484375, 0.0234375, -0.546875}, {0.8203125, 0.328125, -0.203125},
      {-0.8203125, 0.328125, -0.203125}, {0.40625, -0.171875, 0.1484375},
      {-0.40625, -0.171875, 0.1484375}, {0.4296875, -0.1953125, -0.2109375},
      {-0.4296875, -0.1953125, -0.2109375}, {0.890625, 0.40625, -0.234375},
      {-0.890625, 0.40625, -0.234375}, {0.7734375, -0.140625, -0.125},
      {-0.7734375, -0.140625, -0.125}, {1.0390625, -0.1015625, -0.328125},
      {-1.0390625, -0.1015625, -0.328125}, {1.28125, 0.0546875, -0.4296875},
      {-1.28125, 0.0546875, -0.4296875}, {1.3515625, 0.3203125, -0.421875},
      {-1.3515625, 0.3203125, -0.421875}, {1.234375, 0.5078125, -0.421875},
      {-1.234375, 0.5078125, -0.421875}, {1.0234375, 0.4765625, -0.3125},
      {-1.0234375, 0.4765625, -0.3125}, {1.015625, 0.4140625, -0.2890625},
      {-1.015625, 0.4140625, -0.2890625}, {1.1875, 0.4375, -0.390625},
      {-1.1875, 0.4375, -0.390625}, {1.265625, 0.2890625, -0.40625},
      {-1.265625, 0.2890625, -0.40625}, {1.2109375, 0.078125, -0.40625},
      {-1.2109375, 0.078125, -0.40625}, {1.03125, -0.0390625, -0.3046875},
      {-1.03125, -0.0390625, -0.3046875}, {0.828125, -0.0703125, -0.1328125},
      {-0.828125, -0.0703125, -0.1328125}, {0.921875, 0.359375, -0.21875},
      {-0.921875, 0.359375, -0.21875}, {0.9453125, 0.3046875, -0.2890625},
      {-0.9453125, 0.3046875, -0.2890625}, {0.8828125, -0.0234375, -0.2109375},
      {-0.8828125, -0.0234375, -0.2109375}, {1.0390625, 0.0, -0.3671875},
      {-1.0390625, 0.0, -0.3671875}, {1.1875, 0.09375, -0.4453125},
      {-1.1875, 0.09375, -0.4453125}, {1.234375, 0.25, -0.4453125},
      {-1.234375, 0.25, -0.4453125}, {1.171875, 0.359375, -0.4375},
      {-1.171875, 0.359375, -0.4375}, {1.0234375, 0.34375, -0.359375},
      {-1.0234375, 0.34375, -0.359375}, {0.84375, 0.2890625, -0.2109375},
      {-0.84375, 0.2890625, -0.2109375}, {0.8359375, 0.171875, -0.2734375},
      {-0.8359375, 0.171875, -0.2734375}, {0.7578125, 0.09375, -0.2734375},
      {-0.7578125, 0.09375, -0.2734375}, {0.8203125, 0.0859375, -0.2734375},
      {-0.8203125, 0.0859375, -0.2734375}, {0.84375, 0.015625, -0.2734375},
      {-0.84375, 0.015625, -0.2734375}, {0.8125, -0.015625, -0.2734375},
      {-0.8125, -0.015625, -0.2734375}, {0.7265625, 0.0, -0.0703125},
      {-0.7265625, 0.0, -0.0703125}, {0.71875, -0.0234375, -0.171875},
      {-0.71875, -0.0234375, -0.171875}, {0.71875, 0.0390625, -0.1875},
      {-0.71875, 0.0390625, -0.1875}, {0.796875, 0.203125, -0.2109375},
      {-0.796875, 0.203125, -0.2109375}, {0.890625, 0.2421875, -0.265625},
      {-0.890625, 0.2421875, -0.265625}, {0.890625, 0.234375, -0.3203125},
      {-0.890625, 0.234375, -0.3203125}, {0.8125, -0.015625, -0.3203125},
      {-0.8125, -0.015625, -0.3203125}, {0.8515625, 0.015625, -0.3203125},
      {-0.8515625, 0.015625, -0.3203125}, {0.828125, 0.078125, -0.3203125},
      {-0.828125, 0.078125, -0.3203125}, {0.765625, 0.09375, -0.3203125},
      {-0.765625, 0.09375, -0.3203125}, {0.84375, 0.171875, -0.3203125},
      {-0.84375, 0.171875, -0.3203125}, {1.0390625, 0.328125, -0.4140625},
      {-1.0390625, 0.328125, -0.4140625}, {1.1875, 0.34375, -0.484375},
      {-1.1875, 0.34375, -0.484375}, {1.2578125, 0.2421875, -0.4921875},
      {-1.2578125, 0.2421875, -0.4921875}, {1.2109375, 0.0859375, -0.484375},
      {-1.2109375, 0.0859375, -0.484375}, {1.046875, 0.0, -0.421875},
      {-1.046875, 0.0, -0.421875}, {0.8828125, -0.015625, -0.265625},
      {-0.8828125, -0.015625, -0.265625}, {0.953125, 0.2890625, -0.34375},
      {-0.953125, 0.2890625, -0.34375}, {0.890625, 0.109375, -0.328125},
      {-0.890625, 0.109375, -0.328125}, {0.9375, 0.0625, -0.3359375},
      {-0.9375, 0.0625, -0.3359375}, {1.0, 0.125, -0.3671875},
      {-1.0, 0.125, -0.3671875}, {0.9609375, 0.171875, -0.3515625},
      {-0.9609375, 0.171875, -0.3515625}, {1.015625, 0.234375, -0.375},
      {-1.015625, 0.234375, -0.375}, {1.0546875, 0.1875, -0.3828125},
      {-1.0546875, 0.1875, -0.3828125}, {1.109375, 0.2109375, -0.390625},
      {-1.109375, 0.2109375, -0.390625}, {1.0859375, 0.2734375, -0.390625},
      {-1.0859375, 0.2734375, -0.390625}, {1.0234375, 0.4375, -0.484375},
      {-1.0234375, 0.4375, -0.484375}, {1.25, 0.46875, -0.546875},
      {-1.25, 0.46875, -0.546875}, {1.3671875, 0.296875, -0.5},
      {-1.3671875, 0.296875, -0.5}, {1.3125, 0.0546875, -0.53125},
      {-1.3125, 0.0546875, -0.53125}, {1.0390625, -0.0859375, -0.4921875},
      {-1.0390625, -0.0859375, -0.4921875}, {0.7890625, -0.125, -0.328125},
      {-0.7890625, -0.125, -0.328125}, {0.859375, 0.3828125, -0.3828125},
      {-0.859375, 0.3828125, -0.3828125}};
  static const auto suzanne_quads = std::vector<vec4i>{{46, 0, 2, 44},
      {3, 1, 47, 45}, {44, 2, 4, 42}, {5, 3, 45, 43}, {2, 8, 6, 4},
      {7, 9, 3, 5}, {0, 10, 8, 2}, {9, 11, 1, 3}, {10, 12, 14, 8},
      {15, 13, 11, 9}, {8, 14, 16, 6}, {17, 15, 9, 7}, {14, 20, 18, 16},
      {19, 21, 15, 17}, {12, 22, 20, 14}, {21, 23, 13, 15}, {22, 24, 26, 20},
      {27, 25, 23, 21}, {20, 26, 28, 18}, {29, 27, 21, 19}, {26, 32, 30, 28},
      {31, 33, 27, 29}, {24, 34, 32, 26}, {33, 35, 25, 27}, {34, 36, 38, 32},
      {39, 37, 35, 33}, {32, 38, 40, 30}, {41, 39, 33, 31}, {38, 44, 42, 40},
      {43, 45, 39, 41}, {36, 46, 44, 38}, {45, 47, 37, 39}, {46, 36, 50, 48},
      {51, 37, 47, 49}, {36, 34, 52, 50}, {53, 35, 37, 51}, {34, 24, 54, 52},
      {55, 25, 35, 53}, {24, 22, 56, 54}, {57, 23, 25, 55}, {22, 12, 58, 56},
      {59, 13, 23, 57}, {12, 10, 62, 58}, {63, 11, 13, 59}, {10, 0, 64, 62},
      {65, 1, 11, 63}, {0, 46, 48, 64}, {49, 47, 1, 65}, {88, 173, 175, 90},
      {175, 174, 89, 90}, {86, 171, 173, 88}, {174, 172, 87, 89},
      {84, 169, 171, 86}, {172, 170, 85, 87}, {82, 167, 169, 84},
      {170, 168, 83, 85}, {80, 165, 167, 82}, {168, 166, 81, 83},
      {78, 91, 145, 163}, {146, 92, 79, 164}, {91, 93, 147, 145},
      {148, 94, 92, 146}, {93, 95, 149, 147}, {150, 96, 94, 148},
      {95, 97, 151, 149}, {152, 98, 96, 150}, {97, 99, 153, 151},
      {154, 100, 98, 152}, {99, 101, 155, 153}, {156, 102, 100, 154},
      {101, 103, 157, 155}, {158, 104, 102, 156}, {103, 105, 159, 157},
      {160, 106, 104, 158}, {105, 107, 161, 159}, {162, 108, 106, 160},
      {107, 66, 67, 161}, {67, 66, 108, 162}, {109, 127, 159, 161},
      {160, 128, 110, 162}, {127, 178, 157, 159}, {158, 179, 128, 160},
      {125, 155, 157, 178}, {158, 156, 126, 179}, {123, 153, 155, 125},
      {156, 154, 124, 126}, {121, 151, 153, 123}, {154, 152, 122, 124},
      {119, 149, 151, 121}, {152, 150, 120, 122}, {117, 147, 149, 119},
      {150, 148, 118, 120}, {115, 145, 147, 117}, {148, 146, 116, 118},
      {113, 163, 145, 115}, {146, 164, 114, 116}, {113, 180, 176, 163},
      {176, 181, 114, 164}, {109, 161, 67, 111}, {67, 162, 110, 112},
      {111, 67, 177, 182}, {177, 67, 112, 183}, {176, 180, 182, 177},
      {183, 181, 176, 177}, {134, 136, 175, 173}, {175, 136, 135, 174},
      {132, 134, 173, 171}, {174, 135, 133, 172}, {130, 132, 171, 169},
      {172, 133, 131, 170}, {165, 186, 184, 167}, {185, 187, 166, 168},
      {130, 169, 167, 184}, {168, 170, 131, 185}, {143, 189, 188, 186},
      {188, 189, 144, 187}, {184, 186, 188, 68}, {188, 187, 185, 68},
      {129, 130, 184, 68}, {185, 131, 129, 68}, {141, 192, 190, 143},
      {191, 193, 142, 144}, {139, 194, 192, 141}, {193, 195, 140, 142},
      {138, 196, 194, 139}, {195, 197, 138, 140}, {137, 70, 196, 138},
      {197, 70, 137, 138}, {189, 143, 190, 69}, {191, 144, 189, 69},
      {69, 190, 205, 207}, {206, 191, 69, 207}, {70, 198, 199, 196},
      {200, 198, 70, 197}, {196, 199, 201, 194}, {202, 200, 197, 195},
      {194, 201, 203, 192}, {204, 202, 195, 193}, {192, 203, 205, 190},
      {206, 204, 193, 191}, {198, 203, 201, 199}, {202, 204, 198, 200},
      {198, 207, 205, 203}, {206, 207, 198, 204}, {138, 139, 163, 176},
      {164, 140, 138, 176}, {139, 141, 210, 163}, {211, 142, 140, 164},
      {141, 143, 212, 210}, {213, 144, 142, 211}, {143, 186, 165, 212},
      {166, 187, 144, 213}, {80, 208, 212, 165}, {213, 209, 81, 166},
      {208, 214, 210, 212}, {211, 215, 209, 213}, {78, 163, 210, 214},
      {211, 164, 79, 215}, {130, 129, 71, 221}, {71, 129, 131, 222},
      {132, 130, 221, 219}, {222, 131, 133, 220}, {134, 132, 219, 217},
      {220, 133, 135, 218}, {136, 134, 217, 216}, {218, 135, 136, 216},
      {216, 217, 228, 230}, {229, 218, 216, 230}, {217, 219, 226, 228},
      {227, 220, 218, 229}, {219, 221, 224, 226}, {225, 222, 220, 227},
      {221, 71, 223, 224}, {223, 71, 222, 225}, {223, 230, 228, 224},
      {229, 230, 223, 225}, {182, 180, 233, 231}, {234, 181, 183, 232},
      {111, 182, 231, 253}, {232, 183, 112, 254}, {109, 111, 253, 255},
      {254, 112, 110, 256}, {180, 113, 251, 233}, {252, 114, 181, 234},
      {113, 115, 249, 251}, {250, 116, 114, 252}, {115, 117, 247, 249},
      {248, 118, 116, 250}, {117, 119, 245, 247}, {246, 120, 118, 248},
      {119, 121, 243, 245}, {244, 122, 120, 246}, {121, 123, 241, 243},
      {242, 124, 122, 244}, {123, 125, 239, 241}, {240, 126, 124, 242},
      {125, 178, 235, 239}, {236, 179, 126, 240}, {178, 127, 237, 235},
      {238, 128, 179, 236}, {127, 109, 255, 237}, {256, 110, 128, 238},
      {237, 255, 257, 275}, {258, 256, 238, 276}, {235, 237, 275, 277},
      {276, 238, 236, 278}, {239, 235, 277, 273}, {278, 236, 240, 274},
      {241, 239, 273, 271}, {274, 240, 242, 272}, {243, 241, 271, 269},
      {272, 242, 244, 270}, {245, 243, 269, 267}, {270, 244, 246, 268},
      {247, 245, 267, 265}, {268, 246, 248, 266}, {249, 247, 265, 263},
      {266, 248, 250, 264}, {251, 249, 263, 261}, {264, 250, 252, 262},
      {233, 251, 261, 279}, {262, 252, 234, 280}, {255, 253, 259, 257},
      {260, 254, 256, 258}, {253, 231, 281, 259}, {282, 232, 254, 260},
      {231, 233, 279, 281}, {280, 234, 232, 282}, {66, 107, 283, 72},
      {284, 108, 66, 72}, {107, 105, 285, 283}, {286, 106, 108, 284},
      {105, 103, 287, 285}, {288, 104, 106, 286}, {103, 101, 289, 287},
      {290, 102, 104, 288}, {101, 99, 291, 289}, {292, 100, 102, 290},
      {99, 97, 293, 291}, {294, 98, 100, 292}, {97, 95, 295, 293},
      {296, 96, 98, 294}, {95, 93, 297, 295}, {298, 94, 96, 296},
      {93, 91, 299, 297}, {300, 92, 94, 298}, {307, 308, 327, 337},
      {328, 308, 307, 338}, {306, 307, 337, 335}, {338, 307, 306, 336},
      {305, 306, 335, 339}, {336, 306, 305, 340}, {88, 90, 305, 339},
      {305, 90, 89, 340}, {86, 88, 339, 333}, {340, 89, 87, 334},
      {84, 86, 333, 329}, {334, 87, 85, 330}, {82, 84, 329, 331},
      {330, 85, 83, 332}, {329, 335, 337, 331}, {338, 336, 330, 332},
      {329, 333, 339, 335}, {340, 334, 330, 336}, {325, 331, 337, 327},
      {338, 332, 326, 328}, {80, 82, 331, 325}, {332, 83, 81, 326},
      {208, 341, 343, 214}, {344, 342, 209, 215}, {80, 325, 341, 208},
      {342, 326, 81, 209}, {78, 214, 343, 345}, {344, 215, 79, 346},
      {78, 345, 299, 91}, {300, 346, 79, 92}, {76, 323, 351, 303},
      {352, 324, 76, 303}, {303, 351, 349, 77}, {350, 352, 303, 77},
      {77, 349, 347, 304}, {348, 350, 77, 304}, {304, 347, 327, 308},
      {328, 348, 304, 308}, {325, 327, 347, 341}, {348, 328, 326, 342},
      {295, 297, 317, 309}, {318, 298, 296, 310}, {75, 315, 323, 76},
      {324, 316, 75, 76}, {301, 357, 355, 302}, {356, 358, 301, 302},
      {302, 355, 353, 74}, {354, 356, 302, 74}, {74, 353, 315, 75},
      {316, 354, 74, 75}, {291, 293, 361, 363}, {362, 294, 292, 364},
      {363, 361, 367, 365}, {368, 362, 364, 366}, {365, 367, 369, 371},
      {370, 368, 366, 372}, {371, 369, 375, 373}, {376, 370, 372, 374},
      {313, 377, 373, 375}, {374, 378, 314, 376}, {315, 353, 373, 377},
      {374, 354, 316, 378}, {353, 355, 371, 373}, {372, 356, 354, 374},
      {355, 357, 365, 371}, {366, 358, 356, 372}, {357, 359, 363, 365},
      {364, 360, 358, 366}, {289, 291, 363, 359}, {364, 292, 290, 360},
      {73, 359, 357, 301}, {358, 360, 73, 301}, {283, 285, 287, 289},
      {288, 286, 284, 290}, {283, 289, 359, 73}, {360, 290, 284, 73},
      {293, 295, 309, 361}, {310, 296, 294, 362}, {309, 311, 367, 361},
      {368, 312, 310, 362}, {311, 381, 369, 367}, {370, 382, 312, 368},
      {313, 375, 369, 381}, {370, 376, 314, 382}, {347, 349, 385, 383},
      {386, 350, 348, 384}, {317, 383, 385, 319}, {386, 384, 318, 320},
      {297, 299, 383, 317}, {384, 300, 298, 318}, {299, 343, 341, 383},
      {342, 344, 300, 384}, {313, 321, 379, 377}, {380, 322, 314, 378},
      {315, 377, 379, 323}, {380, 378, 316, 324}, {319, 385, 379, 321},
      {380, 386, 320, 322}, {349, 351, 379, 385}, {380, 352, 350, 386},
      {399, 387, 413, 401}, {414, 388, 400, 402}, {399, 401, 403, 397},
      {404, 402, 400, 398}, {397, 403, 405, 395}, {406, 404, 398, 396},
      {395, 405, 407, 393}, {408, 406, 396, 394}, {393, 407, 409, 391},
      {410, 408, 394, 392}, {391, 409, 411, 389}, {412, 410, 392, 390},
      {409, 419, 417, 411}, {418, 420, 410, 412}, {407, 421, 419, 409},
      {420, 422, 408, 410}, {405, 423, 421, 407}, {422, 424, 406, 408},
      {403, 425, 423, 405}, {424, 426, 404, 406}, {401, 427, 425, 403},
      {426, 428, 402, 404}, {401, 413, 415, 427}, {416, 414, 402, 428},
      {317, 319, 443, 441}, {444, 320, 318, 442}, {319, 389, 411, 443},
      {412, 390, 320, 444}, {309, 317, 441, 311}, {442, 318, 310, 312},
      {381, 429, 413, 387}, {414, 430, 382, 388}, {411, 417, 439, 443},
      {440, 418, 412, 444}, {437, 445, 443, 439}, {444, 446, 438, 440},
      {433, 445, 437, 435}, {438, 446, 434, 436}, {431, 447, 445, 433},
      {446, 448, 432, 434}, {429, 447, 431, 449}, {432, 448, 430, 450},
      {413, 429, 449, 415}, {450, 430, 414, 416}, {311, 447, 429, 381},
      {430, 448, 312, 382}, {311, 441, 445, 447}, {446, 442, 312, 448},
      {415, 449, 451, 475}, {452, 450, 416, 476}, {449, 431, 461, 451},
      {462, 432, 450, 452}, {431, 433, 459, 461}, {460, 434, 432, 462},
      {433, 435, 457, 459}, {458, 436, 434, 460}, {435, 437, 455, 457},
      {456, 438, 436, 458}, {437, 439, 453, 455}, {454, 440, 438, 456},
      {439, 417, 473, 453}, {474, 418, 440, 454}, {427, 415, 475, 463},
      {476, 416, 428, 464}, {425, 427, 463, 465}, {464, 428, 426, 466},
      {423, 425, 465, 467}, {466, 426, 424, 468}, {421, 423, 467, 469},
      {468, 424, 422, 470}, {419, 421, 469, 471}, {470, 422, 420, 472},
      {417, 419, 471, 473}, {472, 420, 418, 474}, {457, 455, 479, 477},
      {480, 456, 458, 478}, {477, 479, 481, 483}, {482, 480, 478, 484},
      {483, 481, 487, 485}, {488, 482, 484, 486}, {485, 487, 489, 491},
      {490, 488, 486, 492}, {463, 475, 485, 491}, {486, 476, 464, 492},
      {451, 483, 485, 475}, {486, 484, 452, 476}, {451, 461, 477, 483},
      {478, 462, 452, 484}, {457, 477, 461, 459}, {462, 478, 458, 460},
      {453, 473, 479, 455}, {480, 474, 454, 456}, {471, 481, 479, 473},
      {480, 482, 472, 474}, {469, 487, 481, 471}, {482, 488, 470, 472},
      {467, 489, 487, 469}, {488, 490, 468, 470}, {465, 491, 489, 467},
      {490, 492, 466, 468}, {391, 389, 503, 501}, {504, 390, 392, 502},
      {393, 391, 501, 499}, {502, 392, 394, 500}, {395, 393, 499, 497},
      {500, 394, 396, 498}, {397, 395, 497, 495}, {498, 396, 398, 496},
      {399, 397, 495, 493}, {496, 398, 400, 494}, {387, 399, 493, 505},
      {494, 400, 388, 506}, {493, 501, 503, 505}, {504, 502, 494, 506},
      {493, 495, 499, 501}, {500, 496, 494, 502}, {313, 381, 387, 505},
      {388, 382, 314, 506}, {313, 505, 503, 321}, {504, 506, 314, 322},
      {319, 321, 503, 389}, {504, 322, 320, 390},
      // ttriangles
      {60, 64, 48, 48}, {49, 65, 61, 61}, {62, 64, 60, 60}, {61, 65, 63, 63},
      {60, 58, 62, 62}, {63, 59, 61, 61}, {60, 56, 58, 58}, {59, 57, 61, 61},
      {60, 54, 56, 56}, {57, 55, 61, 61}, {60, 52, 54, 54}, {55, 53, 61, 61},
      {60, 50, 52, 52}, {53, 51, 61, 61}, {60, 48, 50, 50}, {51, 49, 61, 61},
      {224, 228, 226, 226}, {227, 229, 225, 255}, {72, 283, 73, 73},
      {73, 284, 72, 72}, {341, 347, 383, 383}, {384, 348, 342, 342},
      {299, 345, 343, 343}, {344, 346, 300, 300}, {323, 379, 351, 351},
      {352, 380, 324, 324}, {441, 443, 445, 445}, {446, 444, 442, 442},
      {463, 491, 465, 465}, {466, 492, 464, 464}, {495, 497, 499, 499},
      {500, 498, 496, 496}};
  quads                           = suzanne_quads;
  positions                       = suzanne_positions;
  if (scale != 1) {
    for (auto& p : positions) p *= scale;
  }
}
void make_quad(std::vector<vec4i>& quads, std::vector<vec3f>& positions,
    std::vector<vec3f>& normals, std::vector<vec2f>& texcoords, float scale) {
  static const auto quad_positions = std::vector<vec3f>{
      {-1, -1, 0}, {+1, -1, 0}, {+1, +1, 0}, {-1, +1, 0}};
  static const auto quad_normals = std::vector<vec3f>{
      {0, 0, 1}, {0, 0, 1}, {0, 0, 1}, {0, 0, 1}};
  static const auto quad_texcoords = std::vector<vec2f>{
      {0, 1}, {1, 1}, {1, 0}, {0, 0}};
  static const auto quad_quads = std::vector<vec4i>{{0, 1, 2, 3}};
  quads                        = quad_quads;
  positions                    = quad_positions;
  normals                      = quad_normals;
  texcoords                    = quad_texcoords;
  if (scale != 1) {
    for (auto& p : positions) p *= scale;
  }
}

void make_quady(std::vector<vec4i>& quads, std::vector<vec3f>& positions,
    std::vector<vec3f>& normals, std::vector<vec2f>& texcoords, float scale) {
  static const auto quady_positions = std::vector<vec3f>{
      {-1, 0, -1}, {-1, 0, +1}, {+1, 0, +1}, {+1, 0, -1}};
  static const auto quady_normals = std::vector<vec3f>{
      {0, 1, 0}, {0, 1, 0}, {0, 1, 0}, {0, 1, 0}};
  static const auto quady_texcoords = std::vector<vec2f>{
      {0, 0}, {1, 0}, {1, 1}, {0, 1}};
  static const auto quady_quads = std::vector<vec4i>{{0, 1, 2, 3}};
  quads                         = quady_quads;
  positions                     = quady_positions;
  normals                       = quady_normals;
  texcoords                     = quady_texcoords;
  if (scale != 1) {
    for (auto& p : positions) p *= scale;
  }
}

void make_cube(std::vector<vec4i>& quads, std::vector<vec3f>& positions,
    std::vector<vec3f>& normals, std::vector<vec2f>& texcoords, float scale) {
  static const auto cube_positions = std::vector<vec3f>{{-1, -1, +1},
      {+1, -1, +1}, {+1, +1, +1}, {-1, +1, +1}, {+1, -1, -1}, {-1, -1, -1},
      {-1, +1, -1}, {+1, +1, -1}, {+1, -1, +1}, {+1, -1, -1}, {+1, +1, -1},
      {+1, +1, +1}, {-1, -1, -1}, {-1, -1, +1}, {-1, +1, +1}, {-1, +1, -1},
      {-1, +1, +1}, {+1, +1, +1}, {+1, +1, -1}, {-1, +1, -1}, {+1, -1, +1},
      {-1, -1, +1}, {-1, -1, -1}, {+1, -1, -1}};
  static const auto cube_normals   = std::vector<vec3f>{{0, 0, +1}, {0, 0, +1},
      {0, 0, +1}, {0, 0, +1}, {0, 0, -1}, {0, 0, -1}, {0, 0, -1}, {0, 0, -1},
      {+1, 0, 0}, {+1, 0, 0}, {+1, 0, 0}, {+1, 0, 0}, {-1, 0, 0}, {-1, 0, 0},
      {-1, 0, 0}, {-1, 0, 0}, {0, +1, 0}, {0, +1, 0}, {0, +1, 0}, {0, +1, 0},
      {0, -1, 0}, {0, -1, 0}, {0, -1, 0}, {0, -1, 0}};
  static const auto cube_texcoords = std::vector<vec2f>{{0, 1}, {1, 1}, {1, 0},
      {0, 0}, {0, 1}, {1, 1}, {1, 0}, {0, 0}, {0, 1}, {1, 1}, {1, 0}, {0, 0},
      {0, 1}, {1, 1}, {1, 0}, {0, 0}, {0, 1}, {1, 1}, {1, 0}, {0, 0}, {0, 1},
      {1, 1}, {1, 0}, {0, 0}};
  static const auto cube_quads = std::vector<vec4i>{{0, 1, 2, 3}, {4, 5, 6, 7},
      {8, 9, 10, 11}, {12, 13, 14, 15}, {16, 17, 18, 19}, {20, 21, 22, 23}};
  quads                        = cube_quads;
  positions                    = cube_positions;
  normals                      = cube_normals;
  texcoords                    = cube_texcoords;
  if (scale != 1) {
    for (auto& p : positions) p *= scale;
  }
}

void make_fvcube(std::vector<vec4i>& quadspos, std::vector<vec4i>& quadsnorm,
    std::vector<vec4i>& quadstexcoord, std::vector<vec3f>& positions,
    std::vector<vec3f>& normals, std::vector<vec2f>& texcoords, float scale) {
  static const auto fvcube_positions = std::vector<vec3f>{{-1, -1, +1},
      {+1, -1, +1}, {+1, +1, +1}, {-1, +1, +1}, {+1, -1, -1}, {-1, -1, -1},
      {-1, +1, -1}, {+1, +1, -1}};
  static const auto fvcube_normals = std::vector<vec3f>{{0, 0, +1}, {0, 0, +1},
      {0, 0, +1}, {0, 0, +1}, {0, 0, -1}, {0, 0, -1}, {0, 0, -1}, {0, 0, -1},
      {+1, 0, 0}, {+1, 0, 0}, {+1, 0, 0}, {+1, 0, 0}, {-1, 0, 0}, {-1, 0, 0},
      {-1, 0, 0}, {-1, 0, 0}, {0, +1, 0}, {0, +1, 0}, {0, +1, 0}, {0, +1, 0},
      {0, -1, 0}, {0, -1, 0}, {0, -1, 0}, {0, -1, 0}};
  static const auto fvcube_texcoords     = std::vector<vec2f>{{0, 1}, {1, 1},
      {1, 0}, {0, 0}, {0, 1}, {1, 1}, {1, 0}, {0, 0}, {0, 1}, {1, 1}, {1, 0},
      {0, 0}, {0, 1}, {1, 1}, {1, 0}, {0, 0}, {0, 1}, {1, 1}, {1, 0}, {0, 0},
      {0, 1}, {1, 1}, {1, 0}, {0, 0}};
  static const auto fvcube_quadspos      = std::vector<vec4i>{{0, 1, 2, 3},
      {4, 5, 6, 7}, {1, 4, 7, 2}, {5, 0, 3, 6}, {3, 2, 7, 6}, {1, 0, 5, 4}};
  static const auto fvcube_quadsnorm     = std::vector<vec4i>{{0, 1, 2, 3},
      {4, 5, 6, 7}, {8, 9, 10, 11}, {12, 13, 14, 15}, {16, 17, 18, 19},
      {20, 21, 22, 23}};
  static const auto fvcube_quadstexcoord = std::vector<vec4i>{{0, 1, 2, 3},
      {4, 5, 6, 7}, {8, 9, 10, 11}, {12, 13, 14, 15}, {16, 17, 18, 19},
      {20, 21, 22, 23}};
  quadspos                               = fvcube_quadspos;
  quadsnorm                              = fvcube_quadsnorm;
  quadstexcoord                          = fvcube_quadstexcoord;
  positions                              = fvcube_positions;
  normals                                = fvcube_normals;
  texcoords                              = fvcube_texcoords;
  if (scale != 1) {
    for (auto& p : positions) p *= scale;
  }
}

void make_geosphere(
    std::vector<vec3i>& triangles, std::vector<vec3f>& positions, float scale) {
  // https://stackoverflow.com/questions/17705621/algorithm-for-a-geodesic-sphere
  const float X                   = 0.525731112119133606f;
  const float Z                   = 0.850650808352039932f;
  static auto geosphere_positions = std::vector<vec3f>{{-X, 0.0, Z},
      {X, 0.0, Z}, {-X, 0.0, -Z}, {X, 0.0, -Z}, {0.0, Z, X}, {0.0, Z, -X},
      {0.0, -Z, X}, {0.0, -Z, -X}, {Z, X, 0.0}, {-Z, X, 0.0}, {Z, -X, 0.0},
      {-Z, -X, 0.0}};
  static auto geosphere_triangles = std::vector<vec3i>{{0, 1, 4}, {0, 4, 9},
      {9, 4, 5}, {4, 8, 5}, {4, 1, 8}, {8, 1, 10}, {8, 10, 3}, {5, 8, 3},
      {5, 3, 2}, {2, 3, 7}, {7, 3, 10}, {7, 10, 6}, {7, 6, 11}, {11, 6, 0},
      {0, 6, 1}, {6, 10, 1}, {9, 11, 0}, {9, 2, 11}, {9, 5, 2}, {7, 11, 2}};
  triangles                       = geosphere_triangles;
  positions                       = geosphere_positions;
  if (scale != 1) {
    for (auto& p : positions) p *= scale;
  }
}

// Make a hair ball around a shape
void make_hair(std::vector<vec2i>& lines, std::vector<vec3f>& positions,
    std::vector<vec3f>& normals, std::vector<vec2f>& texcoords,
    std::vector<float>& radius, const std::vector<vec3i>& striangles,
    const std::vector<vec4i>& squads, const std::vector<vec3f>& spos,
    const std::vector<vec3f>& snorm, const std::vector<vec2f>& stexcoord,
    const vec2i& steps, const vec2f& len, const vec2f& rad, const vec2f& noise,
    const vec2f& clump, const vec2f& rotation, int seed) {
  auto alltriangles    = striangles;
  auto quads_triangles = quads_to_triangles(squads);
  alltriangles.insert(
      alltriangles.end(), quads_triangles.begin(), quads_triangles.end());
  auto bpos      = std::vector<vec3f>{};
  auto bnorm     = std::vector<vec3f>{};
  auto btexcoord = std::vector<vec2f>{};
  sample_triangles(bpos, bnorm, btexcoord, alltriangles, spos, snorm, stexcoord,
      steps.y, seed);

  auto rng  = make_rng(seed, 3);
  auto blen = std::vector<float>(bpos.size());
  for (auto& l : blen) {
    l = lerp(len.x, len.y, rand1f(rng));
  }

  auto cidx = std::vector<int>();
  if (clump.x > 0) {
    for (auto bidx = 0; bidx < bpos.size(); bidx++) {
      cidx.push_back(0);
      auto cdist = flt_max;
      for (auto c = 0; c < clump.y; c++) {
        auto d = length(bpos[bidx] - bpos[c]);
        if (d < cdist) {
          cdist       = d;
          cidx.back() = c;
        }
      }
    }
  }

  make_lines(lines, positions, normals, texcoords, radius, steps, {1, 1},
      {1, 1}, {1, 1});
  for (auto i = 0; i < positions.size(); i++) {
    auto u       = texcoords[i].x;
    auto bidx    = i / (steps.x + 1);
    positions[i] = bpos[bidx] + bnorm[bidx] * u * blen[bidx];
    normals[i]   = bnorm[bidx];
    radius[i]    = lerp(rad.x, rad.y, u);
    if (clump.x > 0) {
      positions[i] =
          positions[i] +
          (positions[i + (cidx[bidx] - bidx) * (steps.x + 1)] - positions[i]) *
              u * clump.x;
    }
    if (noise.x > 0) {
      auto nx = perlin_noise(positions[i] * noise.y + vec3f{0, 0, 0}) * noise.x;
      auto ny = perlin_noise(positions[i] * noise.y + vec3f{3, 7, 11}) *
                noise.x;
      auto nz = perlin_noise(positions[i] * noise.y + vec3f{13, 17, 19}) *
                noise.x;
      positions[i] += {nx, ny, nz};
    }
  }

  if (clump.x > 0 || noise.x > 0 || rotation.x > 0) {
    normals = compute_tangents(lines, positions);
  }
}

// Thickens a shape by copy9ing the shape content, rescaling it and flipping
// its normals. Note that this is very much not robust and only useful for
// trivial cases.
void make_shell(std::vector<vec4i>& quads, std::vector<vec3f>& positions,
    std::vector<vec3f>& normals, std::vector<vec2f>& texcoords,
    float thickness) {
  auto bbox = math::invalidb3f;
  for (auto p : positions) bbox = merge(bbox, p);
  auto center              = math::center(bbox);
  auto inner_quads         = quads;
  auto inner_positions     = positions;
  auto inner_normals       = normals;
  auto inner_texturecoords = texcoords;
  for (auto& p : inner_positions) p = (1 - thickness) * (p - center) + center;
  for (auto& n : inner_normals) n = -n;
  merge_quads(quads, positions, normals, texcoords, inner_quads,
      inner_positions, inner_normals, inner_texturecoords);
}

// Make a heightfield mesh.
void make_heightfield(std::vector<vec4i>& quads, std::vector<vec3f>& positions,
    std::vector<vec3f>& normals, std::vector<vec2f>& texcoords,
    const vec2i& size, const std::vector<float>& height) {
  make_yrect(quads, positions, normals, texcoords, size - 1,
      vec2f{(float)size.x, (float)size.y} / max(size));
  for (auto j = 0; j < size.y; j++)
    for (auto i = 0; i < size.x; i++)
      positions[j * size.x + i].y = height[j * size.x + i];
  normals = compute_normals(quads, positions);
}

}  // namespace yocto::shape

// -----------------------------------------------------------------------------
// IMPLEMENTATION OF SHAPE IO
// -----------------------------------------------------------------------------
namespace yocto::shape {

// Get extension (not including '.').
static std::string get_extension(const std::string& filename) {
  auto pos = filename.rfind('.');
  if (pos == std::string::npos) return "";
  return filename.substr(pos);
}

// Load ply mesh
[[nodiscard]] bool load_shape(const std::string& filename,
    std::vector<int>& points, std::vector<vec2i>& lines,
    std::vector<vec3i>& triangles, std::vector<vec4i>& quads,
    std::vector<vec3f>& positions, std::vector<vec3f>& normals,
    std::vector<vec2f>& texcoords, std::vector<vec3f>& colors,
    std::vector<float>& radius, std::string& error, bool flip_texcoord) {
  auto format_error = [filename, &error]() {
    error = filename + ": unknown format";
    return false;
  };
  auto shape_error = [filename, &error]() {
    error = filename + ": empty shape";
    return false;
  };

  points    = {};
  lines     = {};
  triangles = {};
  quads     = {};
  positions = {};
  normals   = {};
  texcoords = {};
  colors    = {};
  radius    = {};

  auto ext = get_extension(filename);
  if (ext == ".ply" || ext == ".PLY") {
    // open ply
    auto ply_guard = std::make_unique<ply::model>();
    auto ply       = ply_guard.get();
    if (!load_ply(filename, ply, error)) return false;

    // gets vertex
    get_positions(ply, positions);
    get_normals(ply, normals);
    get_texcoords(ply, texcoords, flip_texcoord);
    get_colors(ply, colors);
    get_radius(ply, radius);

    // get faces
    if (has_quads(ply)) {
      get_quads(ply, quads);
    } else {
      get_triangles(ply, triangles);
    }
    get_lines(ply, lines);
    get_points(ply, points);

    if (positions.empty()) return shape_error();
    return true;
  } else if (ext == ".obj" || ext == ".OBJ") {
    // load obj
    auto obj_guard = std::make_unique<obj::model>();
    auto obj       = obj_guard.get();
    if (!load_obj(filename, obj, error, true)) return false;

    // get shape
    if (obj->shapes.empty()) return shape_error();
    if (obj->shapes.size() > 1) return shape_error();
    auto shape = obj->shapes.front();
    if (shape->points.empty() && shape->lines.empty() && shape->faces.empty())
      return shape_error();

    // decide what to do and get properties
    auto materials  = std::vector<obj::material*>{};
    auto ematerials = std::vector<int>{};
    auto has_quads_ = has_quads(shape);
    if (!shape->faces.empty() && !has_quads_) {
      get_triangles(shape, triangles, positions, normals, texcoords, materials,
          ematerials, flip_texcoord);
    } else if (!shape->faces.empty() && has_quads_) {
      get_quads(shape, quads, positions, normals, texcoords, materials,
          ematerials, flip_texcoord);
    } else if (!shape->lines.empty()) {
      get_lines(shape, lines, positions, normals, texcoords, materials,
          ematerials, flip_texcoord);
    } else if (!shape->points.empty()) {
      get_points(shape, points, positions, normals, texcoords, materials,
          ematerials, flip_texcoord);
    } else {
      return shape_error();
    }

    if (positions.empty()) return shape_error();
    return true;
  } else {
    return format_error();
  }
}

// Save ply mesh
[[nodiscard]] bool save_shape(const std::string& filename,
    const std::vector<int>& points, const std::vector<vec2i>& lines,
    const std::vector<vec3i>& triangles, const std::vector<vec4i>& quads,
    const std::vector<vec3f>& positions, const std::vector<vec3f>& normals,
    const std::vector<vec2f>& texcoords, const std::vector<vec3f>& colors,
    const std::vector<float>& radius, std::string& error, bool ascii,
    bool flip_texcoord) {
  auto format_error = [filename, &error]() {
    error = filename + ": unknown format";
    return false;
  };
  auto shape_error = [filename, &error]() {
    error = filename + ": empty shape";
    return false;
  };

  auto ext = get_extension(filename);
  if (ext == ".ply" || ext == ".PLY") {
    // create ply
    auto ply_guard = std::make_unique<ply::model>();
    auto ply       = ply_guard.get();
    add_positions(ply, positions);
    add_normals(ply, normals);
    add_texcoords(ply, texcoords, flip_texcoord);
    add_colors(ply, colors);
    add_radius(ply, radius);
    add_faces(ply, triangles, quads);
    add_lines(ply, lines);
    add_points(ply, points);
    if (!save_ply(filename, ply, error)) return false;
    return true;
  } else if (ext == ".obj" || ext == ".OBJ") {
    auto obj_guard = std::make_unique<obj::model>();
    auto obj       = obj_guard.get();
    auto oshape    = add_shape(obj);
    if (!triangles.empty()) {
      set_triangles(
          oshape, triangles, positions, normals, texcoords, {}, flip_texcoord);
    } else if (!quads.empty()) {
      set_quads(
          oshape, quads, positions, normals, texcoords, {}, flip_texcoord);
    } else if (!lines.empty()) {
      set_lines(
          oshape, lines, positions, normals, texcoords, {}, flip_texcoord);
    } else if (!points.empty()) {
      set_points(
          oshape, points, positions, normals, texcoords, {}, flip_texcoord);
    } else {
      return shape_error();
    }
    auto err = ""s;
    if (!save_obj(filename, obj, error)) return false;
    return true;
  } else {
    return format_error();
  }
}

// Load ply mesh
[[nodiscard]] bool load_fvshape(const std::string& filename,
    std::vector<vec4i>& quadspos, std::vector<vec4i>& quadsnorm,
    std::vector<vec4i>& quadstexcoord, std::vector<vec3f>& positions,
    std::vector<vec3f>& normals, std::vector<vec2f>& texcoords,
    std::string& error, bool flip_texcoord) {
  auto format_error = [filename, &error]() {
    error = filename + ": unknown format";
    return false;
  };
  auto shape_error = [filename, &error]() {
    error = filename + ": empty shape";
    return false;
  };

  quadspos      = {};
  quadsnorm     = {};
  quadstexcoord = {};
  positions     = {};
  normals       = {};
  texcoords     = {};

  auto ext = get_extension(filename);
  if (ext == ".ply" || ext == ".PLY") {
    auto ply_guard = std::make_unique<ply::model>();
    auto ply       = ply_guard.get();
    if (!load_ply(filename, ply, error)) return false;
    get_positions(ply, positions);
    get_normals(ply, normals);
    get_texcoords(ply, texcoords, flip_texcoord);
    get_quads(ply, quadspos);
    if (!normals.empty()) quadsnorm = quadspos;
    if (!texcoords.empty()) quadstexcoord = quadspos;
    if (positions.empty()) return shape_error();
    return true;
  } else if (ext == ".obj" || ext == ".OBJ") {
    auto obj_guard = std::make_unique<obj::model>();
    auto obj       = obj_guard.get();
    if (!load_obj(filename, obj, error, true)) return false;
    if (obj->shapes.empty()) return shape_error();
    if (obj->shapes.size() > 1) return shape_error();
    auto shape = obj->shapes.front();
    if (shape->faces.empty()) return shape_error();
    auto materials  = std::vector<obj::material*>{};
    auto ematerials = std::vector<int>{};
    get_fvquads(shape, quadspos, quadsnorm, quadstexcoord, positions, normals,
        texcoords, materials, ematerials, flip_texcoord);
    if (positions.empty()) return shape_error();
    return true;
  } else {
    return format_error();
  }
}

// Save ply mesh
[[nodiscard]] bool save_fvshape(const std::string& filename,
    const std::vector<vec4i>& quadspos, const std::vector<vec4i>& quadsnorm,
    const std::vector<vec4i>& quadstexcoord,
    const std::vector<vec3f>& positions, const std::vector<vec3f>& normals,
    const std::vector<vec2f>& texcoords, std::string& error, bool ascii,
    bool flip_texcoord) {
  auto format_error = [filename, &error]() {
    error = filename + ": unknown format";
    return false;
  };
  auto shape_error = [filename, &error]() {
    error = filename + ": empty shape";
    return false;
  };

  auto ext = get_extension(filename);
  if (ext == ".ply" || ext == ".PLY") {
    // split data
    auto [split_quads, split_positions, split_normals, split_texcoords] =
        split_facevarying(
            quadspos, quadsnorm, quadstexcoord, positions, normals, texcoords);

    // ply model
    auto ply_guard = std::make_unique<ply::model>();
    auto ply       = ply_guard.get();
    add_positions(ply, split_positions);
    add_normals(ply, split_normals);
    add_texcoords(ply, split_texcoords, flip_texcoord);
    add_faces(ply, {}, split_quads);

    // save
    if (!save_ply(filename, ply, error)) return false;
    return true;
  } else if (ext == ".obj" || ext == ".OBJ") {
    // Obj model
    auto obj_guard = std::make_unique<obj::model>();
    auto obj       = obj_guard.get();

    // Add obj data
    auto oshape = add_shape(obj);
    set_fvquads(oshape, quadspos, quadsnorm, quadstexcoord, positions, normals,
        texcoords, {}, flip_texcoord);

    // Save
    if (!save_obj(filename, obj, error)) return false;
    return true;
  } else {
    return format_error();
  }
}

}  // namespace yocto::shape

// -----------------------------------------------------------------------------
// IMPLEMENTATION OF SHAPE STATS AND VALIDATION
// -----------------------------------------------------------------------------
namespace yocto::shape {

std::vector<std::string> shape_stats(const std::vector<int>& points,
    const std::vector<vec2i>& lines, const std::vector<vec3i>& triangles,
    const std::vector<vec4i>& quads, const std::vector<vec4i>& quadspos,
    const std::vector<vec4i>& quadsnorm,
    const std::vector<vec4i>& quadstexcoord,
    const std::vector<vec3f>& positions, const std::vector<vec3f>& normals,
    const std::vector<vec2f>& texcoords, const std::vector<vec3f>& colors,
    const std::vector<float>& radius, bool verbose) {
  auto format = [](auto num) {
    auto str = std::to_string(num);
    while (str.size() < 13) str = " " + str;
    return str;
  };
  auto format3 = [](auto num) {
    auto str = std::to_string(num.x) + " " + std::to_string(num.y) + " " +
               std::to_string(num.z);
    while (str.size() < 13) str = " " + str;
    return str;
  };

  auto bbox = invalidb3f;
  for (auto& pos : positions) bbox = merge(bbox, pos);

  auto stats = std::vector<std::string>{};
  stats.push_back("points:       " + format(points.size()));
  stats.push_back("lines:        " + format(lines.size()));
  stats.push_back("triangles:    " + format(triangles.size()));
  stats.push_back("quads:        " + format(quads.size()));
  stats.push_back("fvquads:      " + format(quadspos.size()));
  stats.push_back("positions:    " + format(positions.size()));
  stats.push_back("normals:      " + format(normals.size()));
  stats.push_back("texcoords:    " + format(texcoords.size()));
  stats.push_back("colors:       " + format(colors.size()));
  stats.push_back("radius:       " + format(radius.size()));
  stats.push_back("center:       " + format3(center(bbox)));
  stats.push_back("size:         " + format3(size(bbox)));
  stats.push_back("min:          " + format3(bbox.min));
  stats.push_back("max:          " + format3(bbox.max));

  return stats;
}

}  // namespace yocto::shape
