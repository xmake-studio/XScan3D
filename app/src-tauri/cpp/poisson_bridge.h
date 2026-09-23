// C ABI over Kazhdan's screened Poisson reconstruction, for src/poisson.rs.
//
// Everything crossing this boundary is plain arrays: the caller hands in
// oriented points, gets back a triangle mesh plus the per-vertex sampling
// density (which is what trimming by support reads). The arrays in the result
// are allocated here and must go back through pr_free().

#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct PrParams {
  int depth;               // maximum octree depth
  int full_depth;          // depth down to which the tree is complete
  float samples_per_node;  // minimum samples per leaf; higher smooths noise
  float point_weight;      // screening weight (interpolation strength)
  float scale;             // bounding cube / bounding box ratio
  int iters;               // Gauss-Seidel iterations per level
  int threads;             // 0 = all hardware threads
  int linear_fit;          // place vertices by linear fit instead of midpoints
} PrParams;

typedef struct PrResult {
  float *verts;            // xyz interleaved, n_verts * 3
  float *density;          // n_verts
  uint32_t *tris;          // n_tris * 3
  size_t n_verts;
  size_t n_tris;
  char error[512];
} PrResult;

// Returns 0 on success. On failure `out->error` says why and nothing needs to
// be freed (pr_free is still safe to call).
int pr_reconstruct(const float *points, const float *normals, size_t count,
                   const PrParams *params, PrResult *out);

void pr_free(PrResult *result);

#ifdef __cplusplus
}
#endif
