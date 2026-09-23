// Screened Poisson surface reconstruction behind a C ABI.
//
// This is the in-memory path of Kazhdan's PoissonRecon (the same solver the
// PoissonRecon executable and MeshLab wrap), instantiated once for float
// samples in 3D with the default degree-1 Neumann elements. The library is
// multi-threaded through its own thread pool; it is switched to std::async
// here because OpenMP would drag in vcomp140.dll and the app ships as one exe.

// The library's fatal-error macro calls exit(); an app must never do that, so
// route it through an exception like the rest of its errors.
#define MK_ERROR_OUT(...) Throw(__FILE__, __LINE__, __FUNCTION__, __VA_ARGS__)

#include "PreProcessor.h"
#include "Reconstructors.h"

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <atomic>
#include <exception>
#include <mutex>
#include <new>
#include <thread>
#include <vector>

#include "poisson_bridge.h"

using namespace PoissonRecon;

namespace {

using Real = float;
constexpr unsigned int kDim = 3;

// Oriented samples straight out of the caller's arrays. The threaded read is
// overridden too: the library's default serialises every threaded read
// through a mutex, which made its parallel stages slower than serial ones.
struct ArraySampleStream : public Reconstructor::InputOrientedSampleStream<Real, kDim> {
  ArraySampleStream(const float *p, const float *n, size_t count)
      : points_(p), normals_(n), count_(count) {}

  void reset(void) override { cursor_.store(0); }

  bool read(Point<Real, kDim> &p, Point<Real, kDim> &n) override { return take(p, n); }

  bool read(unsigned int, Point<Real, kDim> &p, Point<Real, kDim> &n) override { return take(p, n); }

 private:
  bool take(Point<Real, kDim> &p, Point<Real, kDim> &n) {
    const size_t i = cursor_.fetch_add(1);
    if (i >= count_) return false;
    for (unsigned int d = 0; d < kDim; d++) {
      p[d] = points_[3 * i + d];
      n[d] = normals_[3 * i + d];
    }
    return true;
  }
  const float *points_;
  const float *normals_;
  size_t count_;
  std::atomic<size_t> cursor_{0};
};

// Level-set vertices, with the density estimate that rides along with each.
//
// Extraction writes from every worker thread at once; the library's default
// threaded write takes a mutex per vertex, which caps the whole solve at a
// few cores. Here each thread appends to its own bucket and indices come from
// one atomic counter, so the scatter back into order happens once at the end.
struct VertexSink : public Reconstructor::OutputLevelSetVertexStream<Real, kDim> {
  struct Rec {
    size_t idx;
    float x, y, z, w;
  };
  explicit VertexSink(size_t threads) : buckets_(threads + 1) {}

  size_t size(void) const override { return count_.load(); }

  size_t write(const Point<Real, kDim> &p, const Point<Real, kDim> &g, const Real &w) override {
    std::lock_guard<std::mutex> lock(serial_);
    return put(buckets_.size() - 1, p, w);
  }

  size_t write(unsigned int thread, const Point<Real, kDim> &p, const Point<Real, kDim> &g,
               const Real &w) override {
    if (thread + 1 >= buckets_.size()) return write(p, g, w);
    return put(thread, p, w);
  }

  // Vertices in index order: xyz interleaved, and the densities.
  void gather(std::vector<float> &xyz, std::vector<float> &density) const {
    const size_t n = count_.load();
    xyz.assign(n * 3, 0.f);
    density.assign(n, 0.f);
    for (const auto &b : buckets_)
      for (const Rec &r : b) {
        xyz[3 * r.idx + 0] = r.x;
        xyz[3 * r.idx + 1] = r.y;
        xyz[3 * r.idx + 2] = r.z;
        density[r.idx] = r.w;
      }
  }

 private:
  size_t put(size_t bucket, const Point<Real, kDim> &p, const Real &w) {
    const size_t i = count_.fetch_add(1);
    buckets_[bucket].push_back(Rec{i, (float)p[0], (float)p[1], (float)p[2], (float)w});
    return i;
  }
  std::atomic<size_t> count_{0};
  std::vector<std::vector<Rec>> buckets_;
  std::mutex serial_;
};

// Faces arrive as polygons; with polygonMesh off they are always triangles,
// but anything larger is fanned rather than trusted. Per-thread buckets for
// the same reason as the vertices.
struct FaceSink : public Reconstructor::OutputFaceStream<2> {
  explicit FaceSink(size_t threads) : buckets_(threads + 1) {}

  size_t size(void) const override { return count_.load(); }

  size_t write(const std::vector<node_index_type> &poly) override {
    std::lock_guard<std::mutex> lock(serial_);
    return put(buckets_.size() - 1, poly);
  }

  size_t write(unsigned int thread, const std::vector<node_index_type> &poly) override {
    if (thread + 1 >= buckets_.size()) return write(poly);
    return put(thread, poly);
  }

  void gather(std::vector<uint32_t> &tris) const {
    size_t n = 0;
    for (const auto &b : buckets_) n += b.size();
    tris.clear();
    tris.reserve(n);
    for (const auto &b : buckets_) tris.insert(tris.end(), b.begin(), b.end());
  }

 private:
  size_t put(size_t bucket, const std::vector<node_index_type> &poly) {
    auto &out = buckets_[bucket];
    for (size_t k = 2; k < poly.size(); k++) {
      out.push_back((uint32_t)poly[0]);
      out.push_back((uint32_t)poly[k - 1]);
      out.push_back((uint32_t)poly[k]);
    }
    return count_.fetch_add(1);
  }
  std::atomic<size_t> count_{0};
  std::vector<std::vector<uint32_t>> buckets_;
  std::mutex serial_;
};

void set_error(PrResult *out, const char *msg) {
  if (!out) return;
  std::strncpy(out->error, msg ? msg : "unknown error", sizeof(out->error) - 1);
  out->error[sizeof(out->error) - 1] = '\0';
}

template <typename T>
T *copy_out(const std::vector<T> &v) {
  if (v.empty()) return nullptr;
  T *p = static_cast<T *>(std::malloc(v.size() * sizeof(T)));
  if (!p) throw std::bad_alloc();
  std::memcpy(p, v.data(), v.size() * sizeof(T));
  return p;
}

// ThreadPool's settings are process-wide statics, and ParallelFor keeps a
// static futures vector, so reconstructions are serialised.
std::mutex g_solver_mutex;

}  // namespace

extern "C" int pr_reconstruct(const float *points, const float *normals, size_t count,
                              const PrParams *params, PrResult *out) {
  if (!out) return 1;
  std::memset(out, 0, sizeof(*out));
  if (!points || !normals || !params) {
    set_error(out, "null argument");
    return 1;
  }
  if (count < 16) {
    set_error(out, "too few points to reconstruct");
    return 1;
  }

  std::lock_guard<std::mutex> lock(g_solver_mutex);
  try {
    ThreadPool::ParallelizationType = ThreadPool::ParallelType::ASYNC;
    ThreadPool::SetNumThreads(params->threads > 0 ? (unsigned int)params->threads : 0);

    using Poisson = Reconstructor::Poisson;
    static const unsigned int FEMSig =
        FEMDegreeAndBType<Poisson::DefaultFEMDegree, Poisson::DefaultFEMBoundary>::Signature;
    using FEMSigs = IsotropicUIntPack<kDim, FEMSig>;
    using Implicit = Reconstructor::Implicit<Real, kDim, FEMSigs>;
    using Solver = Poisson::Solver<Real, kDim, FEMSigs>;

    Poisson::SolutionParameters<Real> sp;
    sp.verbose = std::getenv("XSCAN_PR_VERBOSE") != nullptr;
    sp.depth = (unsigned int)std::clamp(params->depth, 4, 16);
    sp.fullDepth = (unsigned int)std::clamp(params->full_depth, 2, (int)sp.depth);
    sp.samplesPerNode = params->samples_per_node > 0 ? params->samples_per_node : (Real)1.5;
    sp.pointWeight = params->point_weight >= 0 ? params->point_weight
                                               : (Real)(Poisson::WeightMultiplier *
                                                        Poisson::DefaultFEMDegree);
    sp.scale = params->scale > 1 ? params->scale : (Real)1.1;
    sp.iters = (unsigned int)std::clamp(params->iters, 1, 64);

    Reconstructor::LevelSetExtractionParameters ep;
    ep.linearFit = params->linear_fit != 0;
    ep.outputDensity = true;
    ep.forceManifold = true;
    ep.polygonMesh = false;
    ep.verbose = sp.verbose;

    ArraySampleStream samples(points, normals, count);
    Implicit *implicit = Solver::Solve(samples, sp);
    if (!implicit) {
      set_error(out, "solver returned no implicit function");
      return 1;
    }

    const size_t threads = (size_t)std::max<unsigned int>(ThreadPool::NumThreads(), std::thread::hardware_concurrency()) + 1;
    VertexSink verts(threads);
    FaceSink faces(threads);
    try {
      implicit->extractLevelSet(verts, faces, ep);
    } catch (...) {
      delete implicit;
      throw;
    }
    delete implicit;

    std::vector<float> xyz, density;
    std::vector<uint32_t> tris;
    verts.gather(xyz, density);
    faces.gather(tris);
    out->n_verts = density.size();
    out->n_tris = tris.size() / 3;
    out->verts = copy_out(xyz);
    out->density = copy_out(density);
    out->tris = copy_out(tris);
    return 0;
  } catch (const std::bad_alloc &) {
    pr_free(out);
    set_error(out, "out of memory - lower the detail or the point budget");
  } catch (const std::exception &e) {
    pr_free(out);
    set_error(out, e.what());
  } catch (...) {
    pr_free(out);
    set_error(out, "reconstruction failed");
  }
  return 1;
}

extern "C" void pr_free(PrResult *result) {
  if (!result) return;
  std::free(result->verts);
  std::free(result->density);
  std::free(result->tris);
  result->verts = nullptr;
  result->density = nullptr;
  result->tris = nullptr;
  result->n_verts = 0;
  result->n_tris = 0;
}
