// Minimal C++ wrapper over nlopt.h for systems where nlopt.hpp is not installed.
// Covers exactly the API used by trac_ik: nlopt::opt with LD_SLSQP, vector
// objective/constraint callbacks, bounds, time limit, and force_stop.

#pragma once

#include <nlopt.h>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <vector>

namespace nlopt {

using algorithm = nlopt_algorithm;

constexpr algorithm LD_SLSQP = NLOPT_LD_SLSQP;

class forced_stop : public std::exception {
public:
  const char* what() const noexcept override { return "nlopt: forced stop"; }
};

class roundoff_limited : public std::exception {
public:
  const char* what() const noexcept override { return "nlopt: roundoff limited"; }
};

// C++ callback types matching the trac_ik call sites
typedef double (*vfunc)(const std::vector<double>&, std::vector<double>&, void*);
typedef void (*mfunc)(unsigned, double*, unsigned, const double*, double*, void*);

class opt {
public:
  opt() : o_(nullptr) {}

  opt(algorithm a, unsigned n) : o_(nlopt_create(static_cast<nlopt_algorithm>(a), n)) {
    if (!o_) throw std::runtime_error("nlopt: failed to create optimizer");
  }

  ~opt() { if (o_) nlopt_destroy(o_); }

  opt(const opt&) = delete;
  opt& operator=(const opt&) = delete;

  opt(opt&& other) noexcept
    : o_(other.o_), vdata_(std::move(other.vdata_)), mdata_(std::move(other.mdata_))
  {
    other.o_ = nullptr;
    // Heap pointers stored in vdata_/mdata_ have not moved; NLopt's references remain valid.
  }

  opt& operator=(opt&& other) noexcept {
    if (this != &other) {
      if (o_) nlopt_destroy(o_);
      o_ = other.o_;
      vdata_ = std::move(other.vdata_);
      mdata_ = std::move(other.mdata_);
      other.o_ = nullptr;
    }
    return *this;
  }

  void set_min_objective(vfunc f, void* data) {
    vdata_ = std::make_unique<VFuncData>(VFuncData{f, data});
    nlopt_set_min_objective(o_, vfunc_c_wrapper, vdata_.get());
  }

  void add_equality_mconstraint(mfunc fc, void* data, const std::vector<double>& tol) {
    mdata_ = std::make_unique<MFuncData>(MFuncData{fc, data});
    nlopt_add_equality_mconstraint(
      o_, static_cast<unsigned>(tol.size()), mfunc_c_wrapper, mdata_.get(), tol.data());
  }

  void set_lower_bounds(const std::vector<double>& lb) {
    nlopt_set_lower_bounds(o_, lb.data());
  }

  void set_upper_bounds(const std::vector<double>& ub) {
    nlopt_set_upper_bounds(o_, ub.data());
  }

  void set_maxtime(double t) { nlopt_set_maxtime(o_, t); }

  void set_xtol_abs(double tol) { nlopt_set_xtol_abs1(o_, tol); }

  void force_stop() { if (o_) nlopt_force_stop(o_); }

  void optimize(std::vector<double>& x, double& opt_f) {
    if (!o_) throw std::runtime_error("nlopt: uninitialized optimizer");
    nlopt_result rc = nlopt_optimize(o_, x.data(), &opt_f);
    if (rc == NLOPT_FORCED_STOP) throw forced_stop();
    if (rc == NLOPT_ROUNDOFF_LIMITED) throw roundoff_limited();
    if (rc < 0) throw std::runtime_error("nlopt: optimization failed");
  }

private:
  nlopt_opt o_;

  struct VFuncData { vfunc f; void* data; };
  struct MFuncData { mfunc f; void* data; };

  std::unique_ptr<VFuncData> vdata_;
  std::unique_ptr<MFuncData> mdata_;

  static double vfunc_c_wrapper(unsigned n, const double* x, double* grad, void* d) {
    auto* wd = static_cast<VFuncData*>(d);
    std::vector<double> xv(x, x + n);
    std::vector<double> gv(grad ? n : 0);
    double result = wd->f(xv, gv, wd->data);
    if (grad && !gv.empty()) std::memcpy(grad, gv.data(), n * sizeof(double));
    return result;
  }

  static void mfunc_c_wrapper(unsigned m, double* result, unsigned n,
                               const double* x, double* grad, void* d) {
    auto* wd = static_cast<MFuncData*>(d);
    wd->f(m, result, n, x, grad, wd->data);
  }
};

}  // namespace nlopt
