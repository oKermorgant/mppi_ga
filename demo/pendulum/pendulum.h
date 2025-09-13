#ifndef MPPI_GA_PEND
#define MPPI_GA_PEND


#include <mppi_ga/types.h>
#include <mppi_ga/model.h>
#include <mppi_ga/states.h>

using namespace mppi_ga;

struct PendState
{
  Float theta{};
  Float tdot{};
};

struct Pend : public mppi_ga::Model<PendState, 1, 1>
{
  static constexpr Float M{1.};
  static constexpr Float L{1.};
  Pend();

  PendState xNext(const PendState &x, const Vec &u, Float dt) override;

  // define ref @ time t
  virtual void reference(Float t, PendState &xr) const override;

  // define vectorized error between two states
  virtual Eigen::Vector<Float,1> error(const PendState &x, const PendState &xr) const override;


};


#endif
