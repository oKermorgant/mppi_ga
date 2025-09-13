

#include <mppi_ga/types.h>
#include "pendulum.h"
#include <Eigen/QR>

using namespace mppi_ga;

constexpr Float g{9.81};

Pend::Pend()
{
  setMaxCommand(2.);
}

PendState Pend::xNext(const PendState &x, const Vec &u, Float dt)
{
  auto xNext = x;

  // update state with semi-implicit euler
  const auto accel{(-g/L*std::sin(x.theta) + u(0)/M)};
  xNext.tdot += half*accel*dt;
  xNext.theta += xNext.tdot*dt;
  xNext.tdot += half*accel*dt;
  return xNext;
}

// define ref @ time t
void Pend::reference(Float t, PendState &xr) const
{
  xr.tdot = xr.theta = 0.;
}

// define vectorized error between two states
Eigen::Vector<Float,1> Pend::error(const PendState &x, const PendState &xr) const
{
  return Eigen::Vector<Float,1>{x.theta};
}
