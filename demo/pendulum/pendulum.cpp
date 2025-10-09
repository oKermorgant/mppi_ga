

#include <mppi_ga/types.h>
#include "pendulum.h"
#include <Eigen/QR>

using namespace mppi_ga;

constexpr Float g{9.81};

Pend::Pend()
{
  setMaxCommand(10.);
}

PendState Pend::xNext(const PendState &x, const Vec &u, Float dt)
{
  auto xNext = x;

  // update state with semi-implicit euler
  const auto s{std::sin(xNext.theta)};
  const auto c{std::cos(xNext.theta)};
  const auto tdd = (u(0)*c+(M+m)*g*s - L*m*c*x.tdot*x.tdot)/(L*(M+m*s*s));

  xNext.tdot += half*tdd*dt;
  xNext.theta += x.tdot*dt;
  xNext.tdot += half*tdd*dt;
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
