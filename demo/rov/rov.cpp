

#include <mppi_ga/types.h>
#include "rov.h"
#include <Eigen/QR>

using namespace mppi_ga;

ROV::ROV(bool use_wrench_input)
{
  if(use_wrench_input)
  {
    TAM.setIdentity(6,6);
    setMaxCommand({40,40,40,2,10,30});
  }
  else
  {
    setMaxCommand(40);
    TAM <<  0.70710678, 0.70710678, 0.70710678, 0.70710678, 0.        , 0.,
        0.70710678, -0.70710678, -0.70710678, 0.70710678, 0.        , 0.,
        0.        , 0.        , 0.        , 0.    ,      1.    ,     -1.,
        0.05126524, -0.05126524, -0.05126524, 0.05126524, -0.1105   , -0.1105,
        -0.05126524, -0.05126524, -0.05126524, -0.05126524, -0.0025    , 0.0025,
        0.16652365, -0.16652365, 0.17500893, -0.17500893, 0.        , 0.;
  }

  Minv << 14.8, 0, 0, 0, 0, 0,
      0, 14.8, 0, 0, 0, 0,
      0, 0,14.8,  0, 0, 0,
      0, 0, 0, 5.25, 0.01, 0.33,
      0, 0, 0, 0.01, 7.94, 0.026,
      0, 0, 0, 0.33, 0.026, 6.91;
  Minv = Minv.completeOrthogonalDecomposition().pseudoInverse();

  dl << 1.31, 9.14, 2.015, 10., 10., 0.;
  dq << 33.8, 54.26875, 73.37135, 40.0, 40.0, 40.0;
}

SE3twist ROV::xNext(const SE3twist &x, const Vec &u, Float dt)
{
  auto xNext = x;

  //Eigen::AngleAxis<Float> o(x.pose.q.inverse());

  // apply wrench from props and drag
  Vec wrench{TAM * u - dl.cwiseProduct(x.twist)};
  for(size_t i = 0; i < 6; ++i)
    wrench(i) -= dq(i)*x.twist(i)*std::abs(x.twist(i));
  xNext.update(Minv*wrench, dt);
  return xNext;
}
/*
SE3twist ROV::xNext(const SE3twist &x, const Vec &u, Float dt)
{
  auto xNext = x;

  // apply wrench from props and drag
  Vec wrench{TAM * u - dl.cwiseProduct(x.twist)};
  for(size_t i = 0; i < 6; ++i)
    wrench(i) -= dq(i)*x.twist(i)*std::abs(x.twist(i));
  xNext.update(Minv*wrench, dt);
  return xNext;
}
*/

// define ref @ time t
void ROV::reference(Float t, SE3twist &xr) const
{
  // polar rose trajectory
  xr.pose.p(0) = amp * cos(petals*w*t)*cos(w*t);
  xr.pose.p(1) = amp * cos(petals*w*t)*sin(w*t);
  xr.pose.p(2) = Zamp * sin(freq*t);
  //xr.pose.p.setZero();
}

// define vectorized error between two states
Eigen::Vector<Float,3> ROV::error(const SE3twist &x, const SE3twist &xr) const
{
  return x.pose.p - xr.pose.p; // position error only, no orientation error
}
