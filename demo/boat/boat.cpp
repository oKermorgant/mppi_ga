

#include <mppi_ga/types.h>
#include "boat.h"
#include <Eigen/QR>

//#define USE_ANGLE

using namespace mppi_ga;

constexpr Float Tmax{M_PI/4};
constexpr Float Tx{-3};
constexpr Float Ty{0.6};
constexpr Float fMax{1000};

Boat::Boat()
{
#ifdef USE_ANGLE
  setMaxCommand({fMax,fMax,Tmax, Tmax});
#else
  setMaxCommand({fMax,fMax,Tmax/3, Tmax/3});
#endif

  Minv << 1000, 0, 0,
      0, 1000, 0,
      0, 0, 200;
  Minv = Minv.completeOrthogonalDecomposition().pseudoInverse();

  dl << 182, 183, 200;
  dq << 224, 149, 200;

  // get vmax
  const auto D{dl(0)*dl(0) + 8*(dq(0)*fMax)};
  std::cout << "vmax = " << (-dl(0) + sqrt(D))/(2*dl(0)) << "\n";
}

BoatState Boat::xNext(const BoatState &x, const Vec &u, Float dt)
{
  auto xNext = x;

// update thruster angles
#ifdef USE_ANGLE
  xNext.tl = half*(x.tl+u(2));
  xNext.tr = half*(x.tr+u(3));
#else
  xNext.tl = std::clamp(x.tl+u(2)*dt*half, -Tmax, Tmax);
  xNext.tr = std::clamp(x.tr+u(3)*dt*half, -Tmax, Tmax);
#endif

  // get force
  const auto fl{u(0)};
  const auto fr{u(1)};

  Eigen::Vector<Float, 3> wrench;
  const auto cr{cos(xNext.tr)};
  const auto sr{sin(xNext.tr)};
  const auto cl{cos(xNext.tl)};
  const auto sl{sin(xNext.tl)};

  const auto& v{x.x.twist};

  wrench(0) = fl*cl + fr*cr;
  wrench(1) = fl*sl + fr*sr;
  wrench(2) = fl*(Tx*sl - Ty*cl) + fr*(Tx*sr + Ty*cr);
  wrench -= dl.cwiseProduct(v);
  for(size_t i = 0; i < 3; ++i)
    wrench(i) -= dq(i)*v(i)*std::abs(v(i));

/*  std::cout << "v prev: " << x.x.twist.transpose() << "\n";
  std::cout << "wrench: " << wrench.transpose() << "\n";
  std::cout << "accel: " << (Minv*wrench).transpose() << "\n";
*/
  xNext.x.update(Minv*wrench, dt);

  //std::cout << "v next: " << x.x.twist.transpose() << "\n";

  // update thruster angles
#ifdef USE_ANGLE
  xNext.tl = u(2);
  xNext.tr = u(3);
#else
  xNext.tl = std::clamp(x.tl+u(2)*dt*half, -Tmax, Tmax);
  xNext.tr = std::clamp(x.tr+u(3)*dt*half, -Tmax, Tmax);
#endif

  return xNext;
}

// define ref @ time t
void Boat::reference(Float t, BoatState &xr) const
{
  /*const auto cr{cos(w*t)};
  const auto sr{sin(w*t)};
  xr.x.pose.x = xs*cr;
  xr.x.pose.y = ys*sr*cr;

  return;*/


  constexpr static auto tFull{(xAdv+yAmp)/vx};
  constexpr static auto tx{xAdv/vx};
  const uint done = t/tFull;
  const auto part{t - done*tFull};

  xr.x.pose.x = done * xAdv;
  xr.x.pose.y = yAmp/2;
  if(part < tx)
  {
    // horizontal part
    xr.x.pose.x += vx*part;
  }
  else
  {
    xr.x.pose.x += xAdv;
    // vertical part
    xr.x.pose.y -= vx*(part - tx);
  }

  if(done % 2)
    xr.x.pose.y *= -1;

}

// define vectorized error between two states
Eigen::Vector<Float,2> Boat::error(const BoatState &x, const BoatState &xr) const
{
  return {x.x.pose.x - xr.x.pose.x,
          x.x.pose.y - xr.x.pose.y};
}
