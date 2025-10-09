#ifndef MPPI_GA_BICYCLE
#define MPPI_GA_BICYCLE

#include <mppi_ga/types.h>
#include <mppi_ga/model.h>
#include <mppi_ga/states.h>

using namespace mppi_ga;

// reference
constexpr auto w{.4};
constexpr auto xs{10.5};
constexpr auto ys{9.};
constexpr auto ymax{10.};
constexpr auto tf{2*M_PI/w};
constexpr auto beta_max{M_PI/3};


// model
constexpr auto L{1.5};

struct BicycleState
{
  SE2 pose;
  Float beta{};
};

struct Bicycle : public mppi_ga::Model<BicycleState, 2, 2>
{
  bool constraint{false};
  Bicycle(bool constraint) : constraint(constraint)
  {
    setMaxCommand({10., beta_max*3});
  }

  BicycleState xNext(const BicycleState &x, const Vec &u, Float dt) override
  {
    auto xnext = x;
    xnext.pose.update(u(0)*cos(x.beta), 0, u(0)*sin(x.beta)/L, dt);
    xnext.beta = std::clamp<Float>(xnext.beta + u(1)*dt, -beta_max, beta_max);
    return xnext;
  }

  // define ref @ time t
  virtual void reference(Float t, BicycleState &xr) const override
  {
    const auto cr{cos(w*t)};
    const auto sr{sin(w*t)};
    xr.pose.x = xs*cr;
    xr.pose.y = ys*sr*cr;

	/*xr(4) = atan2(w*(a*cr - 2*b*sr*sr + b), w*(a + 2*b*cr)*sr);
	xr(2) = cos(xr(4)/2);
	xr(3) = sin(xr(4)/2);
	xr(4) = 0.; // beta*/
  }

  // define vectorized error between two states
  virtual Eigen::Vector<Float,2> error(const BicycleState &x, const BicycleState &xr) const override
  {
    return {x.pose.x-xr.pose.x, x.pose.y-xr.pose.y};
  }

  virtual Float constraintCost(const BicycleState &x) const override
  {
    if(!constraint)
      return 0.;
    if(x.pose.y < ymax*0.99)
      return 0;
    else if(x.pose.y >= ymax)
      return 10000;
    return -log(ymax-x.pose.y);
  }
};

#endif // MPPI_GA_BICYCLE
