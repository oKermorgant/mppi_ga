#ifndef MPPI_GA_ROV
#define MPPI_GA_ROV


#include <mppi_ga/types.h>
#include <mppi_ga/model.h>
#include <mppi_ga/states.h>

using namespace mppi_ga;

// ref trajectory
constexpr Float vx{.3};
constexpr Float yAmp{30};
constexpr Float xAdv{7};
constexpr auto tf{4*(yAmp+xAdv)/vx + xAdv/(2*vx)};

constexpr auto w{.01};
constexpr auto xs{10.5};
constexpr auto ys{9.};
constexpr auto ymax{10.};
//constexpr auto tf{2*M_PI/w};

struct BoatState
{
  Float tl{}, tr{};
  SE2twist x;
};


struct Boat : public mppi_ga::Model<BoatState, 4, 2>
{

  Mat Minv{3,3};
  Vec dl{3};
  Vec dq{3};

  Boat();

  BoatState xNext(const BoatState &x, const Vec &u, Float dt) override;
  BoatState xNextWrench(const BoatState &x, Vec wrench, Float dt);

  // define ref @ time t
  virtual void reference(Float t, BoatState &xr) const override;

  // define vectorized error between two states
  virtual Eigen::Vector<Float,2> error(const BoatState &x, const BoatState &xr) const override;


};


#endif
