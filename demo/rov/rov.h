#ifndef MPPI_GA_ROV
#define MPPI_GA_ROV


#include <mppi_ga/types.h>
#include <mppi_ga/model.h>
#include <mppi_ga/states.h>

using namespace mppi_ga;

// TODO ref trajectory
constexpr auto petals{0};
constexpr Float amp{2};
constexpr Float freq{0.02};
constexpr Float Zamp{0};
constexpr Float w{0.1};
constexpr auto tf{(2-(petals % 2))*M_PI/w};

struct ROV : public mppi_ga::Model<SE3twist, 6, 3>
{

  Mat Minv{6,6};
  Vec dl{6};
  Vec dq{6};
  Mat TAM{6,6};

  ROV(bool use_wrench_input = false);

  SE3twist xNext(const SE3twist &x, const Vec &u, Float dt) override;
  SE3twist xNextWrench(const SE3twist &x, Vec wrench, Float dt);

  // define ref @ time t
  virtual void reference(Float t, SE3twist &xr) const override;

  // define vectorized error between two states
  virtual Eigen::Vector<Float,3> error(const SE3twist &x, const SE3twist &xr) const override;


};


#endif
