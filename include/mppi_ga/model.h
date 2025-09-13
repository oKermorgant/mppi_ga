#ifndef PARAMPC_PROBLEM_H
#define PARAMPC_PROBLEM_H

#include <Eigen/Core>
#include <mppi_ga/types.h>

namespace mppi_ga
{

// to be inherited to define a particular problem
template <class State, Index uDim, Index eDim>
struct Model
{
  Eigen::Vector<Float, uDim> uLim;

  // init with dimension
  Model()
  {
    assert(eDim>0 && uDim>0);
    setMaxCommand(std::numeric_limits<Float>::max());
  }

  /// constraint on max command
  void setMaxCommand(const std::vector<Float> &u)
  {
    if(u.size() == uDim)
      std::copy(u.begin(), u.end(), uLim.data());
  }
  void setMaxCommand(Float u)
  {
    uLim.setConstant(u);
  }    

  // built-in function to get state evolution
  virtual State xNext(const State &x, const Vec &u, Float dt) = 0;
  // define ref @ time t
  virtual void reference(Float t, State &xr) const = 0;
  // define vectorized error between two states
  virtual Eigen::Vector<Float,eDim> error(const State &x, const State &xr) const = 0;
  // any additional constraint
  virtual Float constraintCost(const State &x) const
  {
    return 0.;
  }
};

}

#endif // PARAMPC_PROBLEM_H
