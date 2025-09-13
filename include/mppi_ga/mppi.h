#ifndef PARAMPC_MPC_H
#define PARAMPC_MPC_H

#include <mppi_ga/model.h>
#include <chrono>

namespace mppi_ga
{

using Nano = std::chrono::nanoseconds;

// strong types
struct ControlHorizon
{
  Index h{};
  explicit ControlHorizon(Index h) : h{h} {}
};
struct PredictionHorizon
{
  Index h{};
  explicit PredictionHorizon(Index h) : h{h} {}
};
struct Subsampling
{
  Index sub{};
  explicit Subsampling(Index s) : sub{s} {}
};

enum struct ParamStatus{CHANGE, PARSED, DONE};

struct MPCParams
{  
  ParamStatus status = ParamStatus::CHANGE;
  Index control{1}, prediction{2}, subsampling{1}, subsampled{0};
  Float dt{0.1}, dt_inv{10.};
  bool use_ff{false};
  bool use_zoh{false};
  bool multi{false};

  // cost function
  Vec Q;  // x-weights
  Vec R;  // u-weights
  Float xDecay{1}, uDecay{1};

  void configureCost(Index eDim, Index uDim, Float xDecay, Float uDecay, std::vector<Float> Q, std::vector<Float> R);
  void configureHorizon(Index uDim, ControlHorizon c, PredictionHorizon p, Float dt, Subsampling s=Subsampling{1});
  inline bool changed() const {return status == ParamStatus::CHANGE;}
  inline bool parsed() const {return status == ParamStatus::PARSED;}
};

template <class State, Index uDim, Index eDim>
class MPPI
{
protected:
  using Clock = std::chrono::steady_clock;
  using uVec = Eigen::Vector<Float,uDim>;
public:

  virtual inline std::string describe() const
  {
    auto ret{std::to_string(params.prediction) + "_" + std::to_string(params.control)};

	if(params.subsampling > 1)
	{
	  ret += "_" + std::to_string(params.subsampling);
	  if(params.use_zoh)
		ret += "zoh";
	}
	return ret;
  }

  inline void setMaxSolvingTime(uint tmax_ns)
  {
    tmax = Nano(tmax_ns);
  }

  inline void startTiming()
  {
    start = Clock::now();
  }

  inline auto enoughTime()
  {
    return (Clock::now()-start) < tmax;
  }

  // solve for current state and single future reference
  inline Vec solve(const State &x0, const State &xr)
  {
    this->xr = std::vector<State>(params.prediction, xr);
    return solve(x0);
  }

  // solve for current state at given time
  uVec solve(const State &x0, Float t0)
  {
    if(model == nullptr)
    {
      throw(std::logic_error("MPC::solve(Vec) can only be used when initialized with an MPC problem"
                             ", use solve(Vec, vector<Vec>) to provide the future state references"));
    }

	if(xr.size() != params.prediction)
	  xr.resize(params.prediction);

	for(auto hor = 0; hor < params.prediction; ++hor)
	  model->reference(t0+(hor+1)*params.dt, xr[hor]);

    return solve(x0);
  }

  uVec solve(const State &x0)
  {
    //ScopedTimer("solve");
    if(params.changed())
    {
      parseParams();
      params.status = ParamStatus::PARSED;
      //imc_e = Vec::Zero(xDim);
    }

	// adapt to change
	/*if(params.use_ff)
  {
    if(imc.u.size() == uDim)
    {
      // forward error on reference
      const auto in{imc.in()};
      auto out{imc.out()};
      if(problem)
        problem->prepare(in);
      model(in, out);
      imc.x += out.value * params.dt;
      imc.e = x0 - imc.x;
    }
    else
    {
      imc.x = x0;
      imc.JuDummy.resize(xDim, uDim);
      imc.xdot.resize(xDim);
    }
  }*/
    u_prev = solve_impl(x0, xr);
    params.status = ParamStatus::DONE;
    return u_prev.template head<uDim>();
  }

  /// problem is given externally
  explicit MPPI(Model<State, uDim, eDim> &model) : model{&model}
  {}


  /// Dimensional parameters of MPC
  inline void configureHorizon(ControlHorizon c, PredictionHorizon p, Float dt, Subsampling s=Subsampling{1})
  {
    params.configureHorizon(uDim, c, p, dt, s);
  }

  /// Meta parameters of cost function
  inline void configureCost(Float xDecay, Float uDecay, std::vector<Float> Q = {}, std::vector<Float> R={})
  {
    params.configureCost(eDim, uDim, xDecay, uDecay, Q, R);
  }

  /// Display dimension of underlying problem
  virtual void showDimension() {}

  //protected:

  virtual Vec solve_impl(const State &x0, const std::vector<State> &xr) = 0;

  // timing
  Clock::time_point start;
  Nano tmax{0};

  // MPC params
  MPCParams params;

  // external model
  Model<State, uDim, eDim> * model = nullptr;
  std::vector<State> xr;

  virtual void parseParams() = 0;
  template <typename data>
  inline static void applyOffset(data* start, Index length, Index offset)
  {
    std::copy(start+offset, start+length, start);
  }

  // cache + robustness
  /*struct IMC
  {
    Vec x, u;
  };*/

  Vec u_prev;
  //IMC imc;
  Vec imc_e;
};



}

#endif // PARAMPC_MPC_H
