#include <mppi_ga/mppi.h>
#include <mppi_ga/time.h>

using namespace mppi_ga;

void MPCParams::configureHorizon(Index uDim, ControlHorizon c, PredictionHorizon p, Float dt, Subsampling s)
{
  if(c.h == 0 || p.h == 0 || dt <= 0 || s.sub == 0)
    throw(std::invalid_argument("MPC::configureHorizon: horizons and sampling time should be stricly positive"));

  if((c.h-1) % s.sub || c.h < s.sub)
  {
    std::cerr << "MPC::configureHorizon: subsampling (" << s.sub << ") should be a divisor of control horizon-1 ("
              << c.h-1 << "), increasing control horizon to ";
    c.h = std::max<Index>(s.sub, s.sub*std::ceil((c.h-1.)/s.sub))+1;
    std::cerr << c.h << std::endl;
  }

  if(c.h > p.h)
  {
    std::cerr << "MPC::configureHorizon: control horizon (" << c.h << ") cannot be greater than prediction (" << p.h
              << "), increasing prediction horizon to " << c.h << std::endl;
    p.h = c.h;
  }

  const auto update = [&](auto& cur, auto arg)
  {
    if(cur == arg)
      return;
    cur = arg;
    status = ParamStatus::CHANGE;
  };

  update(control, c.h);
  update(prediction, p.h);
  update(subsampling, s.sub);
  subsampled = uDim*(subsampling-1)*(control/subsampling);

  // changing sampling time does not change the size
  this->dt = dt;
  dt_inv = 1./dt;
}

void MPCParams::configureCost(Index eDim, Index uDim, Float xDecay, Float uDecay, std::vector<Float> Q, std::vector<Float> R)
{
  resizeNullify(this->Q, eDim);

  if(Q.size())
  {
    // forget non-costly states
  for(auto idx = 0; idx < Q.size(); ++ idx)
    this->Q(idx) = Q[idx];
  }
  else
  {
    this->Q.setOnes();
  }

  this->R.resize(uDim);
  if(R.size() == uDim)
    std::copy(R.begin(), R.end(), this->R.data());
  else
    this->R.setOnes();

  if(xDecay < 0 || xDecay > 1)
    std::cerr << "xDecay is not in [0,1]" << std::endl;
  if(uDecay < 0 || uDecay > 1)
    std::cerr << "uDecay is not in [0,1]" << std::endl;
  this->xDecay = xDecay;
  this->uDecay = uDecay;
  status = ParamStatus::CHANGE;
}
