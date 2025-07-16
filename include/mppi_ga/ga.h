#ifndef PARAMPC_MPCGA_H
#define PARAMPC_MPCGA_H

#include <mppi_ga/mppi.h>
#include <mppi_ga/model.h>
#include <iostream>
#include <random>
#include <execution>

// 0 = no parallel, 1 = parallel_for, 2 = threadpool
#define MPPI_NOPAR 0
#define MPPI_STDPAR 1

#define MPPI_PAR 1

#define RAND_LOCAL

namespace mppi_ga
{

struct Stats
{
  size_t rollouts;
  Float cost;
};


struct Candidate
{
  inline static auto unif{std::uniform_real_distribution<mppi_ga::Float>(0.,1.)};
  Vec u;  // subsampled
  Float cost;

  inline static auto& engine()
  {
#ifdef RAND_LOCAL
    static thread_local auto engine = []()
{
  std::default_random_engine eng;
  eng.seed(std::random_device()() + std::hash<std::thread::id>()(std::this_thread::get_id()));
  return eng;
}();
#else
	static std::mutex mtx;
	static std::default_random_engine engine;
	auto lock{std::scoped_lock(mtx)};
#endif
    return engine;
  }

  inline void randomize(const Vec &uLim = {})
  {
    auto rand_range = std::uniform_real_distribution(0., 1.);
    for(size_t i = 0; i < u.size(); ++i)
      u(i) = -uLim[i] + 2*uLim[i]*rand_range(engine());
  }

  inline void crossAndMutate(const Candidate &p1, const Candidate &p2, const Vec &uLim)
  {
    const auto alpha{unif(engine())};
    for(size_t i = 0; i < u.size(); ++i)
    {
      const auto change{.1 *(-1 + 2*unif(engine()))*2*uLim[i]}; // change is within 10 % of range
      u(i) = std::clamp<Float>(alpha*p1.u(i) + (1-alpha)*p2.u(i) + change, -uLim[i], uLim[i]);
    }
  }
  inline bool operator<(const Candidate &other) const
  {
    return cost < other.cost;
  }
};


std::pair<size_t, size_t> differentRandomNumbers(size_t min, size_t max)
{

  const auto n1{std::uniform_int_distribution(min, max)(Candidate::engine())};
  const auto n2{std::uniform_int_distribution(min, max-1)(Candidate::engine())};
  if(n1 == n2)
    return {n1,n2+1};
  return {n1, n2};
}


template <class State, Index uDim, Index eDim>
class MPCGA : public MPPI<State, uDim, eDim>
{
public:
  using MPPI<State, uDim, eDim>::params;
  using MPPI<State, uDim, eDim>::model;
  /// problem is given externally
  explicit MPCGA(Model<State, uDim, eDim> &description) : MPPI<State, uDim, eDim>(description)
  {
  }

  /// Display dimension of underlying GA
  void showDimension() override
  {
    const auto &params{this->params};
    if(params.changed())
      parseParams();

	const auto actualU{params.control-(params.subsampling-1)*(params.control/params.subsampling)};
	std::cout << "control:        " << uDim << "x" << params.control << " -> " << uDim*params.control << '\n'
			  << "actual unknown: " << uDim << "x" << actualU << " -> " << uDim*actualU << '\n';
	std::cout << "ga: " << pop_size << " candidates for " << keep << " elitism" << std::endl;
  }



  Candidate solve_bf(const State &x0, const std::vector<State> &xr)
  {
    constexpr auto lambda{0.000001};
    static std::vector<Float> weights(pop_size);

	// compute and scale weights
	double sum{};
	for(size_t idx = 0; idx < pop_size; idx++)
	{
	  weights[idx] = exp(-lambda*population[idx].cost);
	  sum += weights[idx];
	}

	//const auto sum{std::accumulate(weights.begin(), weights.end(), 0)};
	const auto scale{1./sum};
	auto &best{population.front()};
	best.u *= weights[0];
	for(size_t idx = 1; idx < pop_size; idx++)
	  best.u += weights[idx]*scale * population[idx].u.head(uDim);
	computeCost(best, x0, xr);
	return best;
  }

  Candidate solve_ga(const State &x0, const std::vector<State> &xr)
  {
    this->startTiming();
    auto best{solve_single(x0, xr)};

	if(params.multi)
	{
	  while(this->enoughTime())
	  {
		std::for_each(__pstl::execution::par,
					  population.begin(), population.end(),
					  [&](auto &cand){
						cand.randomize(uLim);
						computeCost(cand, x0, xr);}
					  );
		stats.rollouts += pop_size;
		best = std::min(best, solve_single(x0, xr));
	  }
	}

    return best;
  }

  /// solve for current state and future references starting at t0, when using external description
  /// dt is the control sampling time
  Vec solve_impl(const State &x0, const std::vector<State> &xr) override
  {
    //ScopedTimer timer("MPCGA::solve");

	size_t start{};
	if(this->imc.u.size())
	{
	  // get previous solution and re-evaluate
	  auto &best = population.front() = *std::min_element(population.begin(), population.begin()+keep);
	  // move 1
	  const auto u = U * best.u;
	  best.u.segment(0, uDim) = u.segment(uDim, uDim);
	  start = 1;
	  computeCost(population.front(), x0, xr);
	}

#if MPPI_PAR == MPPI_NOPAR
	std::for_each(population.begin()+start, population.end(),
				  [&](auto &cand){
					cand.randomize(uLim);
					computeCost(cand, x0, xr);}
				  );
#elif MPPI_PAR == MPPI_STDPAR
	std::for_each(std::execution::par,
				  population.begin()+start, population.end(),
				  [&](auto &cand){
					cand.randomize(uLim);
					computeCost(cand, x0, xr);}
				  );

#endif

	stats.rollouts = pop_size;
	auto best = keep ? solve_ga(x0, xr) : solve_bf(x0, xr);
	stats.cost = best.cost;
	return best.u.head(uDim);
  }

  /// Meta parameters of cost function
  inline void configureGA(size_t population, size_t elitism)
  {
    pop_size = population;
    keep = elitism;
  }

  /// Maximum iterations of the GA
  inline void setMaxIter(size_t iter, size_t iter_same) {max_iter = iter; max_same = iter_same;}

  //protected:


  Candidate solve_single(const State &x0, const std::vector<State> &xr)
  {
    const auto reorder = [&]()
    {
      std::nth_element(population.begin(), population.begin()+keep, population.end());
      return std::min_element(population.begin(), population.begin()+keep);
    };

	auto best = reorder();
	auto best_cost = best->cost;
	size_t iter{}, iter_seq{};
	while(iter < max_iter && iter_seq < max_same && (!params.multi || this->enoughTime()))
	{
	  std::for_each(__pstl::execution::par,
					winners.begin(), winners.end(),
					[&](auto &winner){
					  const auto [n1,n2] = differentRandomNumbers(keep, pop_size-1); {}
					  winner = std::min(population[n1], population[n2]);
					});

	  std::copy(winners.begin(), winners.end(),
				population.begin()+keep);

#if MPPI_PAR == MPPI_NOPAR
	  for(auto crossed = population.begin()+pop_size/2; crossed != population.end(); ++crossed)
	  {
		const auto [n1,n2] = differentRandomNumbers(0, pop_size/2-1); {}
		crossed->crossAndMutate(population[n1], population[n2], uLim);
		computeCost(*crossed, x0, xr);
	  }
#else
	  std::for_each(std::execution::par,
					population.begin()+pop_size/2, population.end(),
					[&](auto &cand)
					{
					  const auto [n1,n2] = differentRandomNumbers(0, pop_size/2-1); {}
					  cand.crossAndMutate(population[n1], population[n2], uLim);
					  computeCost(cand, x0, xr);
					});
#endif

	  stats.rollouts += pop_size/2;

	  best = reorder();

	  if(const auto new_cost{best->cost}; new_cost < best_cost)
	  {
		best_cost = new_cost;
		iter_seq = 0;
	  }
	  else
	  {
		iter_seq++;
	  }
	  iter++;
	}
	//computeCost(*best, x0, xr, t0, true);
	//std::cout << "MPCGA: best cost: " << best_cost << " after " << iter << " iterations" << std::endl;
	return *best;
  }

  // MPC params
  void parseParams() override
  {
    Candidate::engine().seed(std::random_device()());
    // new size has not been taken into account
    const auto actualU{params.control*uDim-params.subsampled};
    resizeNullify(U, params.prediction*uDim, actualU);

	uLim.resize(actualU);
	for(size_t hor = 0; hor < uLim.size()/model->uLim.size(); ++hor)
	  uLim.segment(hor*uDim, uDim) = model->uLim;

	if(params.subsampled)
	{
	  // fill U  u_h = h/s*u_0 + (1-h)/s*u_f
	  const auto rows{(params.subsampling+1)*uDim};
	  const auto cols{2*uDim};
	  Mat Sub{Mat::Zero(rows, cols)};

	  if(params.use_zoh)
	  {
		for(size_t hor = 0; hor < params.subsampling; ++hor)
		  Sub.block(uDim*hor, 0, uDim, uDim).diagonal().setConstant(1);
		Sub.block(uDim*params.subsampling, uDim, uDim, uDim).diagonal().setConstant(1);
	  }
	  else
	  {
		for(size_t hor = 0; hor < params.subsampling+1; ++hor)
		{
		  const auto h = static_cast<Float>(params.subsampling-hor)/params.subsampling;
		  Sub.block(uDim*hor, 0, uDim, uDim).diagonal().setConstant(h);
		  Sub.block(uDim*hor, uDim, uDim, uDim).diagonal().setConstant(1.-h);
		}
	  }

	  // write it for all dependant controls
	  for(size_t part = 0; part < params.control/params.subsampling; ++part)
		U.block((rows-uDim)*part, uDim*part, rows, cols) = Sub;
	  //std::cout << U << std::endl;
	}
	else
	{
	  U.block(0,0, params.control,params.control).setIdentity();
	}
	// control vs prediction horizons
	for(size_t part = params.control; part < params.prediction; ++part)
	  U.block(part*uDim, actualU-uDim, uDim, uDim).setIdentity();

	// init GA
	population.resize(pop_size);
	if(keep < pop_size/2)
	  winners.resize(pop_size/2-keep);
	for(auto &candidate: population)
	  candidate.u.resize((actualU));

	Q.resize(params.prediction, params.Q);
	for(size_t x = 1; x < params.prediction; ++x)
	  Q[x] = Q[x-1]*params.xDecay;

	R.resize(params.prediction, params.R);
	for(size_t r = 1; r < params.prediction; ++r)
	  R[r] = params.uDecay * R[r-1];
  }

  Vec uLim;
  std::vector<Vec> Q, R;

  // GA
  Stats stats;
  size_t pop_size{100}, keep{15};
  size_t max_iter{100}, max_same{30};
  std::vector<Candidate> population, winners;

  void computeCost(Candidate& cand, const State &x0, const std::vector<State> &xr, bool display=false)
  {
	// compute full control sequence
	const auto Ufull{U*cand.u}; // = params.subsampled ? U*cand.u : cand.u;

	cand.cost = 0;

	//size_t uIdx{0};
	auto x = x0;
	Vec u;
	auto fromX{cand.cost};
	for(size_t hor = 0; hor < params.prediction; ++hor)
	{
	  const auto u{Ufull.segment(hor*uDim, uDim)};

	  x = model->xNext(x, u, params.dt);
	  const auto err{model->error(x,xr[hor])};
	  cand.cost += err.cwiseProduct(Q[hor]).transpose() * err;
	  cand.cost += model->constraintCost(x);
	  if(display)
		fromX += err.cwiseProduct(Q[hor]).transpose() * err;

	  cand.cost += (u.cwiseProduct(R[hor]).transpose() * u);
	}

	if(display)
	  std::cout << " cost: " << cand.cost << " = " << fromX << " (x) + " << cand.cost-fromX << " (u)" << std::endl;

  }

  // cache
  mutable Mat U;  // map subsampled to full u
};

}

#endif // PARAMPC_MPCGA_H
