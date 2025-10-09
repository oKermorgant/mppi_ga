#include <mppi_ga/ga.h>
#include <mppi_ga/model.h>
#include <demo.h>
#include <log2plot/logger.h>
#include <numeric>

#include "boat.h"
#include <log2plot/config_manager.h>


using namespace mppi_ga;

using Clock = std::chrono::high_resolution_clock;

int main(int argc, char**argv)
{
  log2plot::closePreviousPlots();
  Boat robot;
  const auto dt{.5};

  MPCGA mpc(robot);

  log2plot::ConfigManager config(std::string(MPPI_GA_RESULT_DIR) + "/_config.yaml", argc, argv);

  const auto sub{config.read<int>("sub")};
  mpc.params.use_zoh = config.read<bool>("use_zoh");
  const auto pop{config.read<int>("pop")};
  mpc.params.multi = config.read<bool>("multi");
  const auto iter{100};

  if(config.read<bool>("bf"))
    mpc.configureGA(pop + iter*pop/2, 0);
  else
    mpc.configureGA(pop, pop/10);

  mpc.setMaxIter(iter, iter/2);
  mpc.configureHorizon(ControlHorizon(9), PredictionHorizon(11), dt, Subsampling(sub));
  mpc.configureCost(.8, .8,
                    {10000,10000},
                    std::vector<Float>(4, 1.));

  mpc.showDimension();

  BoatState xr, cur;
  decltype (robot.uLim) u = 0*robot.uLim;
  // starting position
  cur.x.pose = {-0.1,
                yAmp/2,
                0};

  auto t{0.};
  if(mpc.params.multi)
    mpc.setMaxSolvingTime(dt*1e9*0.95);
  robot.reference(0, xr);

  const auto base_path{result_path("boat",config)};

  log2plot::Logger logger(base_path);
  logger.setTime(t);
  Vec ctime(1), rollout(1), cost(1), error(1);
  Vec xy(4);


  logger.saveTimed(u, "u", "[f_l,f_r,\\theta_l,\\theta_r]", "command");
  logger.setPlotArgs("--markEvery 0");

  //logger.regroupNext(2);
  //logger.save3Dpose(cur.pose.p, "pose", "pose");
  //logger.showMovingShape(log2plot::Box(-xl/2,-yl/2,-zl/2, xl/2, yl/2, zl/2, "C0", "pose"));
  //logger.save3Dpose(xr.pose.p, "ref", "ref");
  logger.saveXY(xy, "xy", "[Reference, Actual]", "x [m]", "y [m]");
  logger.setLineType("[C0--,C1-]");
  //logger.setPlotArgs("--equal");
  //logger.setPlotArgs("--equal");
  //logger.saveTimed(theta, "theta_err", "[\\theta-\\theta^*]", "orientation error [rad]");
  //logger.setLineType("[C2]");
  //logger.saveTimed(u, "cmd", "[v, \\omega]", "command");
  /*logger.saveXY(xy2, "xy", "[Actual, Reference]", "x [m]", "y [m]");
  logger.setLineType("[C0d-,C1-]");
  logger.setPlotArgs("--equal");
  logger.showFixedShape(log2plot::Shape({{cur(0), cur(1)},{xr(0), xr(1)}}, "rD"));*/
  logger.saveTimed(ctime, "ctime", "[t_c]", "comp. time [ms]");
  logger.setPlotArgs("--legendLoc none --markEvery 0");
  logger.setLineType("[C0.]");
  logger.saveTimed(rollout, "rollout", "[]", "Rollouts");
  logger.setLineType("[C1.]");
  logger.setPlotArgs("--legendLoc none --markEvery 0");
  logger.saveTimed(cost, "cost", "[]", "Cost function");
  logger.setLineType("[C2.]");
  logger.setPlotArgs("--legendLoc none --markEvery 0");
  logger.saveTimed(error, "error", "[]", "Error [m]");
  logger.setLineType("[C2.]");
  logger.setPlotArgs("--legendLoc none --markEvery 0");


  std::vector<double> times;
  auto first{true};

  std::cout << "tf = " << tf << " -> " << tf/dt << " iterations" << std::endl;

  while(t < tf)
  {
	const auto start{Clock::now()};
	u = mpc.solve(cur, t);
	//u = {200,200,M_PI/4,M_PI/4};
	//u(0) = u(1) = 300;

	//std::cout << u.transpose() << std::endl;
	ctime[0] = std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - start).count()/1000.;
	robot.reference(t, xr);

	rollout[0] = mpc.stats.rollouts;
	cost[0] = mpc.stats.cost;
	error[0] = robot.error(cur, xr).norm();

	xy[2] = cur.x.pose.x;
	xy[3] = cur.x.pose.y;
	xy[0] = xr.x.pose.x;
	xy[1] = xr.x.pose.y;

	if(!first)
	{
	  // apply / update x0
	  cur = robot.xNext(cur,u,dt);

	  u = u.cwiseQuotient(robot.uLim);
	  logger.update();
	  times.push_back(ctime[0]);
	  t += dt;
	}	
	first = false;
  }

  std::cout << "Avg. time: " << std::accumulate(times.begin(), times.end(), 0)/times.size()
            << " ms\n";

  if(config.read<bool>("plot"))
    logger.plot();
}
