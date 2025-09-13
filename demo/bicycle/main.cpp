#include <mppi_ga/ga.h>
#include <mppi_ga/model.h>
#include <demo.h>
#include <log2plot/logger.h>
#include <numeric>

#include "bicycle.h"
#include <log2plot/config_manager.h>

using namespace mppi_ga;

using Clock = std::chrono::high_resolution_clock;

int main(int argc, char**argv)
{
  log2plot::closePreviousPlots();

  log2plot::ConfigManager config(std::string(MPPI_GA_RESULT_DIR) + "/_config.yaml", argc, argv);
  config.updateFrom(argc, argv, true);

  Bicycle robot{config.read<bool>("cons")};

  MPCGA mpc(robot);
  const auto dt{0.05};

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
  // 9 11
  mpc.configureHorizon(ControlHorizon(config.read<int>("cont")),
                       PredictionHorizon(config.read<int>("hor")),
                       0.05, Subsampling(sub));
  mpc.configureCost(.9, .9, {1000,1000}, {.01,0.01}); // theta and beta do not count in the cost

  mpc.showDimension();

  BicycleState xr;
  Vec u;  resizeNullify(u,2);

  auto t{0.};

  if(mpc.params.multi)
    mpc.setMaxSolvingTime(dt*1e9*0.95);
  robot.reference(0, xr);

  // starting position
  BicycleState cur;
  cur.pose = {1.05*xs,
              -0.1,
              M_PI/2};

  const auto base_path{result_path("bicycle",config)};

  log2plot::Logger logger(base_path);
  logger.setTime(t);
  Vec ctime(1), rollout(1), cost(1), xy2(4), error(1);
  //logger.regroupNext(2);
  //logger.saveTimed(xy, "xy_err", "[x-x^*,y-y^*]", "position error [m]");
  //logger.setLineType("[C0,C1]");
  //logger.saveTimed(theta, "theta_err", "[\\theta-\\theta^*]", "orientation error [rad]");
  //logger.setLineType("[C2]");
  //logger.saveTimed(u, "cmd", "[v, \\omega]", "command");
  logger.saveXY(xy2, "xy", "[Reference, Actual]", "x [m]", "y [m]");
  logger.setLineType("[C0--,C1-]");
  logger.setPlotArgs("--equal");
  //logger.showFixedShape(log2plot::Shape({{cur(0), cur(1)},{xr(0), xr(1)}}, "rD"));
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
  while(t < 1.3*tf)
  {
    // find next control input

	const auto start{Clock::now()};
	u = .5*(u+mpc.solve(cur, t));

	//std::cout << u.transpose() << std::endl;
	ctime[0] = std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - start).count()/1000.;
	robot.reference(t, xr);	

	xy2[2] = cur.pose.x;
	xy2[3] = cur.pose.y;
	xy2[0] = xr.pose.x;
	xy2[1] = xr.pose.y;

	rollout[0] = mpc.stats.rollouts;
	cost[0] = mpc.stats.cost;
	error[0] = robot.error(cur, xr).norm();

	if(!first)
	{
	  logger.update();
	  times.push_back(ctime[0]);

	  // apply / update x0
	  cur = robot.xNext(cur,u,dt);
	  t += dt;
	}
	first = false;
  }

  std::cout << "Avg. time: " << std::accumulate(times.begin(), times.end(), 0)/times.size()
            << " ms\n";

  if(config.read<bool>("plot"))
    logger.plot();
}
