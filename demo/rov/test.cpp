#include <mppi_ga/model.h>
#include <demo.h>
#include <log2plot/logger.h>

#include "rov.h"
#include <log2plot/config_manager.h>

using namespace mppi_ga;

using Clock = std::chrono::high_resolution_clock;

int main(int argc, char**argv)
{
  log2plot::closePreviousPlots();
  ROV robot(true);

  log2plot::ConfigManager config(std::string(MPPI_GA_RESULT_DIR) + "/_config.yaml", argc, argv);

  double t{};
  const Float dt{0.1};

  const auto base_path{result_path("rov_test", config)};

  log2plot::Logger logger(base_path);
  logger.setTime(t);
  Vec pose(6);
  logger.save3Dpose(pose, "pose", "pose");

  SE3twist x;
  const auto xl{.5};
  const auto yl{.3};
  const auto zl{.2};
  logger.showMovingShape(log2plot::Box(-xl/2,-yl/2,-zl/2, xl/2, yl/2, zl/2, "C0", "pose"));
  logger.save(x.twist, "v", "[v_x,v_y,v_z,\\omega_x,\\omega_y,\\omega_z]", "Twist");

  SE3 p;
  Eigen::Vector<Float,6> v{1.,0,0,
                            0,0,.1};
  /*
  while(t < 1)
  {
    Eigen::AngleAxisf aa(p.q);
    std::cout << "Current rotation: " << aa.angle() << " around " << aa.axis().transpose()
              << "\n         a.k.a " << p.q
              << "\n         a.k.a " << p.q.coeffs().transpose() << std::endl;
    std::cout << "  world vel: " << (p.q*v).transpose() << std::endl;
    p.update(v, dt);



    t += dt;
  }



  //return 0;
*/


  logger.update();






  Vec wr = Vec::Zero(6);
  wr(0) = 1.;
  wr(1) = .0;
  wr(2) = .0;
  wr(5) = .4;

  while(t < 20)
  {
    std::cout << "from " << x.pose.p.transpose() ;
    x = robot.xNext(x, wr, dt);
    std::cout << " to " << x.pose.p.transpose() << '\n';
    pose.head(3) = x.pose.p;
    Eigen::AngleAxisf aa(x.pose.q);
    pose.tail(3) = aa.angle() * aa.axis();
    logger.update();
    t += dt;
  }
  logger.plot();




}
