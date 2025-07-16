#ifndef MPPI_GA_DEMO_H
#define MPPI_GA_DEMO_H

#include <string>
#include <log2plot/config_manager.h>

inline auto result_path(const std::string &robot, const log2plot::ConfigManager &config)
{
  if(config.read<bool>("plot"))
    return std::string("/tmp/robot_");

  using std::string;
  auto ret{MPPI_GA_RESULT_DIR + robot + '/'};
  if(config.read<bool>("bf"))
    ret += "single_";
  else
    ret += config.read<bool>("multi") ? "multi_" : "single_";

  ret += config.read<string>("hor") + '_' + config.read<string>("cont") + '/';

  ret += config.read<string>("pop");
  if(config.read<bool>("bf"))
    ret += 'f';
  if(const auto sub{config.read<int>("sub")}; sub > 1)
  {
    ret += "_" + std::to_string(sub);
    if(config.read<bool>("use_zoh"))
      ret += "zoh";
  }

  std::cout << "Output: " << ret << std::endl;

  return ret + '_';
}



#endif // MPPI_GA_DEMO_H
