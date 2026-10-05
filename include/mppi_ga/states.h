#ifndef STATES_H
#define STATES_H

#include <mppi_ga/types.h>
#include <Eigen/Geometry>

// define classical states

namespace mppi_ga
{

constexpr Float half{0.5};

struct SE2
{
  Float x{0.};
  Float y{0.};
  Float qw{1.};
  Float qz{0.};

  inline SE2() = default;
  inline SE2(Float x, Float y, Float angle) : x{x}, y{y}, qw{std::cos(angle*half)}, qz{std::sin(angle*half)} {}

  void update(const Float vx, const Float vy, const Float w, Float dt)
  {
    // translation
    const auto c{qw*qw-qz*qz};
    const auto s{2*qw*qz};

	x += (vx*c - vy*s)*dt;
	y += (vy*c + vx*s)*dt;

	// rotation
	const auto qw0{qw};
    qw += -w*qz/2*dt;
    qz += w*qw0/2*dt;

	// normalize
	const auto norm{qw*qw+qz*qz};
	auto scale{Float(2)/(Float(1)+norm)};
	if(std::abs(norm-1) > 2.107342e-08)
	  scale = Float(1)/std::sqrt(norm);
	qw *= scale;
	qz *= scale;
  }
};

struct SE2twist
{
  SE2 pose{};
  Eigen::Vector<Float,3> twist{Vec::Zero(3)}; // vx, vy, w
  inline SE2twist() = default;
  inline void update(const Eigen::Vector<Float,3> &accel, Float dt)
  {
    // update twist
    twist += half*accel*dt;
    pose.update(twist(0), twist(1), twist(2), dt);
    twist += half*accel*dt;
  }
};


struct SE3
{
  Eigen::Quaternion<Float> q{1,0,0,0};
  Eigen::Vector<Float,3> p{Vec::Zero(3)};
  inline SE3() = default;

  inline void update(const Eigen::Vector<Float,6> &twist, Float dt)
  {
    // translation
    p += (q*twist.head<3>())*dt*half;

	// rotation
	// update 3D quaternion from w and dt
	q.coeffs() += (q*Eigen::Quaternion<Float>(0, twist(3), twist(4), twist(5))).coeffs()*dt*half;
	q.normalize();

	// finish translation
	p += (q*twist.head<3>())*dt*half;
  }
};

struct SE3twist
{
  SE3 pose{};
  Eigen::Vector<Float,6> twist{Vec::Zero(6)}; // linear velocity + angular velocity
  inline SE3twist() = default;
  inline void update(const Eigen::Vector<Float,6> &accel, Float dt)
  {
    // update twist
    twist += half*accel*dt/2;
    pose.update(twist, dt);
    twist += half*accel*dt/2;
  }
};

}



#endif // STATES_H
