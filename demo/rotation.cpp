#include <iostream>
#include <mppi_ga/types.h>
#include <mppi_ga/rotation.h>

using namespace mppi_ga;

int main()
{
  Vec v(3);
  v << 1,0,0;

  Vec u(3);
  u << 0,0,1;
  u.normalize();
  Float theta = 1.57/2;

  Vec q(4);
  q << cos(theta/2), u(0)*sin(theta/2), u(1)*sin(theta/2), u(2)*sin(theta/2);

  Vec w(3);
  w << 0,0,1;

  Rotation::rotate(q,v);
  std::cout << v.transpose() << std::endl;

  Vec qn(4);
  Rotation::qNext(q(0),q(1),q(2),q(3),0.1,
                  w(0),w(1),w(2),qn(0),qn(1),qn(2),qn(3));

  theta = 2*atan2(q.segment(1,3).norm(), q(0));
  u = q.segment(1,3)/q.segment(1,3).norm();

  std::cout << theta << " / " << u.transpose() << std::endl;




}
