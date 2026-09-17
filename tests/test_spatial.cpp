#include <cassert>
#include "robomath/spatial/Twist.hpp"
#include "robomath/spatial/Wrench.hpp"
#include "robomath/core/se3.hpp"

using Matrix3 = Eigen::Matrix3d;
using Vector3 = Eigen::Vector3d;

int main() {
  Twist V(Vector3(0, 0, 1), Vector3(2, 0, 0));
  assert(V.angular().isApprox(Vector3(0, 0, 1)));
  assert(V.linear().isApprox(Vector3(2, 0, 0)));

  // frame {b} sits one unit up the z axis from frame {a}
  Vector3 p(0, 0, 1);
  Eigen::Matrix4d T = transform(Matrix3::Identity(), p);

  // unit force along +x through the origin of {a}, no moment
  Wrench F_a(Vector3::Zero(), Vector3(1, 0, 0));
  Wrench F_b = wrench_transform(F_a, T);

  // same force, plus the moment its 1-unit offset arm creates about {b}
  assert(F_b.force().isApprox(Vector3(1, 0, 0)));
  assert(F_b.moment().isApprox(Vector3(0, -1, 0)));
  assert(F_b.moment().isApprox(-p.cross(F_a.force())));
}
