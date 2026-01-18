#pragma once
#include <array>
#include <vector>

static constexpr size_t ARM_DOF = 7;
using JointArray = std::array<double, ARM_DOF>;
using JointGrid  = std::vector<std::vector<JointArray>>;

class JointGridGenerator
{
public:
  JointGridGenerator(
    const JointArray& left_up,
    const JointArray& right_up,
    const JointArray& left_down,
    const JointArray& right_down);

  JointGrid generate(size_t grid_size = 10) const;

private:
  JointArray lu_, ru_, ld_, rd_;
};
