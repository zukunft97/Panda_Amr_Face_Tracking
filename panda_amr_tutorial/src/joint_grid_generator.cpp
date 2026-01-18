#include "panda_amr_tutorial/joint_grid_generator.h"

JointGridGenerator::JointGridGenerator(
  const JointArray& left_up,
  const JointArray& right_up,
  const JointArray& left_down,
  const JointArray& right_down)
  : lu_(left_up), ru_(right_up),
    ld_(left_down), rd_(right_down)
{}

JointGrid JointGridGenerator::generate(size_t N) const
{
  JointGrid grid(N, std::vector<JointArray>(N));
//   grid[0][0] = ld_;
//   grid[0][N-1] = lu_;
//   grid[N-1][0] = rd_;
//   grid[N-1][N-1] = ru_;

  JointArray diff_x0{};
  JointArray diff_xn{};
  
  for (size_t i = 0; i < ARM_DOF; i++){
    diff_x0[i] = (rd_[i] - ld_[i]) / (N - 1);
    diff_xn[i] = (ru_[i] - lu_[i]) / (N - 1);
  }

  for (size_t i = 0; i < N; i++){
    for (size_t j = 0; j < ARM_DOF; j++){
        grid[i][0][j] = ld_[j] + i * diff_x0[j];
        grid[i][N-1][j] = lu_[j] + i * diff_xn[j];
    }
  }

    std::vector<JointArray>diff_y(N);
    for (size_t i = 0; i < N; i++){
        for (size_t j = 0; j < ARM_DOF; j++){
            diff_y[i][j] = (grid[i][N-1][j] - grid[i][0][j]) / (N - 1);
        }
    }

  for (size_t i = 0; i < N; i++){
    for (size_t j = 0; j < N; j++){
        for (size_t k = 0; k < ARM_DOF; k++){
            grid[i][j][k] = grid[i][0][k] + j * diff_y[i][k];
        }
    }
  }

//   {
//     double tx = static_cast<double>(x) / (N - 1);

//     for (size_t y = 0; y < N; ++y)
//     {
//       double ty = static_cast<double>(y) / (N - 1);

//       for (size_t j = 0; j < ARM_DOF; ++j)
//       {
//         grid[x][y][j] =
//           (1 - tx) * (1 - ty) * ld_[j] +
//           (tx)     * (1 - ty) * rd_[j] +
//           (1 - tx) * (ty)     * lu_[j] +
//           (tx)     * (ty)     * ru_[j];
//       }
//     }
//   }
  return grid;
}
