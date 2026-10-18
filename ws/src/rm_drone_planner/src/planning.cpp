#include "rm_drone_planner/planning.hpp"

#include <cmath>
#include <queue>
#include <unordered_map>

namespace rm_drone
{

// ------------------------------------------------------------ OccupancyGrid
void OccupancyGrid::build(const FieldMap & map, const GridConfig & cfg)
{
  cfg_ = cfg;
  origin_ = Eigen::Vector3d(-map.length / 2, -map.width / 2, 0.0);
  size_ = Eigen::Vector3i(
    static_cast<int>(std::ceil(map.length / cfg.resolution)),
    static_cast<int>(std::ceil(map.width / cfg.resolution)),
    static_cast<int>(std::ceil(cfg.ceiling / cfg.resolution)) + 1);
  data_.assign(static_cast<size_t>(size_.prod()), 0);
  for (int x = 0; x < size_.x(); ++x) {
    for (int y = 0; y < size_.y(); ++y) {
      for (int z = 0; z < size_.z(); ++z) {
        const Eigen::Vector3d p = toPos({x, y, z});
        bool occ = !map.insideField(p, cfg.wall_margin, cfg.ceiling) || p.z() < cfg.floor_margin ||
          map.inCollision(p, cfg.obstacle_margin);
        data_[(static_cast<size_t>(x) * size_.y() + y) * size_.z() + z] = occ ? 1 : 0;
      }
    }
  }
}

Eigen::Vector3i OccupancyGrid::toIdx(const Eigen::Vector3d & p) const
{
  return ((p - origin_) / cfg_.resolution).array().floor().cast<int>();
}

Eigen::Vector3d OccupancyGrid::toPos(const Eigen::Vector3i & i) const
{
  return origin_ + (i.cast<double>().array() + 0.5).matrix() * cfg_.resolution;
}

bool OccupancyGrid::valid(const Eigen::Vector3i & i) const
{
  return (i.array() >= 0).all() && (i.array() < size_.array()).all();
}

bool OccupancyGrid::occupiedIdx(int x, int y, int z) const
{
  if (!valid({x, y, z})) {return true;}
  return data_[(static_cast<size_t>(x) * size_.y() + y) * size_.z() + z] != 0;
}

bool OccupancyGrid::occupied(const Eigen::Vector3d & p) const
{
  const auto i = toIdx(p);
  return occupiedIdx(i.x(), i.y(), i.z());
}

bool OccupancyGrid::lineFree(const Eigen::Vector3d & a, const Eigen::Vector3d & b, double step) const
{
  const double L = (b - a).norm();
  const int n = std::max(1, static_cast<int>(std::ceil(L / step)));
  for (int k = 0; k <= n; ++k) {
    if (occupied(a + (b - a) * (static_cast<double>(k) / n))) {return false;}
  }
  return true;
}

bool OccupancyGrid::nearestFree(const Eigen::Vector3d & p, Eigen::Vector3d & out, double max_r) const
{
  if (!occupied(p)) {
    out = p;
    return true;
  }
  const int R = static_cast<int>(max_r / cfg_.resolution);
  const Eigen::Vector3i c = toIdx(p);
  double best = 1e9;
  bool found = false;
  for (int dx = -R; dx <= R; ++dx) {
    for (int dy = -R; dy <= R; ++dy) {
      for (int dz = -R; dz <= R; ++dz) {
        const Eigen::Vector3i i = c + Eigen::Vector3i(dx, dy, dz);
        if (occupiedIdx(i.x(), i.y(), i.z())) {continue;}
        const double d = (toPos(i) - p).norm();
        if (d < best) {
          best = d;
          out = toPos(i);
          found = true;
        }
      }
    }
  }
  return found;
}

// ------------------------------------------------------------ A*
bool astar(const OccupancyGrid & grid, const Eigen::Vector3d & start, const Eigen::Vector3d & goal,
  std::vector<Eigen::Vector3d> & path)
{
  path.clear();
  Eigen::Vector3d s, g;
  if (!grid.nearestFree(start, s, 0.8) || !grid.nearestFree(goal, g, 0.8)) {return false;}
  const Eigen::Vector3i si = grid.toIdx(s), gi = grid.toIdx(g);
  const Eigen::Vector3i sz = grid.size();
  auto key = [&](const Eigen::Vector3i & i) {
      return (static_cast<int64_t>(i.x()) * sz.y() + i.y()) * sz.z() + i.z();
    };
  auto h = [&](const Eigen::Vector3i & i) {return (i - gi).cast<double>().norm();};

  using QItem = std::pair<double, int64_t>;
  std::priority_queue<QItem, std::vector<QItem>, std::greater<QItem>> open;
  std::unordered_map<int64_t, double> gscore;
  std::unordered_map<int64_t, int64_t> parent;
  std::unordered_map<int64_t, Eigen::Vector3i> cell;
  const int64_t sk = key(si), gk = key(gi);
  gscore[sk] = 0.0;
  cell[sk] = si;
  open.push({h(si), sk});
  size_t expanded = 0;
  while (!open.empty() && expanded < 3000000) {
    const auto [f, k] = open.top();
    open.pop();
    if (k == gk) {break;}
    const Eigen::Vector3i c = cell[k];
    const double gc = gscore[k];
    if (f - h(c) > gc + 1e-6) {continue;}
    ++expanded;
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dz = -1; dz <= 1; ++dz) {
          if (!dx && !dy && !dz) {continue;}
          const Eigen::Vector3i n = c + Eigen::Vector3i(dx, dy, dz);
          if (grid.occupiedIdx(n.x(), n.y(), n.z())) {continue;}
          const int64_t nk = key(n);
          const double ng = gc + std::sqrt(static_cast<double>(dx * dx + dy * dy + dz * dz));
          auto it = gscore.find(nk);
          if (it == gscore.end() || ng < it->second) {
            gscore[nk] = ng;
            parent[nk] = k;
            cell[nk] = n;
            open.push({ng + h(n), nk});
          }
        }
      }
    }
  }
  if (!parent.count(gk) && sk != gk) {return false;}
  std::vector<Eigen::Vector3d> rev{goal};
  for (int64_t k = gk; k != sk; k = parent[k]) {rev.push_back(grid.toPos(cell[k]));}
  rev.push_back(start);
  path.assign(rev.rbegin(), rev.rend());
  return true;
}

std::vector<Eigen::Vector3d> simplifyPath(const OccupancyGrid & grid, const std::vector<Eigen::Vector3d> & path)
{
  if (path.size() <= 2) {return path;}
  std::vector<Eigen::Vector3d> out{path.front()};
  size_t i = 0;
  while (i < path.size() - 1) {
    size_t j = path.size() - 1;
    while (j > i + 1 && !grid.lineFree(path[i], path[j])) {--j;}
    out.push_back(path[j]);
    i = j;
  }
  return out;
}

// ------------------------------------------------------------ minimum snap
namespace
{
double fact(int n, int k)  // n!/(n-k)!
{
  double r = 1.0;
  for (int i = 0; i < k; ++i) {r *= (n - i);}
  return r;
}

Eigen::Matrix<double, 1, 8> basis(double t, int d)
{
  Eigen::Matrix<double, 1, 8> b = Eigen::Matrix<double, 1, 8>::Zero();
  for (int i = d; i < 8; ++i) {b(i) = fact(i, d) * std::pow(t, i - d);}
  return b;
}

double allocTime(double L, double v, double a)
{
  if (L < v * v / a) {return 2.0 * std::sqrt(L / a);}
  return L / v + v / a;
}
}  // namespace

bool MinSnapTrajectory::generate(const std::vector<Eigen::Vector3d> & wps, const Eigen::Vector3d & v0,
  const Eigen::Vector3d & a0, const Eigen::Vector3d & vf, double v_max, double a_max)
{
  if (wps.size() < 2) {return false;}
  wps_ = wps;
  const int N = static_cast<int>(wps.size()) - 1;
  T_.assign(N, 0.0);
  total_ = 0.0;
  for (int i = 0; i < N; ++i) {
    T_[i] = std::max(0.3, allocTime((wps[i + 1] - wps[i]).norm(), v_max, a_max));
    total_ += T_[i];
  }
  const int nv = 8 * N;
  const int nc = 8 + (N - 1) * 6;
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(nv, nv);
  for (int s = 0; s < N; ++s) {
    for (int i = 4; i < 8; ++i) {
      for (int j = 4; j < 8; ++j) {
        const int pw = i + j - 7;
        Q(8 * s + i, 8 * s + j) = fact(i, 4) * fact(j, 4) * std::pow(T_[s], pw) / pw;
      }
    }
  }
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(nc, nv);
  Eigen::MatrixXd b = Eigen::MatrixXd::Zero(nc, 3);
  int r = 0;
  // 起点 p v a j
  for (int d = 0; d < 4; ++d) {
    A.block(r, 0, 1, 8) = basis(0.0, d);
    if (d == 0) {b.row(r) = wps.front().transpose();}
    if (d == 1) {b.row(r) = v0.transpose();}
    if (d == 2) {b.row(r) = a0.transpose();}
    ++r;
  }
  // 终点 p v a j
  for (int d = 0; d < 4; ++d) {
    A.block(r, 8 * (N - 1), 1, 8) = basis(T_[N - 1], d);
    if (d == 0) {b.row(r) = wps.back().transpose();}
    if (d == 1) {b.row(r) = vf.transpose();}
    ++r;
  }
  // 中间航点位置 + 0..4 阶连续
  for (int s = 0; s < N - 1; ++s) {
    A.block(r, 8 * s, 1, 8) = basis(T_[s], 0);
    b.row(r) = wps[s + 1].transpose();
    ++r;
    for (int d = 0; d < 5; ++d) {
      A.block(r, 8 * s, 1, 8) = basis(T_[s], d);
      A.block(r, 8 * (s + 1), 1, 8) = -basis(0.0, d);
      ++r;
    }
  }
  Eigen::MatrixXd KKT = Eigen::MatrixXd::Zero(nv + nc, nv + nc);
  KKT.topLeftCorner(nv, nv) = 2.0 * Q;
  KKT.topRightCorner(nv, nc) = A.transpose();
  KKT.bottomLeftCorner(nc, nv) = A;
  Eigen::MatrixXd rhs = Eigen::MatrixXd::Zero(nv + nc, 3);
  rhs.bottomRows(nc) = b;
  const Eigen::MatrixXd sol = KKT.fullPivLu().solve(rhs);
  if (!sol.allFinite()) {return false;}
  coef_.resize(N);
  for (int s = 0; s < N; ++s) {coef_[s] = sol.block(8 * s, 0, 8, 3);}
  return true;
}

TrajState MinSnapTrajectory::sample(double t) const
{
  TrajState st{Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
  if (T_.empty()) {return st;}
  t = std::clamp(t, 0.0, total_);
  size_t s = 0;
  while (s + 1 < T_.size() && t > T_[s]) {
    t -= T_[s];
    ++s;
  }
  t = std::min(t, T_[s]);
  st.p = (basis(t, 0) * coef_[s]).transpose();
  st.v = (basis(t, 1) * coef_[s]).transpose();
  st.a = (basis(t, 2) * coef_[s]).transpose();
  return st;
}

int MinSnapTrajectory::firstCollidingSegment(const OccupancyGrid & grid) const
{
  double t0 = 0.0;
  for (size_t s = 0; s < T_.size(); ++s) {
    for (double t = 0.0; t <= T_[s]; t += 0.02) {
      if (grid.occupied(sample(t0 + t).p)) {return static_cast<int>(s);}
    }
    t0 += T_[s];
  }
  return -1;
}

bool MinSnapTrajectory::generateSafe(const OccupancyGrid & grid, std::vector<Eigen::Vector3d> wps,
  const Eigen::Vector3d & v0, const Eigen::Vector3d & a0, const Eigen::Vector3d & vf, double v_max, double a_max)
{
  for (int iter = 0; iter < 12; ++iter) {
    if (!generate(wps, v0, a0, vf, v_max, a_max)) {return false;}
    const int s = firstCollidingSegment(grid);
    if (s < 0) {return true;}
    wps.insert(wps.begin() + s + 1, 0.5 * (wps[s] + wps[s + 1]));
  }
  return false;
}

}  // namespace rm_drone
