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
        bool occ = !map.insideField(p, cfg.wall_margin, cfg.ceiling) || p.z() < cfg.floor_margin;
        data_[(static_cast<size_t>(x) * size_.y() + y) * size_.z() + z] = occ ? 1 : 0;
      }
    }
  }
  // CAD maps contain many surface boxes. Rasterize each inflated box into its
  // bounded grid range instead of testing every box for every cell.
  auto rasterize = [&](const Eigen::Vector3d & lo, const Eigen::Vector3d & hi, auto contains) {
      Eigen::Vector3i first = toIdx(lo) - Eigen::Vector3i::Ones();
      Eigen::Vector3i last = toIdx(hi) + Eigen::Vector3i::Ones();
      first = first.cwiseMax(Eigen::Vector3i::Zero());
      last = last.cwiseMin(size_ - Eigen::Vector3i::Ones());
      for (int x = first.x(); x <= last.x(); ++x) {
        for (int y = first.y(); y <= last.y(); ++y) {
          for (int z = first.z(); z <= last.z(); ++z) {
            if (contains(toPos({x, y, z}))) {
              data_[(static_cast<size_t>(x) * size_.y() + y) * size_.z() + z] = 1;
            }
          }
        }
      }
    };
  const double margin = cfg.obstacle_margin;
  for (const auto & box : map.obstacles) {
    rasterize(box.min().array() - margin, box.max().array() + margin,
      [&](const Eigen::Vector3d & p) {return box.contains(p, margin);});
  }
  for (const auto & column : map.nofly) {
    rasterize({column.x - column.hx - margin, column.y - column.hy - margin, -margin},
      {column.x + column.hx + margin, column.y + column.hy + margin, cfg.ceiling},
      [&](const Eigen::Vector3d & p) {
        return p.z() >= -margin && std::abs(p.x() - column.x) <= column.hx + margin &&
          std::abs(p.y() - column.y) <= column.hy + margin;
      });
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

void OccupancyGrid::markOccupied(const Eigen::Vector3d & p)
{
  const auto i = toIdx(p);
  if (valid(i)) {data_[(static_cast<size_t>(i.x()) * size_.y() + i.y()) * size_.z() + i.z()] = 1;}
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

bool OccupancyGrid::exitFree(const Eigen::Vector3d & from, const Eigen::Vector3d & to, double step) const
{
  const double L = (to - from).norm();
  if (L < 1e-6) {return !occupied(to);}
  const int n = std::max(1, static_cast<int>(std::ceil(L / step)));
  for (int k = 1; k <= n; ++k) {
    if (occupied(from + (to - from) * (static_cast<double>(k) / n))) {return false;}
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
        if (!exitFree(p, toPos(i))) {continue;}
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
          // 对角步必须同时能走对应的轴向邻格，避免从两个障碍的夹角穿过去。
          const bool diagonal = (dx && dy) || (dx && dz) || (dy && dz);
          if (diagonal &&
            ((dx && grid.occupiedIdx(c.x() + dx, c.y(), c.z())) ||
            (dy && grid.occupiedIdx(c.x(), c.y() + dy, c.z())) ||
            (dz && grid.occupiedIdx(c.x(), c.y(), c.z() + dz)))) {continue;}
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

bool MinSnapTrajectory::solve(const std::vector<Eigen::Vector3d> & wps, const Eigen::Vector3d & v0,
  const Eigen::Vector3d & a0, const Eigen::Vector3d & vf, const std::vector<double> & T, Solved & out)
{
  const int N = static_cast<int>(wps.size()) - 1;
  if (N < 1 || static_cast<int>(T.size()) != N) {return false;}
  const int nv = 8 * N;
  const int nc = 8 + (N - 1) * 6;
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(nv, nv);
  for (int s = 0; s < N; ++s) {
    for (int i = 4; i < 8; ++i) {
      for (int j = 4; j < 8; ++j) {
        const int pw = i + j - 7;
        Q(8 * s + i, 8 * s + j) = fact(i, 4) * fact(j, 4) * std::pow(T[s], pw) / pw;
      }
    }
  }
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(nc, nv);
  Eigen::MatrixXd b = Eigen::MatrixXd::Zero(nc, 3);
  int r = 0;
  for (int d = 0; d < 4; ++d) {
    A.block(r, 0, 1, 8) = basis(0.0, d);
    if (d == 0) {b.row(r) = wps.front().transpose();}
    if (d == 1) {b.row(r) = v0.transpose();}
    if (d == 2) {b.row(r) = a0.transpose();}
    ++r;
  }
  for (int d = 0; d < 4; ++d) {
    A.block(r, 8 * (N - 1), 1, 8) = basis(T[N - 1], d);
    if (d == 0) {b.row(r) = wps.back().transpose();}
    if (d == 1) {b.row(r) = vf.transpose();}
    ++r;
  }
  for (int s = 0; s < N - 1; ++s) {
    A.block(r, 8 * s, 1, 8) = basis(T[s], 0);
    b.row(r) = wps[s + 1].transpose();
    ++r;
    for (int d = 0; d < 5; ++d) {
      A.block(r, 8 * s, 1, 8) = basis(T[s], d);
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
  if (!sol.allFinite() || r != nc) {return false;}
  out.T = T;
  out.coef.resize(N);
  out.total = 0.0;
  for (int s = 0; s < N; ++s) {
    out.coef[s] = sol.block(8 * s, 0, 8, 3);
    out.total += T[s];
    if (!out.coef[s].allFinite()) {return false;}
  }
  const Eigen::MatrixXd coefs = sol.topRows(nv);
  const Eigen::MatrixXd residual = A * coefs - b;
  out.residual = residual.norm();
  const auto s0 = sampleSolved(out, 0.0);
  const auto s1 = sampleSolved(out, out.total);
  out.residual += (s0.p - wps.front()).norm() + (s0.v - v0).norm() + (s0.a - a0).norm();
  out.residual += (s1.p - wps.back()).norm() + (s1.v - vf).norm();
  double pv = 0.0, pa = 0.0;
  const double dt = std::min(0.01, std::max(1e-3, out.total / 400.0));
  for (double t = 0.0; t <= out.total; t += dt) {
    const auto st = sampleSolved(out, std::min(t, out.total));
    if (!st.p.allFinite() || !st.v.allFinite() || !st.a.allFinite()) {return false;}
    pv = std::max(pv, st.v.norm());
    pa = std::max(pa, st.a.norm());
  }
  out.peak_v = pv;
  out.peak_a = pa;
  return out.residual < 1e-2;
}

TrajState MinSnapTrajectory::sampleSolved(const Solved & sol, double t)
{
  TrajState st{Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
  if (sol.T.empty() || sol.coef.empty()) {return st;}
  t = std::clamp(t, 0.0, sol.total);
  size_t s = 0;
  while (s + 1 < sol.T.size() && t > sol.T[s]) {
    t -= sol.T[s];
    ++s;
  }
  t = std::min(t, sol.T[s]);
  st.p = (basis(t, 0) * sol.coef[s]).transpose();
  st.v = (basis(t, 1) * sol.coef[s]).transpose();
  st.a = (basis(t, 2) * sol.coef[s]).transpose();
  return st;
}

bool MinSnapTrajectory::withinLimits(const Solved & s, double v_max, double a_max)
{
  return s.peak_v <= v_max * 1.02 && s.peak_a <= a_max * 1.02;
}

bool MinSnapTrajectory::generate(const std::vector<Eigen::Vector3d> & wps, const Eigen::Vector3d & v0,
  const Eigen::Vector3d & a0, const Eigen::Vector3d & vf, double v_max, double a_max)
{
  if (wps.size() < 2 || !(v_max > 0.05) || !(a_max > 0.05)) {return false;}
  if (!v0.allFinite() || !a0.allFinite() || !vf.allFinite()) {return false;}
  for (const auto & p : wps) {
    if (!p.allFinite()) {return false;}
  }
  // 边界条件本身已经超限时，拉长时间也无法满足，不能把起点速度清成 0 再假装通过。
  if (v0.norm() > v_max * 1.02 || a0.norm() > a_max * 1.02 || vf.norm() > v_max * 1.02) {return false;}

  const int N = static_cast<int>(wps.size()) - 1;
  std::vector<double> T(N, 0.0);
  for (int i = 0; i < N; ++i) {
    T[i] = std::max(0.3, allocTime((wps[i + 1] - wps[i]).norm(), v_max, a_max));
  }
  Solved best;
  bool found = false;
  for (int iter = 0; iter < 8; ++iter) {
    Solved cand;
    if (!solve(wps, v0, a0, vf, T, cand)) {return false;}
    if (withinLimits(cand, v_max, a_max)) {
      best = std::move(cand);
      found = true;
      break;
    }
    const double sv = cand.peak_v > v_max ? cand.peak_v / v_max : 1.0;
    const double sa = cand.peak_a > a_max ? std::sqrt(cand.peak_a / a_max) : 1.0;
    const double scale = std::max(sv, sa) * 1.15;
    if (!(scale > 1.02) || !std::isfinite(scale)) {return false;}
    for (double & seg : T) {seg *= scale;}
  }
  if (!found) {return false;}
  T_ = best.T;
  coef_ = best.coef;
  wps_ = wps;
  total_ = best.total;
  peak_v_ = best.peak_v;
  peak_a_ = best.peak_a;
  approved_ = true;
  return true;
}

TrajState MinSnapTrajectory::sample(double t) const
{
  Solved sol;
  sol.T = T_;
  sol.coef = coef_;
  sol.total = total_;
  if (!approved_) {return {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};}
  return sampleSolved(sol, t);
}

int MinSnapTrajectory::firstCollidingSegment(const OccupancyGrid & grid) const
{
  if (!approved_ || T_.empty()) {return 0;}
  double t0 = 0.0;
  Eigen::Vector3d prev = sample(0.0).p;
  for (size_t s = 0; s < T_.size(); ++s) {
    for (double t = 0.02; t <= T_[s] + 1e-9; t += 0.02) {
      const Eigen::Vector3d p = sample(std::min(t0 + t, total_)).p;
      if (!grid.lineFree(prev, p, 0.05)) {return static_cast<int>(s);}
      prev = p;
    }
    t0 += T_[s];
  }
  return -1;
}

bool MinSnapTrajectory::generateSafe(const OccupancyGrid & grid, std::vector<Eigen::Vector3d> wps,
  const Eigen::Vector3d & v0, const Eigen::Vector3d & a0, const Eigen::Vector3d & vf, double v_max, double a_max)
{
  for (int iter = 0; iter < 12; ++iter) {
    MinSnapTrajectory trial;
    if (!trial.generate(wps, v0, a0, vf, v_max, a_max)) {return false;}
    const int s = trial.firstCollidingSegment(grid);
    if (s < 0) {
      *this = std::move(trial);
      return true;
    }
    if (s + 1 >= static_cast<int>(wps.size())) {return false;}
    wps.insert(wps.begin() + s + 1, 0.5 * (wps[s] + wps[s + 1]));
  }
  return false;
}

}  // namespace rm_drone
