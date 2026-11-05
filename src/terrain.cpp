#include "uav/terrain.hpp"

#include <algorithm>
#include <cctype>
#include <utility>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iterator>
#include <limits>
#include <random>

#include "uav/common.hpp"

namespace uav {

namespace {

constexpr double kMetresPerDegreeLat = kEarthRadiusM * kPi / 180.0;
constexpr int kHgtVoid = -32768;

double toD(std::size_t v) { return static_cast<double>(v); }

class Rng {
 public:
  explicit Rng(std::uint64_t seed) : engine_(seed) {}
  /// Uniform in [0, 1).
  double uniform() { return static_cast<double>(engine_() >> 11U) * (1.0 / 9007199254740992.0); }
  double symmetric(double amp) { return (uniform() * 2.0 - 1.0) * amp; }

 private:
  std::mt19937_64 engine_;
};

}  // namespace

Terrain::Terrain(std::size_t cols, std::size_t rows, double cellXM, double cellYM, LatLon origin,
                 double latRef, std::vector<float> data)
    : cols_(cols),
      rows_(rows),
      cellX_(cellXM),
      cellY_(cellYM),
      origin_(origin),
      latRef_(latRef),
      data_(std::move(data)) {
  if (cols_ < 2 || rows_ < 2) throw Error("terrain: grid must be at least 2x2");
  if (data_.size() != cols_ * rows_) throw Error("terrain: data size does not match dimensions");
  if (!(cellX_ > 0.0) || !(cellY_ > 0.0)) throw Error("terrain: cell size must be positive");
  mPerDegLat_ = kMetresPerDegreeLat;
  mPerDegLon_ = kMetresPerDegreeLat * std::cos(deg2rad(latRef_));
}

double Terrain::elevation(Vec2 p) const {
  const double fx = std::clamp(p.x / cellX_, 0.0, toD(cols_ - 1));
  const double fy = std::clamp(p.y / cellY_, 0.0, toD(rows_ - 1));
  const std::size_t c0 = std::min(static_cast<std::size_t>(fx), cols_ - 1);
  const std::size_t r0 = std::min(static_cast<std::size_t>(fy), rows_ - 1);
  const std::size_t c1 = std::min(c0 + 1, cols_ - 1);
  const std::size_t r1 = std::min(r0 + 1, rows_ - 1);
  const double tx = fx - toD(c0);
  const double ty = fy - toD(r0);
  const double a = static_cast<double>(at(c0, r0));
  const double b = static_cast<double>(at(c1, r0));
  const double c = static_cast<double>(at(c0, r1));
  const double d = static_cast<double>(at(c1, r1));
  return (a * (1.0 - tx) + b * tx) * (1.0 - ty) + (c * (1.0 - tx) + d * tx) * ty;
}

double Terrain::minElevation() const {
  return static_cast<double>(*std::min_element(data_.begin(), data_.end()));
}

double Terrain::maxElevation() const {
  return static_cast<double>(*std::max_element(data_.begin(), data_.end()));
}

Vec2 Terrain::toXY(LatLon p) const {
  return {(p.lon - origin_.lon) * mPerDegLon_, (p.lat - origin_.lat) * mPerDegLat_};
}

LatLon Terrain::toLatLon(Vec2 p) const {
  return {origin_.lat + p.y / mPerDegLat_, origin_.lon + p.x / mPerDegLon_};
}

bool Terrain::contains(Vec2 p) const {
  return p.x >= 0.0 && p.y >= 0.0 && p.x <= widthM() && p.y <= heightM();
}

Terrain Terrain::crop(std::size_t col0, std::size_t row0, std::size_t w, std::size_t h) const {
  if (w < 2 || h < 2 || col0 + w > cols_ || row0 + h > rows_) throw Error("terrain: crop window out of range");
  std::vector<float> out;
  out.reserve(w * h);
  for (std::size_t r = 0; r < h; ++r) {
    for (std::size_t c = 0; c < w; ++c) out.push_back(at(col0 + c, row0 + r));
  }
  const LatLon newOrigin = toLatLon({toD(col0) * cellX_, toD(row0) * cellY_});
  return Terrain(w, h, cellX_, cellY_, newOrigin, latRef_, std::move(out));
}

Terrain Terrain::downsample(std::size_t factor) const {
  if (factor == 0) throw Error("terrain: downsample factor must be positive");
  if (factor == 1) return *this;
  const std::size_t nc = (cols_ - 1) / factor + 1;
  const std::size_t nr = (rows_ - 1) / factor + 1;
  if (nc < 2 || nr < 2) throw Error("terrain: downsample factor too large");
  std::vector<float> out;
  out.reserve(nc * nr);
  for (std::size_t r = 0; r < nr; ++r) {
    for (std::size_t c = 0; c < nc; ++c) out.push_back(at(c * factor, r * factor));
  }
  return Terrain(nc, nr, cellX_ * toD(factor), cellY_ * toD(factor), origin_, latRef_, std::move(out));
}

Terrain Terrain::synthetic(const SyntheticParams& params) {
  if (params.exponent < 1 || params.exponent > 12) throw Error("synthetic: exponent must be in [1, 12]");
  if (!(params.roughness > 0.0 && params.roughness < 1.0)) throw Error("synthetic: roughness must be in (0, 1)");
  const std::size_t n = (static_cast<std::size_t>(1) << static_cast<unsigned>(params.exponent)) + 1;
  std::vector<double> g(n * n, 0.0);
  Rng rng(params.seed);
  auto idx = [n](std::size_t x, std::size_t y) { return y * n + x; };

  g[idx(0, 0)] = rng.uniform();
  g[idx(n - 1, 0)] = rng.uniform();
  g[idx(0, n - 1)] = rng.uniform();
  g[idx(n - 1, n - 1)] = rng.uniform();

  double amp = 1.0;
  for (std::size_t step = n - 1; step > 1; step /= 2) {
    const std::size_t half = step / 2;
    for (std::size_t y = 0; y + step < n; y += step) {
      for (std::size_t x = 0; x + step < n; x += step) {
        const double avg = (g[idx(x, y)] + g[idx(x + step, y)] + g[idx(x, y + step)] +
                            g[idx(x + step, y + step)]) / 4.0;
        g[idx(x + half, y + half)] = avg + rng.symmetric(amp);
      }
    }
    for (std::size_t y = 0; y < n; y += half) {
      const std::size_t xStart = ((y / half) % 2 == 0) ? half : 0;
      for (std::size_t x = xStart; x < n; x += step) {
        double sum = 0.0;
        double cnt = 0.0;
        if (x >= half) { sum += g[idx(x - half, y)]; cnt += 1.0; }
        if (x + half < n) { sum += g[idx(x + half, y)]; cnt += 1.0; }
        if (y >= half) { sum += g[idx(x, y - half)]; cnt += 1.0; }
        if (y + half < n) { sum += g[idx(x, y + half)]; cnt += 1.0; }
        g[idx(x, y)] = sum / cnt + rng.symmetric(amp);
      }
    }
    amp *= params.roughness;
  }

  const auto mm = std::minmax_element(g.begin(), g.end());
  const double lo = *mm.first;
  const double span = std::max(*mm.second - lo, 1e-12);
  std::vector<float> data(n * n);
  for (std::size_t i = 0; i < g.size(); ++i) {
    data[i] = static_cast<float>((g[i] - lo) / span * params.max_height_m);
  }
  const double extentLat = toD(n - 1) * params.cell_m / kMetresPerDegreeLat;
  return Terrain(n, n, params.cell_m, params.cell_m, params.origin, params.origin.lat + extentLat / 2.0,
                 std::move(data));
}

Terrain Terrain::fromHgt(const std::vector<unsigned char>& bytes, LatLon southWest) {
  if (bytes.size() < 8 || bytes.size() % 2 != 0) throw Error("hgt: invalid file size");
  const std::size_t samples = bytes.size() / 2;
  const auto n = static_cast<std::size_t>(std::llround(std::sqrt(static_cast<double>(samples))));
  if (n < 2 || n * n != samples) throw Error("hgt: file is not a square tile (expected 1201x1201 or 3601x3601 samples)");

  std::vector<float> data(samples, 0.0F);
  std::vector<char> isVoid(samples, 0);
  std::size_t voids = 0;
  for (std::size_t row = 0; row < n; ++row) {      // file row 0 is the northern edge
    for (std::size_t col = 0; col < n; ++col) {
      const std::size_t src = (row * n + col) * 2;
      const int raw = static_cast<std::int16_t>(
          static_cast<std::uint16_t>((static_cast<unsigned>(bytes[src]) << 8U) | bytes[src + 1]));
      const std::size_t dst = (n - 1 - row) * n + col;
      if (raw == kHgtVoid) {
        isVoid[dst] = 1;
        ++voids;
      } else {
        data[dst] = static_cast<float>(raw);
      }
    }
  }

  // Fill voids by repeatedly averaging valid 8-neighbours (grows inwards from the void border).
  std::size_t remaining = voids;
  if (remaining == samples) {
    std::fill(data.begin(), data.end(), 0.0F);
    std::fill(isVoid.begin(), isVoid.end(), static_cast<char>(0));
    remaining = 0;
  }
  while (remaining > 0) {
    std::vector<std::pair<std::size_t, float>> updates;
    for (std::size_t r = 0; r < n; ++r) {
      for (std::size_t c = 0; c < n; ++c) {
        if (isVoid[r * n + c] == 0) continue;
        double sum = 0.0;
        int cnt = 0;
        for (int dr = -1; dr <= 1; ++dr) {
          for (int dc = -1; dc <= 1; ++dc) {
            if (dr == 0 && dc == 0) continue;
            const long rr = static_cast<long>(r) + dr;
            const long cc = static_cast<long>(c) + dc;
            if (rr < 0 || cc < 0 || rr >= static_cast<long>(n) || cc >= static_cast<long>(n)) continue;
            const std::size_t k = static_cast<std::size_t>(rr) * n + static_cast<std::size_t>(cc);
            if (isVoid[k] != 0) continue;
            sum += static_cast<double>(data[k]);
            ++cnt;
          }
        }
        if (cnt > 0) updates.emplace_back(r * n + c, static_cast<float>(sum / cnt));
      }
    }
    if (updates.empty()) break;  // unreachable: at least one valid sample exists
    for (const auto& u : updates) {
      data[u.first] = u.second;
      isVoid[u.first] = 0;
    }
    remaining -= updates.size();
  }

  const double mLon = kMetresPerDegreeLat * std::cos(deg2rad(southWest.lat + 0.5));
  const double step = 1.0 / toD(n - 1);
  Terrain t(n, n, mLon * step, kMetresPerDegreeLat * step, southWest, southWest.lat + 0.5, std::move(data));
  t.voidsFilled_ = voids;
  return t;
}

LatLon Terrain::parseHgtName(const std::string& path) {
  const std::size_t slash = path.find_last_of("/\\");
  std::string name = slash == std::string::npos ? path : path.substr(slash + 1);
  const std::size_t dot = name.find('.');
  if (dot != std::string::npos) name.resize(dot);
  for (char& ch : name) ch = static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));
  auto digits = [&](std::size_t pos, std::size_t len) {
    if (pos + len > name.size()) throw Error("hgt: cannot parse tile name '" + name + "'");
    int v = 0;
    for (std::size_t i = 0; i < len; ++i) {
      const char ch = name[pos + i];
      if (ch < '0' || ch > '9') throw Error("hgt: cannot parse tile name '" + name + "'");
      v = v * 10 + (ch - '0');
    }
    return v;
  };
  if (name.size() < 7 || (name[0] != 'N' && name[0] != 'S') || (name[3] != 'E' && name[3] != 'W')) {
    throw Error("hgt: file name must look like N47E008.hgt, got '" + name + "'");
  }
  const int lat = digits(1, 2);
  const int lon = digits(4, 3);
  return {name[0] == 'N' ? static_cast<double>(lat) : -static_cast<double>(lat),
          name[3] == 'E' ? static_cast<double>(lon) : -static_cast<double>(lon)};
}

Terrain Terrain::loadHgt(const std::string& path) {
  const LatLon sw = parseHgtName(path);
  std::ifstream in(path, std::ios::binary);
  if (!in) throw Error("hgt: cannot open '" + path + "'");
  std::vector<unsigned char> bytes((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  return fromHgt(bytes, sw);
}

std::vector<unsigned char> Terrain::toHgt() const {
  if (cols_ != rows_) throw Error("hgt: only square grids can be encoded");
  std::vector<unsigned char> out(cols_ * rows_ * 2);
  for (std::size_t row = 0; row < rows_; ++row) {
    for (std::size_t col = 0; col < cols_; ++col) {
      const double v = std::round(static_cast<double>(at(col, rows_ - 1 - row)));
      const double clamped = std::clamp(v, -32767.0, 32767.0);
      const auto s = static_cast<std::int16_t>(clamped);
      const auto u = static_cast<std::uint16_t>(s);
      const std::size_t o = (row * cols_ + col) * 2;
      out[o] = static_cast<unsigned char>(u >> 8U);
      out[o + 1] = static_cast<unsigned char>(u & 0xFFU);
    }
  }
  return out;
}

}  // namespace uav
