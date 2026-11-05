#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "uav/geo.hpp"

namespace uav {

/// Parameters of the seeded diamond-square terrain generator.
struct SyntheticParams {
  int exponent = 7;                 ///< grid is (2^exponent + 1) samples per side
  std::uint64_t seed = 1;
  double roughness = 0.55;          ///< amplitude decay per octave, (0, 1)
  double max_height_m = 1500.0;     ///< heights are normalised to [0, max_height_m]
  double cell_m = 30.0;             ///< horizontal spacing in metres
  LatLon origin{47.0, 8.0};         ///< south-west corner
};

/// Regular elevation grid. Row 0 is the southern edge, column 0 the western edge.
/// Planar coordinates (x east, y north, metres) are measured from the south-west corner.
class Terrain {
 public:
  Terrain() = default;
  Terrain(std::size_t cols, std::size_t rows, double cellXM, double cellYM, LatLon origin,
          double latRef, std::vector<float> data);

  [[nodiscard]] std::size_t cols() const { return cols_; }
  [[nodiscard]] std::size_t rows() const { return rows_; }
  [[nodiscard]] double cellX() const { return cellX_; }
  [[nodiscard]] double cellY() const { return cellY_; }
  [[nodiscard]] double widthM() const { return static_cast<double>(cols_ - 1) * cellX_; }
  [[nodiscard]] double heightM() const { return static_cast<double>(rows_ - 1) * cellY_; }
  [[nodiscard]] LatLon origin() const { return origin_; }
  [[nodiscard]] double latRef() const { return latRef_; }
  [[nodiscard]] const std::vector<float>& data() const { return data_; }
  [[nodiscard]] std::size_t voidsFilled() const { return voidsFilled_; }

  [[nodiscard]] float at(std::size_t col, std::size_t row) const { return data_[row * cols_ + col]; }
  /// Bilinear elevation at planar position p (clamped to the grid).
  [[nodiscard]] double elevation(Vec2 p) const;
  [[nodiscard]] double minElevation() const;
  [[nodiscard]] double maxElevation() const;

  [[nodiscard]] Vec2 toXY(LatLon p) const;
  [[nodiscard]] LatLon toLatLon(Vec2 p) const;
  [[nodiscard]] bool contains(Vec2 p) const;

  [[nodiscard]] Terrain crop(std::size_t col0, std::size_t row0, std::size_t w, std::size_t h) const;
  [[nodiscard]] Terrain downsample(std::size_t factor) const;

  [[nodiscard]] static Terrain synthetic(const SyntheticParams& params);
  /// Parse SRTM .hgt bytes (big-endian int16, square tile, -32768 = void).
  [[nodiscard]] static Terrain fromHgt(const std::vector<unsigned char>& bytes, LatLon southWest);
  /// Load an .hgt file; the tile origin is taken from the file name (e.g. N47E008.hgt).
  [[nodiscard]] static Terrain loadHgt(const std::string& path);
  [[nodiscard]] static LatLon parseHgtName(const std::string& path);
  /// Encode as big-endian int16 .hgt bytes (grid must be square).
  [[nodiscard]] std::vector<unsigned char> toHgt() const;

 private:
  std::size_t cols_ = 0;
  std::size_t rows_ = 0;
  double cellX_ = 1.0;
  double cellY_ = 1.0;
  LatLon origin_;
  double latRef_ = 0.0;
  double mPerDegLat_ = 1.0;
  double mPerDegLon_ = 1.0;
  std::vector<float> data_;
  std::size_t voidsFilled_ = 0;
};

}  // namespace uav
