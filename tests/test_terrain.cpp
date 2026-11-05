#include <cstdint>
#include <vector>

#include "test_util.hpp"
#include "uav/terrain.hpp"

using namespace uav;

namespace {

void putBe16(std::vector<unsigned char>& out, int value) {
  const auto u = static_cast<std::uint16_t>(static_cast<std::int16_t>(value));
  out.push_back(static_cast<unsigned char>(u >> 8U));
  out.push_back(static_cast<unsigned char>(u & 0xFFU));
}

std::vector<unsigned char> tile(const std::vector<int>& values) {
  std::vector<unsigned char> bytes;
  for (const int v : values) putBe16(bytes, v);
  return bytes;
}

}  // namespace

TEST(synthetic_is_deterministic) {
  SyntheticParams sp;
  sp.exponent = 5;
  sp.seed = 7;
  const Terrain a = Terrain::synthetic(sp);
  const Terrain b = Terrain::synthetic(sp);
  CHECK(a.data() == b.data());
  sp.seed = 8;
  const Terrain c = Terrain::synthetic(sp);
  CHECK(a.data() != c.data());
}

TEST(synthetic_dimensions_and_range) {
  SyntheticParams sp;
  sp.exponent = 5;
  sp.max_height_m = 800.0;
  sp.cell_m = 25.0;
  const Terrain t = Terrain::synthetic(sp);
  CHECK(t.cols() == 33 && t.rows() == 33);
  CHECK_NEAR(t.widthM(), 32 * 25.0, 1e-9);
  CHECK_NEAR(t.minElevation(), 0.0, 1e-3);
  CHECK_NEAR(t.maxElevation(), 800.0, 1e-2);
  CHECK_THROWS(Terrain::synthetic([] {
    SyntheticParams bad;
    bad.exponent = 0;
    return bad;
  }()));
}

TEST(bilinear_interpolation) {
  const Terrain t(2, 2, 10.0, 10.0, LatLon(47.0, 8.0), 47.0, {0.0F, 10.0F, 20.0F, 30.0F});
  CHECK_NEAR(t.elevation(Vec2(0, 0)), 0.0, 1e-9);
  CHECK_NEAR(t.elevation(Vec2(10, 0)), 10.0, 1e-9);
  CHECK_NEAR(t.elevation(Vec2(5, 5)), 15.0, 1e-9);
  CHECK_NEAR(t.elevation(Vec2(-5, 100)), 20.0, 1e-9);  // clamped to the grid
  CHECK(t.contains(Vec2(10, 10)));
  CHECK(!t.contains(Vec2(10.1, 5)));
}

TEST(geo_mapping_round_trip) {
  SyntheticParams sp;
  sp.exponent = 4;
  const Terrain t = Terrain::synthetic(sp);
  const Vec2 p(123.0, 345.0);
  const Vec2 q = t.toXY(t.toLatLon(p));
  CHECK_NEAR(q.x, p.x, 1e-6);
  CHECK_NEAR(q.y, p.y, 1e-6);
  const LatLon o = t.toLatLon(Vec2(0, 0));
  CHECK_NEAR(o.lat, 47.0, 1e-12);
  CHECK_NEAR(o.lon, 8.0, 1e-12);
}

TEST(hgt_orientation_and_values) {
  // File rows run north to south; internal row 0 is the southern edge.
  const std::vector<unsigned char> bytes = tile({300, 301, 302, 200, 201, 202, 100, 101, 102});
  const Terrain t = Terrain::fromHgt(bytes, LatLon(47.0, 8.0));
  CHECK(t.cols() == 3 && t.rows() == 3);
  CHECK_NEAR(static_cast<double>(t.at(0, 0)), 100.0, 1e-9);
  CHECK_NEAR(static_cast<double>(t.at(2, 0)), 102.0, 1e-9);
  CHECK_NEAR(static_cast<double>(t.at(0, 2)), 300.0, 1e-9);
  CHECK_NEAR(static_cast<double>(t.at(2, 2)), 302.0, 1e-9);
  CHECK_NEAR(t.heightM(), 111194.9, 5.0);  // one degree of latitude
  CHECK(t.voidsFilled() == 0);
}

TEST(hgt_negative_elevations) {
  const Terrain t = Terrain::fromHgt(tile({-10, -20, -30, -40}), LatLon(31.0, 35.0));
  CHECK_NEAR(static_cast<double>(t.at(0, 1)), -10.0, 1e-9);
  CHECK_NEAR(static_cast<double>(t.at(1, 0)), -40.0, 1e-9);
}

TEST(hgt_voids_are_filled) {
  const std::vector<unsigned char> bytes = tile({10, 10, 10, 10, -32768, 10, 10, 10, 10});
  const Terrain t = Terrain::fromHgt(bytes, LatLon(0.0, 0.0));
  CHECK(t.voidsFilled() == 1);
  CHECK_NEAR(static_cast<double>(t.at(1, 1)), 10.0, 1e-6);

  // A wide void is filled from its border inwards.
  std::vector<int> big(25, 100);
  for (const int i : {6, 7, 8, 11, 12, 13, 16, 17, 18}) big[static_cast<std::size_t>(i)] = -32768;
  const Terrain t2 = Terrain::fromHgt(tile(big), LatLon(0.0, 0.0));
  CHECK(t2.voidsFilled() == 9);
  for (const float v : t2.data()) CHECK_NEAR(static_cast<double>(v), 100.0, 1e-6);

  // A tile without any valid sample degrades to sea level instead of failing.
  const Terrain t3 = Terrain::fromHgt(tile({-32768, -32768, -32768, -32768}), LatLon(0.0, 0.0));
  CHECK_NEAR(t3.maxElevation(), 0.0, 1e-9);
}

TEST(hgt_round_trip) {
  SyntheticParams sp;
  sp.exponent = 3;
  sp.max_height_m = 3000.0;
  const Terrain t = Terrain::synthetic(sp);
  const std::vector<unsigned char> bytes = t.toHgt();
  CHECK(bytes.size() == 9 * 9 * 2);
  const Terrain r = Terrain::fromHgt(bytes, LatLon(47.0, 8.0));
  CHECK(r.cols() == t.cols());
  for (std::size_t i = 0; i < t.data().size(); ++i) {
    CHECK_NEAR(static_cast<double>(r.data()[i]), static_cast<double>(t.data()[i]), 0.51);
  }
}

TEST(hgt_rejects_bad_input) {
  CHECK_THROWS(Terrain::fromHgt(std::vector<unsigned char>(10, 0), LatLon(0, 0)));  // not square
  CHECK_THROWS(Terrain::fromHgt(std::vector<unsigned char>(), LatLon(0, 0)));
  CHECK_THROWS(Terrain::loadHgt("N47E008_missing.hgt"));
}

TEST(hgt_tile_names) {
  const LatLon a = Terrain::parseHgtName("N47E008.hgt");
  CHECK_NEAR(a.lat, 47.0, 1e-12);
  CHECK_NEAR(a.lon, 8.0, 1e-12);
  const LatLon b = Terrain::parseHgtName("/data/srtm/S34W071.HGT");
  CHECK_NEAR(b.lat, -34.0, 1e-12);
  CHECK_NEAR(b.lon, -71.0, 1e-12);
  CHECK_THROWS(Terrain::parseHgtName("terrain.hgt"));
  CHECK_THROWS(Terrain::parseHgtName("N4xE008.hgt"));
}

TEST(crop_and_downsample) {
  SyntheticParams sp;
  sp.exponent = 3;
  sp.cell_m = 20.0;
  const Terrain t = Terrain::synthetic(sp);
  const Terrain c = t.crop(2, 3, 4, 5);
  CHECK(c.cols() == 4 && c.rows() == 5);
  CHECK_NEAR(static_cast<double>(c.at(0, 0)), static_cast<double>(t.at(2, 3)), 1e-9);
  const Vec2 shifted = t.toXY(c.toLatLon(Vec2(0, 0)));
  CHECK_NEAR(shifted.x, 40.0, 1e-6);
  CHECK_NEAR(shifted.y, 60.0, 1e-6);
  const Terrain d = t.downsample(2);
  CHECK(d.cols() == 5);
  CHECK_NEAR(d.cellX(), 40.0, 1e-9);
  CHECK_NEAR(static_cast<double>(d.at(1, 1)), static_cast<double>(t.at(2, 2)), 1e-9);
  CHECK_THROWS(t.crop(7, 7, 4, 4));
  CHECK_THROWS(t.downsample(0));
}

TEST_MAIN()
