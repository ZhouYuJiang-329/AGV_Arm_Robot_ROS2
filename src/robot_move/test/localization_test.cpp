#include "robot_move/localization/core.hpp"
#include "robot_move/impl/agv/seer/seer_localization_source.hpp"
#include <iostream>
#include <stdexcept>
#include <limits>
using namespace robot_move::localization;
using Source = robot_move::seer::SeerLocalizationSource;
void check(bool b) {if (!b) {throw std::runtime_error("localization core regression");}}
void equal(const SE2 & a, const SE2 & b) {check(near(a, b, 1e-9, 1e-9));}
int main()
{
  const double pi = std::acos(-1.0);
  equal(compose({1, 2, 0}, {3, 4, 0}), {4, 6, 0});
  equal(compose({0, 0, pi / 2}, {1, 0, pi / 2}), {0, 1, pi});
  SE2 alignment{2, -3, pi / 2}, pose{4, 2, -pi / 4}, offset{0.4, -0.3, pi / 2};
  SE2 base = compose(compose(alignment, pose), offset);
  equal(base, {0.7 / std::sqrt(2.0), 1 + 0.1 / std::sqrt(2.0), 3 * pi / 4});
  SE2 odom{1.2, -2, 0.7};
  SE2 correction = compose(base, inverse(odom));
  equal(compose(correction, odom), base);
  equal(compose(inverse(alignment), base), compose(pose, offset));
  equal(compose(base, inverse(base)), {});
  SE2 odom_copy = odom;
  Gate g; g.expected_map = "floor"; g.expected_md5 = "abc";
  Sample s; s.map = "floor"; s.md5 = "abc"; s.status = 1; s.confidence = 0.8;
  check(g.quality(s).empty());
  s.status = 2; check(!g.quality(s).empty()); s.status = 3; check(!g.quality(s).empty());
  s.status = 1; s.map = "other"; check(g.quality(s) == "map_mismatch"); s.map = "floor";
  s.md5 = "def"; check(!g.quality(s).empty()); s.md5 = "abc";
  s.confidence.reset(); check(!g.quality(s).empty());
  g.require_confidence = false; check(g.quality(s).empty()); s.confidence = 0.1;
  check(!g.quality(s).empty()); s.confidence = 0.9;
  s.method = 3; check(!g.quality(s).empty()); s.method = 0;
  s.pose.x = std::numeric_limits<double>::quiet_NaN(); check(!g.quality(s).empty());
  Gate map_gate; map_gate.expected_map = "floor";
  s.pose = {}; check(map_gate.quality(s).empty());
  s.md5 = "new"; check(map_gate.quality(s) == "map_content_changed_restart_required");
  s.md5 = "abc"; check(map_gate.quality(s).empty());
  check(g.update(correction).empty());
  auto shifted = compose(SE2{3, 0, 0.5}, correction);
  check(g.update(shifted) == "jump_quarantined");
  g.interrupt(); check(g.update(shifted) == "jump_quarantined");
  check(g.update(shifted) == "jump_quarantined");
  check(g.update(shifted) == "persistent_correction_accepted");
  equal(odom, odom_copy); equal(*g.accepted, shifted);
  g.recovery_samples = 0; check(g.update(correction) == "jump_quarantined");
  check(g.update(correction) == "jump_quarantined");
  check(timing(0.1, 0.1, 0.2, 0.8, 0.6, 0.4).empty());
  check(timing(1, 0.1, 0.2, 0.8, 0.6, 0.4) == "stale_sample");
  check(timing(-1, 0.1, 0.2, 0.8, 0.6, 0.4) == "clock_backward");
  check(timing(0.1, 0.7, 0.2, 0.8, 0.6, 0.4) == "query_latency_exceeded");
  check(timing(0.1, 0.1, 0.5, 0.8, 0.6, 0.4) == "odom_time_gap");
  Sample p; p.pose.x = 99;
  check(!Source::parsePose("{\"x\":1,\"y\":2}", p)); check(p.pose.x == 99);
  check(!Source::parsePose("{\"x\":1,\"y\":2,\"angle\":0,\"confidence\":2}", p));
  check(!Source::parsePose("{\"x\":1,\"y\":2,\"angle\":0} trailing", p));
  check(Source::parsePose("{\"x\":1,\"y\":2,\"angle\":0,\"create_on\":\"unknown\"}", p));
  check(!p.confidence); int status = -1;
  check(Source::parseStatus("{\"reloc_status\":3}", status) && status == 3);
  check(!Source::parseStatus("{\"reloc_status\":1.5}", status));
  std::string map, md5;
  check(Source::parseMap("{\"current_map\":\"floor\",\"current_map_md5\":\"abc\"}", map, md5));
  check(map == "floor" && md5 == "abc");
  check(!Source::parseMap("{\"current_map\":\"\"}", map, md5));
  std::cout << "localization transforms, gates, timing and parser passed\n";
}
