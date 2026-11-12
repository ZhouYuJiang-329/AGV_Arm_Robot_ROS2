#include "robot_move/impl/agv/seer/seer_localization_source.hpp"
#include "robot_move/interface/cJSON.h"
#include <memory>
namespace robot_move::seer
{
namespace
{
using Json = std::unique_ptr<cJSON, decltype(&cJSON_Delete)>;
Json parse(const std::string & body)
{
  Json json(nullptr, cJSON_Delete);
  if (body.find('\0') != std::string::npos) {return json;}
  json.reset(cJSON_ParseWithOpts(body.c_str(), nullptr, true));
  auto code = cJSON_GetObjectItemCaseSensitive(json.get(), "ret_code");
  if (!cJSON_IsObject(json.get()) || (code &&
    (!cJSON_IsNumber(code) || code->valuedouble != 0))) {json.reset();}
  return json;
}
bool number(cJSON * j, const char * key, double & v)
{
  auto f = cJSON_GetObjectItemCaseSensitive(j, key);
  if (!cJSON_IsNumber(f) || !std::isfinite(f->valuedouble)) {return false;}
  v = f->valuedouble; return true;
}
}
bool SeerLocalizationSource::parsePose(const std::string & body, localization::Sample & out)
{
  auto j = parse(body); if (!j) {return false;}
  localization::Sample s;
  if (!number(j.get(), "x", s.pose.x) || !number(j.get(), "y", s.pose.y) ||
    !number(j.get(), "angle", s.pose.angle)) {return false;}
  double v;
  if (cJSON_GetObjectItemCaseSensitive(j.get(), "confidence")) {
    if (!number(j.get(), "confidence", v) || v < 0 || v > 1) {return false;}
    s.confidence = v;
  }
  if (cJSON_GetObjectItemCaseSensitive(j.get(), "loc_method")) {
    if (!number(j.get(), "loc_method", v) || v < 0 || v > 8 || std::floor(v) != v)
    {return false;}
    s.method = static_cast<int>(v);
  }
  out = s; return true;  // create_on intentionally unused: API upload time.
}
bool SeerLocalizationSource::parseStatus(const std::string & body, int & out)
{
  auto j = parse(body); double v;
  if (!j || !number(j.get(), "reloc_status", v) || v < 0 || v > 3 || std::floor(v) != v)
  {return false;}
  out = static_cast<int>(v); return true;
}
bool SeerLocalizationSource::parseMap(
  const std::string & body, std::string & map,
  std::string & md5)
{
  auto j = parse(body); if (!j) {return false;}
  auto m = cJSON_GetObjectItemCaseSensitive(j.get(), "current_map");
  auto h = cJSON_GetObjectItemCaseSensitive(j.get(), "current_map_md5");
  if (!cJSON_IsString(m) || !m->valuestring[0] || (h && !cJSON_IsString(h))) {return false;}
  map = m->valuestring; md5 = h ? h->valuestring : ""; return true;
}
bool SeerLocalizationSource::read(
  localization::Sample & out, Deadline deadline,
  const std::function<void()> & received)
{
  std::string body, map, md5, after_map, after_md5;
  int status, after_status;
  localization::Sample s;
  auto request = [&](int api) {
      return transport_.request(ApiChannel::Status, api, "", body, deadline);
    };
  // Bracket pose with independent quality/map queries, all sharing one deadline.
  if (!request(1000) || !parseMap(body, map, md5) ||
    !request(1021) || !parseStatus(body, status) || !request(1004)) {return false;}
  received();
  if (!parsePose(body, s) || !request(1021) || !parseStatus(body, after_status) ||
    !request(1000) || !parseMap(body, after_map, after_md5) ||
    status != after_status || map != after_map || md5 != after_md5) {return false;}
  s.status = status; s.map = map; s.md5 = md5; out = s; return true;
}
}
