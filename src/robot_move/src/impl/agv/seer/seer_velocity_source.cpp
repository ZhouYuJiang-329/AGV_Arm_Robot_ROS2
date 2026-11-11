#include "robot_move/impl/agv/seer/seer_velocity_source.hpp"
#include "robot_move/interface/cJSON.h"
#include <cmath>
#include <utility>
namespace robot_move::seer
{
SeerVelocitySource::SeerVelocitySource(std::string address)
: address_(std::move(address)) {}
bool SeerVelocitySource::connect() {return transport_.connect(address_, {ApiChannel::Status});}
void SeerVelocitySource::disconnect() {transport_.disconnect();}
bool SeerVelocitySource::parse(const std::string & body, Velocity & value)
{
  if (body.find('\0') != std::string::npos) {return false;}
  std::unique_ptr<cJSON, decltype(& cJSON_Delete)> json(
    cJSON_ParseWithOpts(body.c_str(), nullptr, true), cJSON_Delete);
  if (!cJSON_IsObject(json.get())) {return false;}
  const auto * code = cJSON_GetObjectItemCaseSensitive(json.get(), "ret_code");
  if (code && (!cJSON_IsNumber(code) || code->valuedouble != 0)) {return false;}
  Velocity candidate;
  const char * keys[] = {"vx", "vy", "w"};
  double * values[] = {&candidate.vx, &candidate.vy, &candidate.w};
  for (int i = 0; i < 3; ++i) {
    const auto * field = cJSON_GetObjectItemCaseSensitive(json.get(), keys[i]);
    if (!cJSON_IsNumber(field) || !std::isfinite(field->valuedouble)) {return false;}
    *values[i] = field->valuedouble;
  }
  value = candidate;
  return true;
}
bool SeerVelocitySource::read(Velocity & value, Deadline deadline)
{
  std::string body;
  if (!transport_.request(ApiChannel::Status, 1005, "", body, deadline)) {return false;}
  if (parse(body, value)) {return true;}
    // Fail closed; reconnect after any invalid response to avoid stale state reuse.
  transport_.disconnect();
  return false;
}
}
