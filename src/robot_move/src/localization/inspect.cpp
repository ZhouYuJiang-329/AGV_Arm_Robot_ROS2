#include "robot_move/impl/agv/seer/seer_localization_source.hpp"
#include "robot_move/interface/cJSON.h"
#include <iostream>
int main(int argc, char ** argv)
{
  if (argc != 2) {std::cerr << "Usage: seer_localization_inspect controller_ipv4\n"; return 1;}
  robot_move::seer::SeerLocalizationSource source(argv[1]);
  robot_move::localization::Sample s;
  if (!source.connect() || !source.read(s,
    std::chrono::steady_clock::now() + std::chrono::seconds(1), [] {}))
  {
    std::cerr << "Read-only status/map/pose query failed\n"; return 2;
  }
  cJSON * j = cJSON_CreateObject();
  cJSON_AddStringToObject(j, "current_map", s.map.c_str());
  cJSON_AddStringToObject(j, "current_map_md5", s.md5.c_str());
  cJSON_AddNumberToObject(j, "reloc_status", s.status);
  cJSON_AddNumberToObject(j, "x", s.pose.x); cJSON_AddNumberToObject(j, "y", s.pose.y);
  cJSON_AddNumberToObject(j, "angle", s.pose.angle);
  if (s.confidence) {cJSON_AddNumberToObject(j, "confidence", *s.confidence);}
  if (s.method) {cJSON_AddNumberToObject(j, "loc_method", *s.method);}
  char * text = cJSON_Print(j); std::cout << text << '\n'; cJSON_free(text); cJSON_Delete(j);
}
