#include <cassert>
#include <string>

#include "colmag_ros/json_escape.h"

int main()
{
  using colmag_ros::jsonEscape;

  assert(jsonEscape("") == "");
  assert(jsonEscape("fr3-hardware-active") == "fr3-hardware-active");
  assert(jsonEscape("say \"hi\"") == "say \\\"hi\\\"");
  assert(jsonEscape("a\\b") == "a\\\\b");
  assert(jsonEscape("line1\nline2") == "line1\\nline2");
  assert(jsonEscape("cr\r") == "cr\\r");
  assert(jsonEscape("tab\there") == "tab\\there");
  return 0;
}
