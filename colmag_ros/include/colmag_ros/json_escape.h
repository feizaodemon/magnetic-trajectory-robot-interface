#pragma once

#include <sstream>
#include <string>

namespace colmag_ros
{

// Escapes a value for a JSON string literal in the adapters' status payloads.
inline std::string jsonEscape(const std::string& value)
{
  std::ostringstream out;
  for (const char ch : value)
  {
    switch (ch)
    {
      case '\\':
        out << "\\\\";
        break;
      case '"':
        out << "\\\"";
        break;
      case '\n':
        out << "\\n";
        break;
      case '\r':
        out << "\\r";
        break;
      case '\t':
        out << "\\t";
        break;
      default:
        out << ch;
        break;
    }
  }
  return out.str();
}

}  // namespace colmag_ros
