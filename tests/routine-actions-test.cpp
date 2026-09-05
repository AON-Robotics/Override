#include "aon/jerryio/routine-actions.hpp"

#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#define CHECK(condition)                                                     \
  do {                                                                       \
    if (!(condition)) {                                                      \
      std::cerr << __FILE__ << ':' << __LINE__ << ": " << #condition       \
                << '\n';                                                     \
      std::exit(1);                                                          \
    }                                                                        \
  } while (false)

int main() {
  std::vector<std::string> events;
  const auto actions = aon::jerryio::makePathJerryIOActions(
      [&] { events.push_back("intake"); },
      [&] { events.push_back("outtake"); },
      [&] { events.push_back("stop"); });

  CHECK(actions.size() == 3);
  for (std::size_t index = 0; index < actions.size(); ++index) {
    CHECK(actions[index].markerOrdinal == index + 1);
    CHECK(actions[index].durationMs == 2000);
    CHECK(actions[index].start);
    CHECK(actions[index].cleanup);
    actions[index].start();
    actions[index].cleanup();
  }

  const std::vector<std::string> expected{
      "intake", "stop", "outtake", "stop", "intake", "stop"};
  CHECK(events == expected);
  std::cout << "JerryIO routine action tests passed\n";
}
