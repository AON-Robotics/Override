#define _CRT_SECURE_NO_WARNINGS
#include <cassert>
#include <cstdio>
#include <fstream>
#include <sstream>
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable: 4458)
#endif
#include "aon/tools/path-trace.hpp"
#ifdef _MSC_VER
#pragma warning(pop)
#endif

int main() {
  const char* base = "path-trace-test";
  std::remove("path-trace-test.csv");
  std::remove("path-trace-test-runs.csv");
  aon::PathTrace trace(123,2);
  aon::FollowSample sample;
  sample.pose = {3,4,20};
  sample.target = {10,0,30};
  sample.left = 100;
  sample.right = 80;
  trace.record(sample,100,1);
  trace.record(sample,200,2);
  trace.record(sample,300,2); // bounded buffer reports truncation
  assert(trace.save(base,"Timed out",sample.pose,{0,0,30},400));
  std::ifstream summary("path-trace-test-runs.csv");
  std::string header,row;
  std::getline(summary,header);
  std::getline(summary,row);
  assert(row.find("Timed out,400,5.000,10.000,2,1") != std::string::npos);
  std::ifstream csv("path-trace-test.csv");
  int lines = 0;
  while (std::getline(csv,row)) ++lines;
  assert(lines == 3);
  assert(!trace.save("missing-directory/path","Completed",{},{},0));
  summary.close(); csv.close();
  std::remove("path-trace-test.csv");
  std::remove("path-trace-test-runs.csv");
}
