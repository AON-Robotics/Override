#pragma once

#include <cmath>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <set>
#include <string>
#include <string_view>
#include <vector>
#include "../math/pose.hpp"
#include "../tools/json.hpp"

namespace aon::jerryio {

/// Only the verified historical sampled export is supported, not every JerryIO format.
enum class Export { V011LemLib05GpsInches };
/// Bare sampled data is accepted only with the caller's explicit historical declaration.
enum class Metadata { Required, ExplicitDeclaration };
/// Ghost handling is opt-in and requires the verified full editor export.
enum class Endpoint { SampleRows, V011GhostTail };
enum class Error {
  None, InvalidOptions, EmptyInput, TooLarge, MalformedRow, NonFinite,
  SpeedOutOfRange, InteriorZero, MissingTerminator, TooFewPoints,
  MetadataMissing, UnsupportedMetadata, MalformedMetadata, CoordinateRange
};

/// Explicit isometry from exported GPS-inch XY to AON odometry-inch XY.
/// xA=originX+xx*x+xy*y; yA=originY+yx*x+yy*y. Columns must be orthonormal;
/// reflection is allowed. No implicit start anchoring or heading conversion.
struct Frame { double originX, originY, xx, xy, yx, yy; };
struct Options {
  Export format;
  Metadata metadata;
  Endpoint endpoint;
  Frame frame;
  double maximumRPM;  ///< Explicit normalized-speed127 ceiling, in motor RPM.
  double finalHeading; ///< Explicit AON clockwise degrees, independent of editor headings.
};
struct Result {
  std::vector<Pose> points;
  std::vector<double> caps; ///< Center-speed RPM ceilings, never theta or wheel RPM.
  Error error = Error::None;
  std::size_t line = 0;
  explicit operator bool() const { return error == Error::None; }
};

namespace detail {
inline std::string_view trim(std::string_view s) {
  while (!s.empty() && (s.front()==' '||s.front()=='\t'||s.front()=='\r')) s.remove_prefix(1);
  while (!s.empty() && (s.back()==' '||s.back()=='\t'||s.back()=='\r')) s.remove_suffix(1);
  return s;
}
// Decimal grammar only: no hex, NaN, infinity, embedded NUL, or locale separators.
inline bool number(std::string_view text, double& out) {
  text=trim(text);
  if (text.empty()||text.size()>127) return false;
  std::size_t i=0, digits=0;
  if (text[i]=='+'||text[i]=='-') ++i;
  while(i<text.size()&&text[i]>='0'&&text[i]<='9'){++i;++digits;}
  if(i<text.size()&&text[i]=='.') {
    ++i;while(i<text.size()&&text[i]>='0'&&text[i]<='9'){++i;++digits;}
  }
  if(!digits) return false;
  if(i<text.size()&&(text[i]=='e'||text[i]=='E')) {
    ++i;if(i<text.size()&&(text[i]=='+'||text[i]=='-'))++i;
    const auto first=i;while(i<text.size()&&text[i]>='0'&&text[i]<='9')++i;
    if(i==first)return false;
  }
  if(i!=text.size())return false;
  char buffer[128];std::memcpy(buffer,text.data(),text.size());buffer[text.size()]='\0';
  char* end=nullptr;errno=0;out=std::strtod(buffer,&end);
  return end==buffer+text.size()&&errno!=ERANGE&&std::isfinite(out);
}
inline bool row(std::string_view line, double* values, std::size_t count) {
  for(std::size_t i=0;i<count;++i) {
    auto comma=line.find(',');
    if((i+1<count)==(comma==std::string_view::npos))return false;
    if(!number(line.substr(0,comma),values[i]))return false;
    if(comma!=std::string_view::npos)line.remove_prefix(comma+1);
  }
  return true;
}
inline Error metadata(std::string_view text, Pose* endpoint) {
  // The existing JSON lexer treats a raw NUL as end-of-input. Reject it so
  // trailing bytes cannot evade validation of the complete metadata record.
  if(text.find('\0')!=std::string_view::npos)return Error::MalformedMetadata;
  // Bound parser depth before constructing the existing JSON DOM. Preparation
  // may allocate; no JSON, strings or parser state enter PurePursuit::update.
  int depth=0;bool quoted=false,escape=false;
  for(char c:text) {
    if(quoted){if(escape)escape=false;else if(c=='\\')escape=true;else if(c=='"')quoted=false;}
    else if(c=='"')quoted=true;
    else if(c=='{'||c=='['){if(++depth>32)return Error::MalformedMetadata;}
    else if(c=='}'||c==']'){if(--depth<0)return Error::MalformedMetadata;}
  }
  if(quoted||depth)return Error::MalformedMetadata;
  bool duplicate=false;
  std::vector<std::set<std::string>> keys;
  const auto callback=[&](int, nlohmann::json::parse_event_t event,nlohmann::json& parsed) {
    if(event==nlohmann::json::parse_event_t::object_start)keys.emplace_back();
    else if(event==nlohmann::json::parse_event_t::key) {
      if(keys.empty()||!keys.back().insert(parsed.get<std::string>()).second)duplicate=true;
    } else if(event==nlohmann::json::parse_event_t::object_end) {
      if(!keys.empty())keys.pop_back();
    }
    return true;
  };
  const auto j=nlohmann::json::parse(text.begin(),text.end(),callback,false);
  if(j.is_discarded()||duplicate||!j.is_object())return Error::MalformedMetadata;
  if(!j.contains("appVersion")||j["appVersion"]!="0.11.0"||
     !j.contains("format")||j["format"]!="LemLib v0.5"||
     !j.contains("gc")||!j["gc"].is_object()||
     !j.contains("paths")||!j["paths"].is_array()||j["paths"].size()!=1)
    return Error::UnsupportedMetadata;
  const auto& gc=j["gc"];
  if(!gc.contains("coordinateSystem")||gc["coordinateSystem"]!="VEX Gaming Positioning System"||
     !gc.contains("uol")||!gc["uol"].is_number()||gc["uol"].get<double>()!=2.54||
     !gc.contains("robotIsHolonomic")||!gc["robotIsHolonomic"].is_boolean()||
     gc["robotIsHolonomic"].get<bool>())return Error::UnsupportedMetadata;
  if(endpoint) {
    const auto& path=j["paths"][0];
    if(!path.is_object()||!path.contains("segments")||!path["segments"].is_array()||path["segments"].empty())return Error::UnsupportedMetadata;
    const auto& last=path["segments"].back();
    if(!last.is_object()||!last.contains("controls")||!last["controls"].is_array()||last["controls"].empty())return Error::UnsupportedMetadata;
    const auto& control=last["controls"].back();
    if(!control.is_object()||!control.contains("x")||!control["x"].is_number()||
       !control.contains("y")||!control["y"].is_number())return Error::UnsupportedMetadata;
    *endpoint=Pose(control["x"].get<double>(),control["y"].get<double>(),0);
    if(!std::isfinite(endpoint->x)||!std::isfinite(endpoint->y))return Error::MalformedMetadata;
  }
  return Error::None;
}
} // namespace detail

/// Decode and adapt once before movement. At most4096 ordered rows/1MiB input.
/// Decimal x,y,speed rows end at endData. Optional legacy editor footer consists
/// of three scalar lines and up to4096 eight-number control rows, then metadata.
/// Footer/control headings never command motion. Metadata is validated when
/// present even under ExplicitDeclaration. All failures return empty vectors.
/// Coordinates are the historical exported inches (uol2.54 is editor metadata,
/// not another scale factor). Speed caps = speed/127*maximumRPM, retaining doubles.
/// Interior zero speeds are unsupported stops and rejected; terminal zero is
/// allowed. V011GhostTail explicitly removes ONLY the verified duplicate-zero
/// endpoint and20-inch ghost, requiring full metadata/legacy footer and matching
/// end control. SampleRows never trims. Remaining duplicates/order stay intact for
/// PurePursuit::setPath to coalesce with the strictest cap. There is no LemLib
/// dependency, guessed frame, inferred final heading, or automatic robot action.
inline Result decode(std::string_view input,const Options& options) {
  const auto bad=[](Error e,std::size_t line=0){return Result{{},{},e,line};};
  const auto& f=options.frame;
  const double values[]={f.originX,f.originY,f.xx,f.xy,f.yx,f.yy,options.maximumRPM,options.finalHeading};
  for(double v:values)if(!std::isfinite(v))return bad(Error::InvalidOptions);
  if(options.format!=Export::V011LemLib05GpsInches||
     (options.metadata!=Metadata::Required&&options.metadata!=Metadata::ExplicitDeclaration)||
     (options.endpoint!=Endpoint::SampleRows&&options.endpoint!=Endpoint::V011GhostTail)||
     options.maximumRPM<=0||options.maximumRPM>32767||
     std::abs(f.xx*f.xx+f.yx*f.yx-1)>1e-12||
     std::abs(f.xy*f.xy+f.yy*f.yy-1)>1e-12||std::abs(f.xx*f.xy+f.yx*f.yy)>1e-12)
    return bad(Error::InvalidOptions);
  if(input.empty())return bad(Error::EmptyInput);
  if(input.size()>1024*1024)return bad(Error::TooLarge);
  Result result;bool ended=false,hasMetadata=false;std::size_t lineNumber=0,offset=0,footer=0;
  Pose metadataEndpoint,footerEndpoint;
  while(offset<input.size()) {
    ++lineNumber;const auto newline=input.find('\n',offset);
    const auto end=newline==std::string_view::npos?input.size():newline;
    const auto line=detail::trim(input.substr(offset,end-offset));offset=end==input.size()?end:end+1;
    if(hasMetadata){if(!line.empty())return bad(Error::MalformedMetadata,lineNumber);continue;}
    if(!ended) {
      if(line=="endData"){ended=true;continue;}
      if(result.points.size()==4096)return bad(Error::TooLarge,lineNumber);
      double v[3];if(!detail::row(line,v,3))return bad(Error::MalformedRow,lineNumber);
      if(v[2]<0||v[2]>127)return bad(Error::SpeedOutOfRange,lineNumber);
      const double x=f.originX+f.xx*v[0]+f.xy*v[1], y=f.originY+f.yx*v[0]+f.yy*v[1];
      if(!std::isfinite(x)||!std::isfinite(y)||std::abs(x)>1e6||std::abs(y)>1e6)
        return bad(Error::CoordinateRange,lineNumber);
      result.points.emplace_back(x,y,0);result.caps.push_back((v[2]/127)*options.maximumRPM);
    } else {
      if(line.empty())continue;
      constexpr std::string_view prefix="#PATH.JERRYIO-DATA ";
      if(line.substr(0,prefix.size())==prefix) {
        if(footer>0&&footer<4)return bad(Error::MalformedMetadata,lineNumber);
        const auto error=detail::metadata(line.substr(prefix.size()),options.endpoint==Endpoint::V011GhostTail?&metadataEndpoint:nullptr);
        if(error!=Error::None)return bad(error,lineNumber);
        hasMetadata=true;
      } else {
        double v[8];if(footer>=4099||!detail::row(line,v,footer<3?1:8))return bad(Error::MalformedMetadata,lineNumber);
        if((footer==0&&v[0]<=0)||(footer==1&&(v[0]<0||v[0]>127))||(footer==2&&v[0]!=200))return bad(Error::UnsupportedMetadata,lineNumber);
        if(footer>=3)footerEndpoint=Pose(v[6],v[7],0);
        ++footer;
      }
    }
  }
  if(!ended)return bad(Error::MissingTerminator,lineNumber);
  if(result.points.size()<2)return bad(Error::TooFewPoints,lineNumber);
  if(!hasMetadata&&(options.metadata==Metadata::Required||footer))return bad(Error::MetadataMissing,lineNumber);
  if(options.endpoint==Endpoint::V011GhostTail) {
    const std::size_t n=result.points.size();
    if(!hasMetadata||footer<4||n<4)return bad(Error::UnsupportedMetadata,lineNumber);
    const auto& end=result.points[n-3];const auto& duplicate=result.points[n-2];const auto& ghost=result.points[n-1];
    const auto convert=[&](const Pose& p){return Pose(f.originX+f.xx*p.x+f.xy*p.y,f.originY+f.yx*p.x+f.yy*p.y,0);};
    const auto& prior=result.points[n-4];const double incoming=end.distanceTo(prior);
    // Exported coordinates may be rounded to0.001in. Bound the endpoint/chord
    // rounding error propagated over the20in ghost, rather than guess a heading.
    const double cross=incoming>0?std::abs((ghost.x-end.x)*(end.y-prior.y)-(ghost.y-end.y)*(end.x-prior.x))/incoming:INFINITY;
    const double dot=(ghost.x-end.x)*(end.x-prior.x)+(ghost.y-end.y)*(end.y-prior.y);
    if(result.caps[n-3]!=0||result.caps[n-2]!=0||result.caps[n-1]!=0||
       end.x!=duplicate.x||end.y!=duplicate.y||
       end.distanceTo(convert(metadataEndpoint))>0.002||end.distanceTo(convert(footerEndpoint))>0.002||
       std::abs(end.distanceTo(ghost)-20)>0.003||incoming<=0||dot<=0||cross>0.003*(1+20/incoming))
      return bad(Error::UnsupportedMetadata,lineNumber);
    result.points.resize(n-2);result.caps.resize(n-2);
  }
  for(std::size_t i=0;i+1<result.caps.size();++i)if(result.caps[i]==0)return bad(Error::InteriorZero,lineNumber);
  result.points.back().theta=options.finalHeading;
  return result;
}
} // namespace aon::jerryio
