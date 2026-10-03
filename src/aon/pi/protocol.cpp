#include "../../../include/aon/pi/protocol.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>

// Portable on purpose: no PROS headers here (see protocol.hpp).

namespace aon::pi {

// ============================================================================
// Framing helpers
// ============================================================================

std::uint8_t checksum(const std::string &body) {
  std::uint8_t sum = 0;
  for (char c : body) sum ^= static_cast<std::uint8_t>(c);
  return sum;
}

std::string withChecksum(const std::string &body) {
  char suffix[4];
  std::snprintf(suffix, sizeof(suffix), "*%02X", checksum(body));
  return body + suffix;
}

static int hexValue(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

bool verifyChecksum(const std::string &line, std::string &body, bool &hadChecksum) {
  const std::size_t star = line.rfind('*');
  hadChecksum = star != std::string::npos && star + 3 == line.size();
  if (!hadChecksum) {
    body = line;
    return false;
  }
  body = line.substr(0, star);
  const int hi = hexValue(line[star + 1]);
  const int lo = hexValue(line[star + 2]);
  if (hi < 0 || lo < 0) return false;
  return checksum(body) == static_cast<std::uint8_t>(hi * 16 + lo);
}

std::string sanitize(const std::string &value) {
  std::string out = value;
  for (char &c : out) {
    if (c == ',' || c == ';' || c == '=' || c == '*' || c == '\r' || c == '\n' ||
        static_cast<unsigned char>(c) < 0x20 || static_cast<unsigned char>(c) > 0x7E) {
      c = '_';
    }
  }
  return out;
}

static std::vector<std::string> split(const std::string &text, char separator) {
  std::vector<std::string> parts;
  std::size_t start = 0;
  while (true) {
    const std::size_t end = text.find(separator, start);
    parts.push_back(text.substr(start, end == std::string::npos ? std::string::npos : end - start));
    if (end == std::string::npos) break;
    start = end + 1;
  }
  return parts;
}

static bool parseLong(const std::string &text, long &out) {
  if (text.empty()) return false;
  char *end = nullptr;
  const long value = std::strtol(text.c_str(), &end, 10);
  if (end != text.c_str() + text.size()) return false;
  out = value;
  return true;
}

// ============================================================================
// KV
// ============================================================================

void KV::append(const std::string &key, const std::string &value) {
  if (!fields.empty()) fields += ';';
  fields += sanitize(key);
  fields += '=';
  fields += sanitize(value);
}

KV &KV::add(const std::string &key, double value, int decimals) {
  if (!std::isfinite(value)) {
    append(key, "nan");
    return *this;
  }
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.*f", decimals, value);
  append(key, buffer);
  return *this;
}

KV &KV::add(const std::string &key, int value) { append(key, std::to_string(value)); return *this; }
KV &KV::add(const std::string &key, long value) { append(key, std::to_string(value)); return *this; }
KV &KV::add(const std::string &key, unsigned long value) { append(key, std::to_string(value)); return *this; }
KV &KV::add(const std::string &key, bool value) { append(key, value ? "1" : "0"); return *this; }
KV &KV::add(const std::string &key, const std::string &value) { append(key, value); return *this; }
KV &KV::add(const std::string &key, const char *value) { append(key, value ? value : ""); return *this; }

KV &KV::merge(const KV &other, const std::string &prefix) {
  if (other.fields.empty()) return *this;
  if (prefix.empty()) {
    if (!fields.empty()) fields += ';';
    fields += other.fields;
    return *this;
  }
  for (const std::string &pair : split(other.fields, ';')) {
    if (!fields.empty()) fields += ';';
    fields += sanitize(prefix) + "." + pair;
  }
  return *this;
}

// ============================================================================
// Args / Status
// ============================================================================

bool Args::number(std::size_t i, double &out) const {
  if (i >= values.size() || values[i].empty()) return false;
  char *end = nullptr;
  const double value = std::strtod(values[i].c_str(), &end);
  if (end != values[i].c_str() + values[i].size() || !std::isfinite(value)) return false;
  out = value;
  return true;
}

const char *statusName(Status status) {
  switch (status) {
    case Status::OK: return "ok";
    case Status::ERR: return "err";
    case Status::ABORTED: return "aborted";
    case Status::TIMEOUT: return "timeout";
  }
  return "err";
}

// ============================================================================
// Link
// ============================================================================

Link::Link(Hooks hooks, Config config) : hooks(std::move(hooks)), config(config) {}

void Link::registerCommand(const std::string &verb, Kind kind, int minArgs, int maxArgs, Handler fn) {
  commands.push_back({verb, kind, minArgs, maxArgs, std::move(fn)});
}

void Link::registerSensor(const std::string &name, SensorFn fn) {
  sensors.push_back({name, std::move(fn)});
}

void Link::registerPacket(char tag, std::function<void(const std::vector<std::string> &)> fn) {
  PacketSlot slot;
  slot.tag = tag;
  slot.fn = std::move(fn);
  packets.push_back(std::move(slot));
}

bool Link::latestPacket(char tag, std::vector<std::string> &fields, std::uint32_t &ageMs) const {
  Guard guard(hooks);
  for (const PacketSlot &slot : packets) {
    if (slot.tag != tag || !slot.seen) continue;
    fields = slot.fields;
    ageMs = hooks.millis() - slot.atMs;
    return true;
  }
  return false;
}

void Link::handlePacket(const std::string &line) {
  // Checksum optional (vexpi and red_tracker send none); if present, check it.
  std::string body;
  bool hadChecksum = false;
  const bool valid = verifyChecksum(line, body, hadChecksum);
  std::vector<std::string> parts = split(body, ',');
  const char tag = line[0];

  PacketSlot *slot = nullptr;
  for (PacketSlot &candidate : packets) {
    if (candidate.tag == tag) slot = &candidate;
  }
  {
    Guard guard(hooks);
    if (hadChecksum && !valid) {
      counters.badChecksum++;
      return;
    }
    if (slot == nullptr) {
      counters.unknownPackets++;
      return;
    }
    counters.packets++;
    parts.erase(parts.begin());
    slot->seen = true;
    slot->fields = parts;
    slot->atMs = hooks.millis();
  }
  if (slot->fn) slot->fn(parts);
}

const Link::Command *Link::find(const std::string &verb) const {
  for (const Command &command : commands) {
    if (command.verb == verb) return &command;
  }
  return nullptr;
}

void Link::send(const std::string &body) {
  if (hooks.writeLine) hooks.writeLine(withChecksum(body) + "\n");
}

void Link::ack(long seq) { send("@A," + std::to_string(seq)); }

void Link::reply(long seq, const Result &result) {
  const std::string head = "@D," + std::to_string(seq) + "," + statusName(result.status);
  std::string fields = result.fields.str();

  // Split long replies into @P lines of whole k=v pairs, then the @D.
  const std::string partial = "@P," + std::to_string(seq) + ",";
  while (head.size() + 1 + fields.size() > MAX_TX_BODY) {
    std::size_t cut = fields.rfind(';', MAX_TX_BODY - partial.size());
    if (cut == std::string::npos || cut == 0) cut = fields.find(';');  // one huge pair: send it alone
    if (cut == std::string::npos) break;
    send(partial + fields.substr(0, cut));
    fields.erase(0, cut + 1);
  }
  send(fields.empty() ? head : head + "," + fields);
}

void Link::event(const std::string &code, const KV &fields) {
  std::string body = "@E," + sanitize(code);
  if (!fields.empty()) body += "," + fields.str();
  send(body);
  if (hooks.log) hooks.log("[PI] " + code + (fields.empty() ? "" : " " + fields.str()));
}

void Link::feed(char c) {
  if (c == '\r') return;
  if (c == '\n') {
    if (rxOverflow) {
      {
        Guard guard(hooks);
        counters.tooLong++;
      }
      reply(0, Result::error("too_long", "line longer than " + std::to_string(MAX_RX_LINE) + " bytes was dropped"));
    } else if (!rxBuffer.empty()) {
      handleLine(rxBuffer);
    }
    rxBuffer.clear();
    rxOverflow = false;
    return;
  }
  if (rxBuffer.size() >= MAX_RX_LINE) {
    rxOverflow = true;
    return;
  }
  rxBuffer.push_back(c);
}

void Link::handleLine(const std::string &line) {
  // Sensor packets from other Pi programs sharing the port (vexpi's OTOS
  // stream at 50 Hz, red_tracker): never answered, never feed the deadman.
  if (line.size() > 1 && line[0] >= 'A' && line[0] <= 'Z' && line[0] != 'C' && line[1] == ',') {
    handlePacket(line);
    return;
  }
  // Not a command either (e.g. depth_center_demo's untagged "123,0"):
  // count it, but don't answer; a stream of these must not cause traffic.
  if (line.size() < 2 || line[0] != 'C' || line[1] != ',') {
    Guard guard(hooks);
    counters.ignoredLines++;
    return;
  }

  // A bridge command: this is what keeps the link (and the deadman) alive.
  {
    Guard guard(hooks);
    everRx = true;
    lastRxMs = hooks.millis();
    counters.rxLines++;
  }

  std::string body;
  bool hadChecksum = false;
  const bool checksumOk = verifyChecksum(line, body, hadChecksum);
  const std::vector<std::string> parts = split(body, ',');

  long seq = 0;
  const bool seqOk = parts.size() >= 2 && parseLong(parts[1], seq) && seq > 0;

  if (!checksumOk) {
    {
      Guard guard(hooks);
      counters.badChecksum++;
    }
    reply(seqOk ? seq : 0, Result::error("bad_checksum", hadChecksum ? "checksum mismatch, line ignored"
                                                                    : "missing *HH checksum, line ignored"));
    return;
  }
  if (!seqOk || parts.size() < 3) {
    reply(0, Result::error("bad_seq", "expected C,<seq>,<VERB> with seq > 0"));
    return;
  }

  const std::string &verb = parts[2];
  const Command *command = find(verb);
  if (command == nullptr) {
    {
      Guard guard(hooks);
      counters.unknownVerb++;
    }
    std::string known;
    for (const Command &c : commands) known += (known.empty() ? "" : " ") + c.verb;
    reply(seq, Result::error("unknown_verb", "unknown verb " + verb + "; known: " + known));
    return;
  }

  std::vector<std::string> args(parts.begin() + 3, parts.end());
  const int argc = static_cast<int>(args.size());
  if (argc < command->minArgs || argc > command->maxArgs) {
    reply(seq, Result::error("bad_args", verb + " takes " + std::to_string(command->minArgs) + " to " +
                                             std::to_string(command->maxArgs) + " arguments, got " +
                                             std::to_string(argc)));
    return;
  }

  if (command->kind == Kind::IMMEDIATE) {
    reply(seq, command->fn(Args(std::move(args))));
    return;
  }

  // MOTION: one at a time, queued to the worker task.
  std::string busyWith;
  long busySeq = 0;
  {
    Guard guard(hooks);
    if (pending.valid || running) {
      busyWith = pending.valid ? pending.verb : runningVerb;
      busySeq = pending.valid ? pending.seq : runningSeq;
    } else {
      pending.valid = true;
      pending.seq = seq;
      pending.verb = verb;
      pending.args = std::move(args);
      control = true;
    }
  }
  if (!busyWith.empty()) {
    reply(seq, Result::error("busy", "already running " + busyWith + " (seq " + std::to_string(busySeq) +
                                         "); wait for it or send STOP"));
    return;
  }
  ack(seq);
}

bool Link::runPendingMotion() {
  Pending job;
  std::string abortedBeforeStart;
  {
    Guard guard(hooks);
    if (!pending.valid) return false;
    job = pending;
    pending = Pending();
    if (!abortReason.empty()) {
      // STOP / deadman arrived before the worker picked the job up.
      abortedBeforeStart = abortReason;
      abortReason.clear();
      control = false;
    } else {
      running = true;
      runningSeq = job.seq;
      runningVerb = job.verb;
      if (hooks.clearAbort) hooks.clearAbort();
    }
  }

  if (!abortedBeforeStart.empty()) {
    Result result{Status::ABORTED, KV()};
    result.fields.add("reason", abortedBeforeStart).add("started", false);
    reply(job.seq, result);
    return true;
  }

  const Command *command = find(job.verb);
  const std::uint32_t start = hooks.millis();
  Result result = command ? command->fn(Args(job.args)) : Result::error("unknown_verb", job.verb);
  const std::uint32_t duration = hooks.millis() - start;

  std::string reason;
  {
    Guard guard(hooks);
    reason = abortReason;
    abortReason.clear();
    running = false;
    runningSeq = 0;
    runningVerb.clear();
    // Leave the drivetrain's abort flag clear so autons and driver code that
    // run afterwards are never cut short by a stale Pi abort.
    if (hooks.clearAbort) hooks.clearAbort();
    control = false;
  }

  if (!reason.empty() && result.status != Status::ERR) {
    result.status = Status::ABORTED;
    result.fields.add("reason", reason);
  }
  result.fields.add("dur_ms", static_cast<unsigned long>(duration));
  reply(job.seq, result);
  return true;
}

void Link::abort(const std::string &reason) {
  bool aborted = false;
  {
    Guard guard(hooks);
    if ((pending.valid || running) && abortReason.empty()) {
      abortReason = reason;
      counters.aborts++;
      if (hooks.abortMotion) hooks.abortMotion();
      aborted = true;
    }
  }
  if (aborted) {
    event("aborted", KV().add("reason", reason).add("msg", "Pi motion aborted - brain has control"));
  }
}

void Link::tick() {
  const std::uint32_t now = hooks.millis();
  bool tripped = false;
  bool sendHeartbeat = false;
  bool linkWentUp = false;
  bool linkWentDown = false;
  std::uint32_t silentMs = 0;
  {
    Guard guard(hooks);
    silentMs = now - lastRxMs;
    const bool active = everRx && silentMs < config.linkIdleMs;
    linkWentUp = active && !linkActive;
    linkWentDown = !active && linkActive;
    linkActive = active;

    if ((pending.valid || running) && everRx && silentMs > config.deadmanMs && abortReason.empty()) {
      abortReason = "link_lost";
      counters.aborts++;
      counters.deadmanTrips++;
      if (hooks.abortMotion) hooks.abortMotion();
      tripped = true;
    }
    if (linkActive && now - lastHeartbeatMs >= config.heartbeatMs) {
      lastHeartbeatMs = now;
      sendHeartbeat = true;
    }
  }

  if (linkWentUp && hooks.log) hooks.log("[PI] link up");
  if (linkWentDown && hooks.log) hooks.log("[PI] link idle: no data from the Pi for " + std::to_string(silentMs) + " ms");
  if (tripped) {
    event("link_lost", KV()
                           .add("silent_ms", static_cast<unsigned long>(silentMs))
                           .add("msg", "no data from the Pi - motion aborted and brain has control"));
  }
  if (sendHeartbeat) {
    KV kv;
    kv.add("up", static_cast<unsigned long>(now)).add("pi", hasControl()).add("busy", busyVerb());
    if (hooks.heartbeatFields) hooks.heartbeatFields(kv);
    send("@H," + kv.str());
  }
}

bool Link::readSensors(KV &out, const std::string &name) {
  bool found = false;
  for (const Sensor &sensor : sensors) {
    if (!name.empty() && sensor.name != name) continue;
    KV fields;
    sensor.fn(fields);
    out.merge(fields, sensor.name);
    found = true;
  }
  return found;
}

std::vector<std::string> Link::sensorNames() const {
  std::vector<std::string> names;
  for (const Sensor &sensor : sensors) names.push_back(sensor.name);
  return names;
}

Link::Stats Link::stats() const {
  Guard guard(hooks);
  return counters;
}

long Link::msSinceLastRx() const {
  Guard guard(hooks);
  if (!everRx) return -1;
  return static_cast<long>(hooks.millis() - lastRxMs);
}

std::string Link::busyVerb() const {
  Guard guard(hooks);
  if (running) return runningVerb;
  if (pending.valid) return pending.verb;
  return "-";
}

}  // namespace aon::pi
