#include "motionslam_runtime/plugin_registry.hpp"

#include <cassert>

using motionslam::runtime::PluginAuthority;
using motionslam::runtime::PluginDescriptor;
using motionslam::runtime::PluginKind;
using motionslam::runtime::PluginLiveness;
using motionslam::runtime::PluginRegistry;
using motionslam::runtime::RegisterResult;

namespace {

PluginDescriptor make_plugin(const char* id, PluginKind kind,
                             PluginAuthority authority) {
  PluginDescriptor d;
  d.plugin_id = id;
  d.name = id;
  d.interface_version = "v1";
  d.kind = kind;
  d.authority = authority;
  return d;
}

void test_register_and_lookup() {
  PluginRegistry reg;
  auto contract = make_plugin("fsr-contract", PluginKind::kContract,
                              PluginAuthority::kEvidence);
  contract.provides = {"spatial-state", "reason-codes"};
  assert(reg.add(contract) == RegisterResult::kRegistered);
  assert(reg.find("fsr-contract") != nullptr);
  assert(reg.providers("spatial-state").size() == 1);
  assert(reg.providers("nonexistent").empty());
  // duplicate id rejected
  assert(reg.add(contract) == RegisterResult::kDuplicateId);
}

void test_requires_must_resolve() {
  PluginRegistry reg;
  auto token = make_plugin("fsr-token", PluginKind::kToken,
                           PluginAuthority::kEvidence);
  token.needs = {"spatial-state"};
  // contract not yet registered -> requires unresolved
  assert(reg.add(token) == RegisterResult::kUnresolvedRequires);
  assert(reg.find("fsr-token") == nullptr);

  auto contract = make_plugin("fsr-contract", PluginKind::kContract,
                              PluginAuthority::kEvidence);
  contract.provides = {"spatial-state"};
  assert(reg.add(contract) == RegisterResult::kRegistered);
  assert(reg.add(token) == RegisterResult::kRegistered);
}

void test_invalid_descriptor_rejected() {
  PluginRegistry reg;
  PluginDescriptor bad;  // empty plugin_id / name / interface_version
  assert(reg.add(bad) == RegisterResult::kInvalidDescriptor);

  auto neg = make_plugin("p", PluginKind::kCustom, PluginAuthority::kEvidence);
  neg.health_interval_ns = -1;
  assert(reg.add(neg) == RegisterResult::kInvalidDescriptor);
}

void test_remove_recomputes_provided() {
  PluginRegistry reg;
  auto a = make_plugin("a", PluginKind::kSource, PluginAuthority::kEvidence);
  a.provides = {"pose"};
  auto b = make_plugin("b", PluginKind::kRecovery, PluginAuthority::kAdvice);
  b.provides = {"recovery"};
  b.needs = {"pose"};
  assert(reg.add(a) == RegisterResult::kRegistered);
  assert(reg.add(b) == RegisterResult::kRegistered);

  assert(reg.remove("a"));
  assert(!reg.remove("a"));
  assert(reg.providers("pose").empty());
  // after removal, a new plugin requiring "pose" can no longer register
  auto c = make_plugin("c", PluginKind::kCustom, PluginAuthority::kEvidence);
  c.needs = {"pose"};
  assert(reg.add(c) == RegisterResult::kUnresolvedRequires);
}

void test_liveness() {
  PluginRegistry reg;
  auto no_health = make_plugin("nh", PluginKind::kMemory,
                               PluginAuthority::kEvidence);
  assert(reg.add(no_health) == RegisterResult::kRegistered);
  // no health interval -> always live
  assert(reg.liveness("nh", 1'000'000'000) == PluginLiveness::kLive);

  auto monitored = make_plugin("m", PluginKind::kSource,
                               PluginAuthority::kEvidence);
  monitored.health_interval_ns = 100;
  assert(reg.add(monitored) == RegisterResult::kRegistered);
  // registered but no report yet -> stale
  assert(reg.liveness("m", 0) == PluginLiveness::kStale);

  reg.report_health("m", true, "ok", 1000);
  assert(reg.liveness("m", 1000) == PluginLiveness::kLive);
  assert(reg.liveness("m", 1200) == PluginLiveness::kLive);
  // beyond 2x interval -> stale
  assert(reg.liveness("m", 1201) == PluginLiveness::kStale);

  // failed report -> stale immediately
  reg.report_health("m", false, "lidar timeout", 2000);
  assert(reg.liveness("m", 2000) == PluginLiveness::kStale);

  assert(reg.liveness("ghost", 0) == PluginLiveness::kUnregistered);
}

void test_descriptor_pointer_stable_across_add() {
  PluginRegistry reg;
  auto a = make_plugin("a", PluginKind::kSource, PluginAuthority::kEvidence);
  a.provides = {"pose"};
  assert(reg.add(a) == RegisterResult::kRegistered);
  const PluginDescriptor* saved = reg.find("a");
  // register many more plugins; the saved pointer must stay valid
  for (int i = 0; i < 50; ++i) {
    auto extra = make_plugin((std::string("x") + std::to_string(i)).c_str(),
                             PluginKind::kCustom, PluginAuthority::kEvidence);
    assert(reg.add(extra) == RegisterResult::kRegistered);
  }
  assert(saved == reg.find("a"));
  assert(saved->plugin_id == "a");
  // removing a different plugin keeps it valid too
  assert(reg.remove("x0"));
  assert(saved->plugin_id == "a");
}

void test_late_report_does_not_revive() {
  PluginRegistry reg;
  auto m = make_plugin("m", PluginKind::kSource, PluginAuthority::kEvidence);
  m.health_interval_ns = 100;
  assert(reg.add(m) == RegisterResult::kRegistered);
  assert(reg.remove("m"));
  // late heartbeat for the removed instance is ignored
  reg.report_health("m", true, "late ok", 1000);
  assert(reg.add(m) == RegisterResult::kRegistered);
  // replacement has not reported yet -> stale, not live
  assert(reg.liveness("m", 1001) == PluginLiveness::kStale);
  // reports before registration are ignored entirely
  PluginRegistry reg2;
  reg2.report_health("m", true, "early", 500);
  auto m2 = make_plugin("m", PluginKind::kSource, PluginAuthority::kEvidence);
  m2.health_interval_ns = 100;
  assert(reg2.add(m2) == RegisterResult::kRegistered);
  assert(reg2.liveness("m", 501) == PluginLiveness::kStale);
}

void test_enumeration_follows_registration_order() {
  PluginRegistry reg;
  auto z = make_plugin("z-camera", PluginKind::kSource,
                       PluginAuthority::kEvidence);
  z.provides = {"pose"};
  auto a = make_plugin("a-camera", PluginKind::kSource,
                       PluginAuthority::kEvidence);
  a.provides = {"pose"};
  assert(reg.add(z) == RegisterResult::kRegistered);
  assert(reg.add(a) == RegisterResult::kRegistered);
  // registration order, not id sort
  assert(reg.providers("pose").front()->plugin_id == "z-camera");
  assert(reg.plugins().front()->plugin_id == "z-camera");
  assert(reg.remove("z-camera"));
  assert(reg.providers("pose").front()->plugin_id == "a-camera");
}

}  // namespace

int main() {
  test_register_and_lookup();
  test_requires_must_resolve();
  test_invalid_descriptor_rejected();
  test_remove_recomputes_provided();
  test_liveness();
  test_descriptor_pointer_stable_across_add();
  test_late_report_does_not_revive();
  test_enumeration_follows_registration_order();
  return 0;
}
