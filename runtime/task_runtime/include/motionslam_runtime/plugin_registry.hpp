#pragma once

#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace motionslam::runtime {

// Plugin authority boundary: a plugin's output to the BT hub is only ever a
// candidate. Plugins can never hold execution authority or bypass the BT /
// safety layer. Evidence is observational data; Advice is a suggested decision
// the BT may accept or ignore.
enum class PluginAuthority {
  kEvidence,
  kAdvice,
};

enum class PluginKind {
  kSource,     // ⑦ 源头状态提供者 (Super-LIO / Elevator-LIO)
  kContract,   // ① 空间状态契约 (FSR)
  kToken,      // ② Token 编码器 / TokenStore (FSR)
  kMemory,     // ③ ESN 状态语言 + 经验库 (FSR)
  kPolicy,     // ④ Jev + EdgePolicy (FSR)
  kRecovery,   // ⑤ LNR 恢复插件
  kTransport,  // ⑥ link 弱网调度器
  kCustom,
};

struct PluginDescriptor {
  std::string plugin_id;
  std::string name;
  std::string interface_version;
  PluginKind kind{PluginKind::kCustom};
  PluginAuthority authority{PluginAuthority::kEvidence};
  std::vector<std::string> provides;  // capability strings this plugin offers
  std::vector<std::string> needs;  // capabilities that must already be
                                      // provided by registered plugins
  // 0 = no periodic health reporting required; otherwise the plugin must
  // report health at least once per interval to stay live.
  std::int64_t health_interval_ns{0};
  bool operator==(const PluginDescriptor&) const = default;
};

enum class RegisterResult {
  kRegistered,
  kInvalidDescriptor,
  kDuplicateId,
  kUnresolvedRequires,
};

enum class PluginLiveness {
  kUnregistered,
  kLive,
  kStale,
};

struct PluginHealth {
  bool ok{false};
  std::string reason;
  std::int64_t stamp_ns{0};
};

class PluginRegistry final {
 public:
  // registers a plugin; needs are checked against capabilities already
  // provided by registered plugins.
  RegisterResult add(PluginDescriptor descriptor);
  bool remove(const std::string& plugin_id);

  const PluginDescriptor* find(const std::string& plugin_id) const;
  std::vector<const PluginDescriptor*> providers(
      const std::string& capability) const;
  std::vector<const PluginDescriptor*> plugins() const;

  void report_health(const std::string& plugin_id, bool ok,
                     std::string reason, std::int64_t now_ns);
  // A plugin without a health interval is always kLive once registered.
  // A plugin with one is kStale after 2x the interval without a report,
  // or immediately while its last report was not ok.
  PluginLiveness liveness(const std::string& plugin_id,
                          std::int64_t now_ns) const;
  const PluginHealth* health(const std::string& plugin_id) const;

  // Descriptor and health pointers returned by find/providers/health stay
  // valid across add/remove of OTHER ids (std::map has stable references);
  // they die only when that plugin itself is removed.
 private:
  static bool is_valid(const PluginDescriptor& descriptor) noexcept;

  std::map<std::string, PluginDescriptor> plugins_;
  std::set<std::string> provided_;
  std::map<std::string, PluginHealth> health_;
};

}  // namespace motionslam::runtime
