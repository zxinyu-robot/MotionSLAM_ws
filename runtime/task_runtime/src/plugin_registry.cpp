#include "motionslam_runtime/plugin_registry.hpp"

#include <algorithm>

namespace motionslam::runtime {

bool PluginRegistry::is_valid(const PluginDescriptor& d) noexcept {
  return !d.plugin_id.empty() && !d.name.empty() &&
         !d.interface_version.empty() && d.health_interval_ns >= 0;
}

RegisterResult PluginRegistry::add(PluginDescriptor descriptor) {
  if (!is_valid(descriptor)) {
    return RegisterResult::kInvalidDescriptor;
  }
  if (find(descriptor.plugin_id) != nullptr) {
    return RegisterResult::kDuplicateId;
  }
  for (const auto& need : descriptor.needs) {
    if (!provided_.contains(need)) {
      return RegisterResult::kUnresolvedRequires;
    }
  }

  for (const auto& cap : descriptor.provides) {
    provided_.insert(cap);
  }
  plugins_.push_back(std::move(descriptor));
  return RegisterResult::kRegistered;
}

bool PluginRegistry::remove(const std::string& plugin_id) {
  const auto it = std::find_if(plugins_.begin(), plugins_.end(),
                               [&](const PluginDescriptor& d) {
                                 return d.plugin_id == plugin_id;
                               });
  if (it == plugins_.end()) {
    return false;
  }

  provided_.clear();
  for (const auto& d : plugins_) {
    if (d.plugin_id == plugin_id) {
      continue;
    }
    for (const auto& cap : d.provides) {
      provided_.insert(cap);
    }
  }

  const auto h = std::find_if(health_.begin(), health_.end(),
                              [&](const auto& entry) {
                                return entry.first == plugin_id;
                              });
  if (h != health_.end()) {
    health_.erase(h);
  }
  plugins_.erase(it);
  return true;
}

const PluginDescriptor* PluginRegistry::find(
    const std::string& plugin_id) const {
  for (const auto& d : plugins_) {
    if (d.plugin_id == plugin_id) {
      return &d;
    }
  }
  return nullptr;
}

std::vector<const PluginDescriptor*> PluginRegistry::providers(
    const std::string& capability) const {
  std::vector<const PluginDescriptor*> out;
  for (const auto& d : plugins_) {
    if (std::find(d.provides.begin(), d.provides.end(), capability) !=
        d.provides.end()) {
      out.push_back(&d);
    }
  }
  return out;
}

std::vector<const PluginDescriptor*> PluginRegistry::plugins() const {
  std::vector<const PluginDescriptor*> out;
  out.reserve(plugins_.size());
  for (const auto& d : plugins_) {
    out.push_back(&d);
  }
  return out;
}

void PluginRegistry::report_health(const std::string& plugin_id, bool ok,
                                   std::string reason, std::int64_t now_ns) {
  for (auto& [id, h] : health_) {
    if (id == plugin_id) {
      h = PluginHealth{ok, std::move(reason), now_ns};
      return;
    }
  }
  health_.emplace_back(plugin_id, PluginHealth{ok, std::move(reason), now_ns});
}

PluginLiveness PluginRegistry::liveness(const std::string& plugin_id,
                                        std::int64_t now_ns) const {
  const auto* d = find(plugin_id);
  if (d == nullptr) {
    return PluginLiveness::kUnregistered;
  }
  if (d->health_interval_ns <= 0) {
    return PluginLiveness::kLive;
  }
  const auto* h = health(plugin_id);
  if (h == nullptr) {
    return PluginLiveness::kStale;
  }
  if (!h->ok || now_ns - h->stamp_ns > 2 * d->health_interval_ns) {
    return PluginLiveness::kStale;
  }
  return PluginLiveness::kLive;
}

const PluginHealth* PluginRegistry::health(
    const std::string& plugin_id) const {
  for (const auto& [id, h] : health_) {
    if (id == plugin_id) {
      return &h;
    }
  }
  return nullptr;
}

}  // namespace motionslam::runtime
