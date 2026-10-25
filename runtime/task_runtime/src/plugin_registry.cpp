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

  // Transactional: if any container update throws, roll everything back so
  // the registry never holds a plugin that enumeration cannot see.
  const auto caps = descriptor.provides;
  const auto id = descriptor.plugin_id;
  order_.push_back(id);
  try {
    plugins_.emplace(id, std::move(descriptor));
    for (const auto& cap : caps) {
      provided_.insert(cap);
    }
  } catch (...) {
    plugins_.erase(id);
    order_.pop_back();
    // Roll back only caps no surviving plugin still provides — no
    // bookkeeping allocation in the unwind path.
    for (const auto& cap : caps) {
      const bool still_provided = std::any_of(
          plugins_.begin(), plugins_.end(), [&](const auto& entry) {
            const auto& p = entry.second.provides;
            return std::find(p.begin(), p.end(), cap) != p.end();
          });
      if (!still_provided) {
        provided_.erase(cap);
      }
    }
    throw;
  }
  return RegisterResult::kRegistered;
}

bool PluginRegistry::remove(const std::string& plugin_id) {
  const auto it = plugins_.find(plugin_id);
  if (it == plugins_.end()) {
    return false;
  }

  plugins_.erase(it);
  health_.erase(plugin_id);
  order_.erase(std::remove(order_.begin(), order_.end(), plugin_id),
               order_.end());

  provided_.clear();
  for (const auto& [id, d] : plugins_) {
    for (const auto& cap : d.provides) {
      provided_.insert(cap);
    }
  }
  return true;
}

const PluginDescriptor* PluginRegistry::find(
    const std::string& plugin_id) const {
  const auto it = plugins_.find(plugin_id);
  return it == plugins_.end() ? nullptr : &it->second;
}

std::vector<const PluginDescriptor*> PluginRegistry::providers(
    const std::string& capability) const {
  std::vector<const PluginDescriptor*> out;
  for (const auto& id : order_) {
    const auto& d = plugins_.at(id);
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
  for (const auto& id : order_) {
    out.push_back(&plugins_.at(id));
  }
  return out;
}

void PluginRegistry::report_health(const std::string& plugin_id, bool ok,
                                   std::string reason, std::int64_t now_ns) {
  // Reports only count for a currently registered plugin: a late heartbeat
  // arriving after removal must not validate a future re-registration.
  if (plugins_.find(plugin_id) == plugins_.end()) {
    return;
  }
  health_[plugin_id] = PluginHealth{ok, std::move(reason), now_ns};
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
  const auto it = health_.find(plugin_id);
  return it == health_.end() ? nullptr : &it->second;
}

}  // namespace motionslam::runtime
