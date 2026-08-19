#include "motionslam/MapSessionManager.hpp"

#include <gtest/gtest.h>
#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <string>

namespace {

namespace fs = std::filesystem;
using motionslam::GateRequest;
using motionslam::MapSessionManager;
using motionslam::NavigationContextState;
using motionslam::NavigationContextStatePage;

class MapSessionManagerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    root_ = fs::temp_directory_path() /
            ("motionslam_registry_test_" + std::to_string(::getpid()));
    fs::create_directories(root_ / "building_01/floor_01/registry");
    pcd_ = root_ / "map.pcd";
    std::ofstream pcd(pcd_);
    pcd << "# .PCD v0.7\n"
           "VERSION 0.7\n"
           "FIELDS x y z\n"
           "SIZE 4 4 4\n"
           "TYPE F F F\n"
           "COUNT 1 1 1\n"
           "WIDTH 3\n"
           "HEIGHT 1\n"
           "VIEWPOINT 0 0 0 1 0 0 0\n"
           "POINTS 3\n"
           "DATA ascii\n"
           "-2 -1 -0.5\n"
           "1 3 0\n"
           "4 2 1.5\n";
    pcd.close();
    const std::string md5 = MapSessionManager::ComputeMd5(pcd_.string());

    std::ofstream root(root_ / "floor_registry.yaml");
    root << "schema_version: 1\n"
            "floors:\n"
            "  - building_id: building_01\n"
            "    floor_id: floor_01\n"
            "    registry: building_01/floor_01/registry/floor.yaml\n";
    root.close();

    std::ofstream floor(root_ / "building_01/floor_01/registry/floor.yaml");
    floor << "schema_version: 1\n"
             "building_id: building_01\n"
             "floor_id: floor_01\n"
             "map_id: test_map\n"
             "map_version: 7\n"
             "frame_id: world\n"
             "tiles:\n"
             "  main:\n";
    for (const char* role : {"map_nav", "map_reloc", "map_viz"}) {
      floor << "    " << role << ":\n"
            << "      path: ../../../map.pcd\n"
            << "      md5: " << md5 << '\n';
    }
  }

  void TearDown() override { fs::remove_all(root_); }

  MapSessionManager LoadManager() const {
    MapSessionManager manager;
    std::string error;
    EXPECT_TRUE(manager.Load((root_ / "floor_registry.yaml").string(),
                             "building_01", "floor_01", "test_map",
                             "session_a", "main", 0.5, &error))
        << error;
    return manager;
  }

  fs::path root_;
  fs::path pcd_;
};

TEST_F(MapSessionManagerTest, OdomOnlyModeSkipsRelocationGate) {
  auto manager = LoadManager();
  manager.SetRequireRelocationConverged(false);
  EXPECT_TRUE(manager.Context().ready);
  EXPECT_TRUE(manager.Context().relocation_converged);
  GateRequest request{"test_map", "session_a", "floor_01", "main", 0, 0, 0};
  EXPECT_TRUE(manager.CheckStart(request).accepted);
}

TEST_F(MapSessionManagerTest, LoadsRegistryComputesBoundsAndGatesFitness) {
  auto manager = LoadManager();
  EXPECT_TRUE(manager.Context().registry_valid);
  EXPECT_DOUBLE_EQ(manager.Context().floor.bounds.min_x, -2.0);
  EXPECT_DOUBLE_EQ(manager.Context().floor.bounds.max_y, 3.0);
  EXPECT_FALSE(manager.Context().ready);

  manager.UpdateRelocation(true, 0.25);
  ASSERT_TRUE(manager.Context().ready);
  GateRequest request{"test_map", "session_a", "floor_01", "main", 0, 0, 0};
  EXPECT_TRUE(manager.CheckStart(request).accepted);
  request.x = 10;
  EXPECT_EQ(manager.CheckGoal(request).reason, "outside_map_bounds");
  request.x = 0;
  request.session_id = "stale_session";
  EXPECT_EQ(manager.CheckGoal(request).reason, "session_id_mismatch");

  manager.UpdateRelocation(true, 0.75);
  EXPECT_FALSE(manager.Context().ready);
  EXPECT_EQ(manager.Context().reason, "fitness_above_threshold");
}

TEST_F(MapSessionManagerTest, RejectsWrongTileAndMd5) {
  MapSessionManager manager;
  std::string error;
  EXPECT_FALSE(manager.Load((root_ / "floor_registry.yaml").string(),
                            "building_01", "floor_01", "test_map", "session_a",
                            "missing", 0.5, &error));
  EXPECT_NE(error.find("tile"), std::string::npos);

  std::ofstream(pcd_, std::ios::app) << "\n";
  EXPECT_FALSE(manager.Load((root_ / "floor_registry.yaml").string(),
                            "building_01", "floor_01", "test_map", "session_a",
                            "main", 0.5, &error));
  EXPECT_NE(error.find("md5"), std::string::npos);
}

TEST_F(MapSessionManagerTest, PublishesVersionedSharedMemoryState) {
  auto manager = LoadManager();
  manager.UpdateRelocation(true, 0.2);
  const fs::path state_path = root_ / "navigation_context.state";
  NavigationContextStatePage page(state_path.string());
  std::string error;
  ASSERT_TRUE(page.Write(manager.Context(), &error)) << error;

  NavigationContextState state;
  ASSERT_TRUE(NavigationContextStatePage::Read(state_path.string(), &state,
                                               &error))
      << error;
  EXPECT_EQ(state.magic, motionslam::kNavigationContextMagic);
  EXPECT_EQ(state.schema_version, motionslam::kNavigationContextSchema);
  EXPECT_GT(state.generation, 0U);
  EXPECT_EQ(state.sequence % 2, 0U);
  EXPECT_TRUE(state.ready);
  EXPECT_STREQ(state.map_id.data(), "test_map");
  EXPECT_STREQ(state.session_id.data(), "session_a");
  EXPECT_EQ(
      state.session_token, motionslam::StableContextToken("session_a"));
  EXPECT_EQ(state.tile_token, motionslam::StableContextToken("main"));

  ASSERT_TRUE(page.Write(manager.Context(), &error)) << error;
  NavigationContextState heartbeat;
  ASSERT_TRUE(NavigationContextStatePage::Read(
      state_path.string(), &heartbeat, &error)) << error;
  EXPECT_EQ(heartbeat.generation, state.generation);
  EXPECT_GT(heartbeat.sequence, state.sequence);
}

TEST_F(MapSessionManagerTest, HealthUpdatesDoNotChangeMapGeneration) {
  auto manager = LoadManager();
  const fs::path state_path = root_ / "health_generation.state";
  NavigationContextStatePage page(state_path.string());
  std::string error;
  auto write_and_read = [&](const motionslam::NavigationContext& context) {
    EXPECT_TRUE(page.Write(context, &error)) << error;
    NavigationContextState state;
    EXPECT_TRUE(NavigationContextStatePage::Read(
        state_path.string(), &state, &error)) << error;
    return state;
  };

  auto context = manager.Context();
  const auto initial = write_and_read(context);
  context.relocation_fitness = 0.4;
  const auto fitness_update = write_and_read(context);
  context.relocation_fitness = 0.2;
  context.max_fitness = 0.3;
  context.relocation_converged = true;
  context.ready = true;
  context.reason = "ready";
  const auto ready = write_and_read(context);
  context.relocation_fitness = 0.8;
  context.ready = false;
  context.reason = "fitness_above_threshold";
  const auto unhealthy = write_and_read(context);

  EXPECT_EQ(fitness_update.generation, initial.generation);
  EXPECT_EQ(ready.generation, initial.generation);
  EXPECT_EQ(unhealthy.generation, initial.generation);
  EXPECT_LT(initial.sequence, fitness_update.sequence);
  EXPECT_LT(fitness_update.sequence, ready.sequence);
  EXPECT_LT(ready.sequence, unhealthy.sequence);
}

TEST_F(MapSessionManagerTest, NewWriterInstanceAdvancesGenerationOnce) {
  auto manager = LoadManager();
  manager.UpdateRelocation(true, 0.2);
  const fs::path state_path = root_ / "writer_restart.state";
  std::string error;
  NavigationContextState before_restart;
  {
    NavigationContextStatePage writer(state_path.string());
    ASSERT_TRUE(writer.Write(manager.Context(), &error)) << error;
    ASSERT_TRUE(NavigationContextStatePage::Read(
        state_path.string(), &before_restart, &error)) << error;
    ASSERT_TRUE(writer.Write(manager.Context(), &error)) << error;
    NavigationContextState heartbeat;
    ASSERT_TRUE(NavigationContextStatePage::Read(
        state_path.string(), &heartbeat, &error)) << error;
    EXPECT_EQ(heartbeat.generation, before_restart.generation);
  }

  NavigationContextState after_restart;
  {
    NavigationContextStatePage restarted_writer(state_path.string());
    ASSERT_TRUE(restarted_writer.Write(manager.Context(), &error)) << error;
    ASSERT_TRUE(NavigationContextStatePage::Read(
        state_path.string(), &after_restart, &error)) << error;
    EXPECT_EQ(after_restart.generation, before_restart.generation + 1U);

    ASSERT_TRUE(restarted_writer.Write(manager.Context(), &error)) << error;
    NavigationContextState heartbeat;
    ASSERT_TRUE(NavigationContextStatePage::Read(
        state_path.string(), &heartbeat, &error)) << error;
    EXPECT_EQ(heartbeat.generation, after_restart.generation);
  }
}

TEST_F(MapSessionManagerTest, MapIdentityUpdatesIncrementGeneration) {
  auto manager = LoadManager();
  const fs::path state_path = root_ / "identity_generation.state";
  NavigationContextStatePage page(state_path.string());
  std::string error;
  auto write_and_read = [&](const motionslam::NavigationContext& context) {
    EXPECT_TRUE(page.Write(context, &error)) << error;
    NavigationContextState state;
    EXPECT_TRUE(NavigationContextStatePage::Read(
        state_path.string(), &state, &error)) << error;
    return state;
  };

  auto context = manager.Context();
  auto previous = write_and_read(context);
  auto expect_increment = [&](const motionslam::NavigationContext& changed) {
    const auto current = write_and_read(changed);
    EXPECT_EQ(current.generation, previous.generation + 1U);
    previous = current;
  };

  context.floor.map_id = "replacement-map";
  expect_increment(context);
  context.session_id = "session_b";
  expect_increment(context);
  context.floor.tile_id = "secondary";
  expect_increment(context);
  context.floor.frame_id = "map";
  expect_increment(context);
  context.floor.map_version += 1U;
  expect_increment(context);
  context.floor.bounds.max_x += 1.0;
  expect_increment(context);
}

}  // namespace
