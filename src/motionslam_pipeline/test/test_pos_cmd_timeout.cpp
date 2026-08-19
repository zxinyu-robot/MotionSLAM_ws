#include <gtest/gtest.h>

#include "motionslam/WatchdogPolicy.hpp"

namespace motionslam {

TEST(PosCmdTimeout, MatchesPipelineYaml500ms) {
  EXPECT_TRUE(ShouldZeroOnInputTimeout(501, 0, 500));
  EXPECT_TRUE(ShouldZeroOnInputTimeout(0, 501, 500));
  EXPECT_FALSE(ShouldZeroOnInputTimeout(499, 499, 500));
}

}  // namespace motionslam
