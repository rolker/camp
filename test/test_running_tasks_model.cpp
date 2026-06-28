// Tests for RunningTasksModel::hasTaskPoses — the guard that prevents
// taskSelected from firing for synthetic group/parent rows that have no
// geometry. Covers: group row with empty poses returns false; leaf task with
// at least one pose returns true.

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "marine_nav_interfaces/msg/task_information.hpp"
#include "running_tasks/running_tasks_model.h"

namespace
{

marine_nav_interfaces::msg::TaskInformation makeTask(
    const std::string& id,
    const std::string& type,
    bool with_pose = false)
{
  marine_nav_interfaces::msg::TaskInformation t;
  t.id = id;
  t.type = type;
  t.priority = 10;
  t.done = false;
  if (with_pose)
  {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = "map";
    ps.pose.position.x = 1.0;
    ps.pose.position.y = 2.0;
    t.poses.push_back(ps);
  }
  return t;
}

}  // namespace

class RunningTasksModelTest : public ::testing::Test
{
protected:
  rclcpp::Node::SharedPtr node;
  RunningTasksModel model;

  void SetUp() override
  {
    node = rclcpp::Node::make_shared("running_tasks_model_test");
  }

  void setTasks(const std::string& current,
                const std::vector<marine_nav_interfaces::msg::TaskInformation>& tasks)
  {
    model.setTasks(QString::fromStdString(current), tasks, node->get_clock());
  }
};

TEST_F(RunningTasksModelTest, hasTaskPoses_invalidIndex_returnsFalse)
{
  EXPECT_FALSE(model.hasTaskPoses(QModelIndex()));
}

TEST_F(RunningTasksModelTest, hasTaskPoses_groupRowWithEmptyPoses_returnsFalse)
{
  // A parent/group row whose TaskInformation has no poses (structural parent).
  std::vector<marine_nav_interfaces::msg::TaskInformation> tasks = {
      makeTask("survey_a", "survey", /*with_pose=*/false),
      makeTask("survey_a/line_1", "survey_line", /*with_pose=*/true),
  };
  setTasks("survey_a/line_1", tasks);

  // First top-level row should be the parent "survey_a" task (no poses).
  const QModelIndex parent_idx = model.index(0, 0);
  ASSERT_TRUE(parent_idx.isValid());
  EXPECT_EQ(model.idForIndex(parent_idx), QStringLiteral("survey_a"));
  EXPECT_FALSE(model.hasTaskPoses(parent_idx))
      << "group row with empty poses must return false";
}

TEST_F(RunningTasksModelTest, hasTaskPoses_leafTaskWithPoses_returnsTrue)
{
  std::vector<marine_nav_interfaces::msg::TaskInformation> tasks = {
      makeTask("survey_a", "survey", /*with_pose=*/false),
      makeTask("survey_a/line_1", "survey_line", /*with_pose=*/true),
  };
  setTasks("survey_a/line_1", tasks);

  // Child of the first top-level row: survey_a/line_1 (has a pose).
  const QModelIndex parent_idx = model.index(0, 0);
  ASSERT_TRUE(parent_idx.isValid());
  const QModelIndex leaf_idx = model.index(0, 0, parent_idx);
  ASSERT_TRUE(leaf_idx.isValid());
  EXPECT_EQ(model.idForIndex(leaf_idx), QStringLiteral("survey_a/line_1"));
  EXPECT_TRUE(model.hasTaskPoses(leaf_idx))
      << "leaf task with at least one pose must return true";
}

TEST_F(RunningTasksModelTest, hasTaskPoses_topLevelTaskWithPoses_returnsTrue)
{
  // A flat (non-hierarchical) task with poses — e.g. a simple goto.
  std::vector<marine_nav_interfaces::msg::TaskInformation> tasks = {
      makeTask("goto_wp1", "goto", /*with_pose=*/true),
  };
  setTasks("goto_wp1", tasks);

  const QModelIndex idx = model.index(0, 0);
  ASSERT_TRUE(idx.isValid());
  EXPECT_TRUE(model.hasTaskPoses(idx));
}

TEST_F(RunningTasksModelTest, hasTaskPoses_emptyModel_returnsFalse)
{
  // Nothing set — any index is invalid.
  EXPECT_FALSE(model.hasTaskPoses(model.index(0, 0)));
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
