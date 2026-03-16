#include "rm_behavior_tree/plugins/rmuc_2026/action/select_nearest_dispel_card.hpp"
#include <array>

namespace rm_behavior_tree
{

SelectNearestBuffZoneAction::SelectNearestBuffZoneAction(
  const std::string & name, const BT::NodeConfig & conf)
: BT::SyncActionNode(name, conf) {}

BT::NodeStatus SelectNearestBuffZoneAction::tick()
{
  double px = 0, py = 0;
  getInput("pose_x", px);
  getInput("pose_y", py);

  struct Pt { double x; double y; };
  std::array<Pt, 3> pts{};
  double v = 0;
  // 候选点 0: buff_zone （补给区 RFID 中心）
  getInput("buff_zone_x", v); pts[0].x = v;
  getInput("buff_zone_y", v); pts[0].y = v;
  // 候选点 1: base_buff （基地增益区）
  getInput("base_buff_x", v); pts[1].x = v;
  getInput("base_buff_y", v); pts[1].y = v;
  // 候选点 2: outpost_buff （前哨站增益区）
  getInput("outpost_buff_x", v); pts[2].x = v;
  getInput("outpost_buff_y", v); pts[2].y = v;

  double best = 1e9;
  Pt best_pt{0, 0};
  for (auto & p : pts) {
    double d = std::hypot(p.x - px, p.y - py);
    if (d < best) { best = d; best_pt = p; }
  }
  setOutput("goal_x", best_pt.x);
  setOutput("goal_y", best_pt.y);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  // 场景 A: 申弱解除（WeaknessRecovery）——申弱时寻找最近 RFID 刷卡点
  factory.registerNodeType<rm_behavior_tree::SelectNearestBuffZoneAction>("SelectNearestDispelCard");
  // 场景 B: 补弹（AmmoPlan）——缺弹时寻找最近补弹点（三个區均支持补弹）
  factory.registerNodeType<rm_behavior_tree::SelectNearestBuffZoneAction>("SelectNearestResupplyStation");
}
