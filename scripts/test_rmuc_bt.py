#!/usr/bin/env python3
"""
RMUC 行为树集成测试：模拟 5 阶段比赛场景，验证所有主要子树逻辑。
阶段:
  Phase 1 (0-5s)  : 比赛进行中，满血满弹，正常巡逻   → PatrolAndScan
  Phase 2 (5-10s) : 机器人死亡                        → RespawnRecovery (死亡停车)
  Phase 3 (10-15s): 复活 + 虚弱态，补给区 RFID 触发   → RespawnRecovery (回血/触卡)
  Phase 4 (15-20s): 低血量 + 脱战 → 撤退回血          → CriticalSurvival / HealPlan
  Phase 5 (20-25s): 满血满弹 + 有敌方目标              → EngageCombat
"""

import rclpy
from rclpy.node import Node
from rm_decision_interfaces.msg import RMUC
from std_msgs.msg import Header
import time
import sys


class RMUCTestPublisher(Node):
    def __init__(self):
        super().__init__('rmuc_test_publisher')
        self.pub = self.create_publisher(RMUC, '/rmuc', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        self.start_time = time.time()
        self.phase = 0
        self.get_logger().info('=== RMUC 行为树测试开始 ===')

    def timer_callback(self):
        elapsed = time.time() - self.start_time
        msg = RMUC()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()

        if elapsed < 5.0:
            self._phase1_patrol(msg, elapsed)
        elif elapsed < 10.0:
            self._phase2_dead(msg, elapsed)
        elif elapsed < 15.0:
            self._phase3_respawn(msg, elapsed)
        elif elapsed < 20.0:
            self._phase4_critical(msg, elapsed)
        elif elapsed < 25.0:
            self._phase5_combat(msg, elapsed)
        else:
            self.get_logger().info('=== RMUC 行为树测试完成 (25s) ===')
            rclpy.shutdown()
            return

        self.pub.publish(msg)

    def _phase1_patrol(self, msg, elapsed):
        """Phase 1: 比赛进行，满血满弹，正常巡逻"""
        if self.phase != 1:
            self.phase = 1
            self.get_logger().info(
                '▶ Phase 1: 比赛进行中 — 满血满弹正常巡逻 (PatrolAndScan)')
        # 比赛状态
        msg.game_progress = 4  # 比赛进行中
        msg.stage_remain_time = 400
        # 满血满弹
        msg.current_hp = 600
        msg.max_hp = 600
        msg.shooter_heat = 0
        msg.heat_limit = 200
        msg.cooling_rate = 40
        msg.ammo_allow = 200
        msg.ammo_left = 200
        # 状态标志
        msg.is_dead = False
        msg.is_weak = False
        msg.is_disengaged = True
        msg.disengage_cd_s = 0.0
        # 经济
        msg.can_remote_heal = False
        msg.can_remote_ammo = False
        msg.team_coins = 500
        # 复活
        msg.can_respawn = False
        msg.respawn_countdown_s = 0
        # 己方基地/前哨站
        msg.base_hp_cur = 5000
        msg.base_hp_max = 5000
        msg.outpost_alive = True
        # RFID 全关
        msg.rfid_supply = False
        msg.rfid_base_buff = False
        msg.rfid_outpost_buff = False
        msg.rfid_fortress_ally = False
        msg.rfid_fortress_enemy = False
        msg.rfid_central_highland = False
        msg.rfid_ladder_highland = False
        # 位置
        msg.pose_x = 3.0
        msg.pose_y = 2.0
        msg.pose_yaw = 0.0
        msg.is_at_nav_goal = False
        # 无敌方目标
        msg.enemy_count = 0

    def _phase2_dead(self, msg, elapsed):
        """Phase 2: 机器人死亡"""
        if self.phase != 2:
            self.phase = 2
            self.get_logger().info(
                '▶ Phase 2: 机器人死亡 — RespawnRecovery 死亡停车')
        msg.game_progress = 4
        msg.stage_remain_time = 395
        msg.current_hp = 0
        msg.max_hp = 600
        msg.shooter_heat = 0
        msg.heat_limit = 200
        msg.cooling_rate = 40
        msg.ammo_allow = 150
        msg.ammo_left = 150
        msg.is_dead = True
        msg.is_weak = False
        msg.is_disengaged = False
        msg.disengage_cd_s = 6.0
        msg.can_remote_heal = False
        msg.can_remote_ammo = False
        msg.team_coins = 450
        # 复活读条
        if elapsed < 8.0:
            msg.can_respawn = False
            msg.respawn_countdown_s = int(10 - elapsed)
        else:
            msg.can_respawn = True
            msg.respawn_countdown_s = 0
        msg.base_hp_cur = 4800
        msg.base_hp_max = 5000
        msg.outpost_alive = True
        msg.rfid_supply = False
        msg.rfid_base_buff = False
        msg.rfid_outpost_buff = False
        msg.rfid_fortress_ally = False
        msg.rfid_fortress_enemy = False
        msg.rfid_central_highland = False
        msg.rfid_ladder_highland = False
        msg.pose_x = 3.0
        msg.pose_y = 2.0
        msg.pose_yaw = 0.0
        msg.is_at_nav_goal = False
        msg.enemy_count = 0

    def _phase3_respawn(self, msg, elapsed):
        """Phase 3: 复活 + 虚弱态，到达补给区触卡"""
        if self.phase != 3:
            self.phase = 3
            self.get_logger().info(
                '▶ Phase 3: 复活 + 虚弱态 — 导航补给区触卡回血')
        msg.game_progress = 4
        msg.stage_remain_time = 385
        msg.current_hp = 200  # 复活后低血量
        msg.max_hp = 600
        msg.shooter_heat = 0
        msg.heat_limit = 200
        msg.cooling_rate = 40
        msg.ammo_allow = 150
        msg.ammo_left = 150
        msg.is_dead = False
        msg.is_weak = True   # 复活后虚弱
        msg.is_disengaged = True
        msg.disengage_cd_s = 0.0
        msg.can_remote_heal = False
        msg.can_remote_ammo = False
        msg.team_coins = 400
        msg.can_respawn = False
        msg.respawn_countdown_s = 0
        msg.base_hp_cur = 4800
        msg.base_hp_max = 5000
        msg.outpost_alive = True
        # 模拟到达补给区 RFID
        if elapsed > 13.0:
            msg.rfid_supply = True
            msg.is_at_nav_goal = True
        else:
            msg.rfid_supply = False
            msg.is_at_nav_goal = False
        msg.rfid_base_buff = False
        msg.rfid_outpost_buff = False
        msg.rfid_fortress_ally = False
        msg.rfid_fortress_enemy = False
        msg.rfid_central_highland = False
        msg.rfid_ladder_highland = False
        msg.pose_x = 1.0
        msg.pose_y = 1.0
        msg.pose_yaw = 0.0
        msg.enemy_count = 0

    def _phase4_critical(self, msg, elapsed):
        """Phase 4: 低血量 + 脱战 → 紧急撤退回血"""
        if self.phase != 4:
            self.phase = 4
            self.get_logger().info(
                '▶ Phase 4: 低血量脱战 — CriticalSurvival / 撤退回血')
        msg.game_progress = 4
        msg.stage_remain_time = 370
        msg.current_hp = 100  # 低血量
        msg.max_hp = 600
        msg.shooter_heat = 180  # 高热量
        msg.heat_limit = 200
        msg.cooling_rate = 40
        msg.ammo_allow = 50   # 低弹药
        msg.ammo_left = 50
        msg.is_dead = False
        msg.is_weak = False
        msg.is_disengaged = True
        msg.disengage_cd_s = 0.0
        msg.can_remote_heal = True
        msg.can_remote_ammo = True
        msg.team_coins = 350
        msg.can_respawn = False
        msg.respawn_countdown_s = 0
        msg.base_hp_cur = 4600
        msg.base_hp_max = 5000
        msg.outpost_alive = True
        msg.rfid_supply = False
        msg.rfid_base_buff = False
        msg.rfid_outpost_buff = False
        msg.rfid_fortress_ally = False
        msg.rfid_fortress_enemy = False
        msg.rfid_central_highland = False
        msg.rfid_ladder_highland = False
        msg.pose_x = 5.0
        msg.pose_y = 3.0
        msg.pose_yaw = 1.57
        msg.is_at_nav_goal = False
        msg.enemy_count = 0

    def _phase5_combat(self, msg, elapsed):
        """Phase 5: 满血满弹 + 敌方目标出现 → 交战"""
        if self.phase != 5:
            self.phase = 5
            self.get_logger().info(
                '▶ Phase 5: 满血满弹 + 有目标 — EngageCombat 交战')
        msg.game_progress = 4
        msg.stage_remain_time = 350
        msg.current_hp = 600
        msg.max_hp = 600
        msg.shooter_heat = 20
        msg.heat_limit = 200
        msg.cooling_rate = 40
        msg.ammo_allow = 200
        msg.ammo_left = 200
        msg.is_dead = False
        msg.is_weak = False
        msg.is_disengaged = False
        msg.disengage_cd_s = 4.0
        msg.can_remote_heal = False
        msg.can_remote_ammo = False
        msg.team_coins = 300
        msg.can_respawn = False
        msg.respawn_countdown_s = 0
        msg.base_hp_cur = 4400
        msg.base_hp_max = 5000
        msg.outpost_alive = True
        msg.rfid_supply = False
        msg.rfid_base_buff = False
        msg.rfid_outpost_buff = False
        msg.rfid_fortress_ally = False
        msg.rfid_fortress_enemy = False
        msg.rfid_central_highland = False
        msg.rfid_ladder_highland = False
        msg.pose_x = 4.0
        msg.pose_y = 3.0
        msg.pose_yaw = 0.5
        msg.is_at_nav_goal = False
        # 两个敌方目标
        msg.enemy_count = 2
        msg.enemy_robot_id = [103, 104]  # 蓝方步兵
        msg.enemy_x = [5.5, 6.0]
        msg.enemy_y = [4.0, 2.5]
        msg.enemy_confidence = [0.85, 0.6]


def main():
    rclpy.init()
    node = RMUCTestPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
