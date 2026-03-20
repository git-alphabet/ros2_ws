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
from rm_decision_interfaces.msg import (
    RMUCGameStatus,
    RMUCRobotStatus,
    RMUCRFIDStatus,
    RMUCRobotPosition,
    RMUCEnemyTracks
)
from std_msgs.msg import Header
import time
import sys


class RMUCTestPublisher(Node):
    def __init__(self):
        super().__init__('rmuc_test_publisher')
        
        # 创建 5 个独立话题发布者
        self.pub_game = self.create_publisher(RMUCGameStatus, '/game_status', 10)
        self.pub_robot = self.create_publisher(RMUCRobotStatus, '/robot_status', 10)
        self.pub_rfid = self.create_publisher(RMUCRFIDStatus, '/rfid_status', 10)
        self.pub_pos = self.create_publisher(RMUCRobotPosition, '/robot_position', 10)
        self.pub_radar = self.create_publisher(RMUCEnemyTracks, '/radar/enemy_tracks', 10)

        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        self.start_time = time.time()
        self.phase = 0
        self.get_logger().info('=== RMUC 行为树测试开始 (5 Topics Split Version) ===')

    def timer_callback(self):
        elapsed = time.time() - self.start_time
        
        # 初始化所有消息
        msg_game = RMUCGameStatus()
        msg_robot = RMUCRobotStatus()
        msg_rfid = RMUCRFIDStatus()
        msg_pos = RMUCRobotPosition()
        msg_radar = RMUCEnemyTracks()

        # 设置统一 header timestamps
        now_time = self.get_clock().now().to_msg()
        msg_game.header.stamp = now_time
        msg_robot.header.stamp = now_time
        msg_rfid.header.stamp = now_time
        msg_pos.header.stamp = now_time
        msg_radar.header.stamp = now_time

        if elapsed < 5.0:
            self._phase1_patrol(msg_game, msg_robot, msg_rfid, msg_pos, msg_radar)
        elif elapsed < 10.0:
            self._phase2_dead(msg_game, msg_robot, msg_rfid, msg_pos, msg_radar)
        elif elapsed < 15.0:
            self._phase3_respawn(msg_game, msg_robot, msg_rfid, msg_pos, msg_radar)
        elif elapsed < 20.0:
            self._phase4_critical(msg_game, msg_robot, msg_rfid, msg_pos, msg_radar)
        elif elapsed < 25.0:
            self._phase5_combat(msg_game, msg_robot, msg_rfid, msg_pos, msg_radar)
        else:
            self.get_logger().info('=== RMUC 行为树测试完成 (25s) ===')
            # 发送一帧空数据作为结束标记（可选）
            rclpy.shutdown()
            return

        # 发布所有消息
        self.pub_game.publish(msg_game)
        self.pub_robot.publish(msg_robot)
        self.pub_rfid.publish(msg_rfid)
        self.pub_pos.publish(msg_pos)
        self.pub_radar.publish(msg_radar)

    def _phase1_patrol(self, game, robot, rfid, pos, radar):
        """Phase 1: 比赛进行，满血满弹，正常巡逻"""
        if self.phase != 1:
            self.phase = 1
            self.get_logger().info(
                '▶ Phase 1: 比赛进行中 — 满血满弹正常巡逻 (PatrolAndScan)')
        
        # GameStatus
        game.game_progress = 4  # 比赛进行中
        game.stage_remain_time = 400
        
        # RobotStatus (满血满弹)
        robot.current_hp = 600
        robot.max_hp = 600
        robot.shooter_heat = 0
        robot.heat_limit = 200
        robot.cooling_rate = 40
        robot.ammo_allow = 200
        robot.ammo_left = 200
        robot.is_dead = False
        robot.shooter_power_output = True
        robot.can_remote_heal = False
        robot.can_remote_ammo = False
        robot.team_coins = 500
        robot.can_respawn = False
        robot.respawn_countdown_s = 0
        robot.base_hp_cur = 5000
        robot.base_hp_max = 5000
        robot.outpost_alive = True
        
        # RFID (全关)
        rfid.rfid_supply = False
        rfid.rfid_base_buff = False
        rfid.rfid_outpost_buff = False
        rfid.rfid_fortress_ally = False
        rfid.rfid_fortress_enemy = False
        rfid.rfid_central_highland = False
        rfid.rfid_ladder_highland = False
        
        # Position
        pos.pose_x = 3.0
        pos.pose_y = 2.0
        pos.pose_yaw = 0.0
        pos.is_at_nav_goal = False
        
        # Radar (无敌人)
        radar.enemy_count = 0

    def _phase2_dead(self, game, robot, rfid, pos, radar):
        """Phase 2: 机器人死亡 -> RespawnRecovery (死亡停车)"""
        if self.phase != 2:
            self.phase = 2
            self.get_logger().info('▶ Phase 2: 机器人死亡 (Dead State)')

        # GameStatus
        game.game_progress = 4
        game.stage_remain_time = 395

        # RobotStatus (死亡)
        robot.current_hp = 0
        robot.max_hp = 600
        robot.is_dead = True  # <--- DEAD
        robot.can_respawn = True
        robot.respawn_countdown_s = 5

        # 其他默认
        robot.outpost_alive = True
        
        # Position
        pos.pose_x = 3.0
        pos.pose_y = 2.0
        
        # Radar
        radar.enemy_count = 0

    def _phase3_respawn(self, game, robot, rfid, pos, radar):
        """Phase 3: 复活 + 虚弱态 + 补给区 RFID 触发 -> 应该触发补血"""
        if self.phase != 3:
            self.phase = 3
            self.get_logger().info('▶ Phase 3: 复活虚弱 + 补给区 RFID (Respawn & Supply)')

        # GameStatus
        game.game_progress = 4
        
        # RobotStatus (复活，虚弱，满血但缺弹)
        robot.current_hp = 600
        robot.max_hp = 600
        robot.is_dead = False
        robot.shooter_power_output = False  # <--- WEAK
        robot.ammo_left = 0   # 缺弹
        robot.can_remote_ammo = True
        robot.team_coins = 200

        # RFID (补给区触发)
        rfid.rfid_supply = True  # <--- RFID Triggered
        
        # Position (在补给区附近)
        pos.pose_x = 0.5
        pos.pose_y = 0.5
        pos.is_at_nav_goal = True

    def _phase4_critical(self, game, robot, rfid, pos, radar):
        """Phase 4: 低血量 + 脱战 -> CriticalSurvival (撤退回血)"""
        if self.phase != 4:
            self.phase = 4
            self.get_logger().info('▶ Phase 4: 残血脱战 (Critical Retreat)')

        # GameStatus
        game.game_progress = 4
        
        # RobotStatus (残血)
        robot.current_hp = 100  # <--- Critical HP
        robot.max_hp = 600
        robot.is_dead = False
        robot.shooter_power_output = True
        robot.outpost_alive = True
        
        # RFID
        rfid.rfid_supply = False
        
        # Position (在外游荡)
        pos.pose_x = 5.0
        pos.pose_y = 4.0

    def _phase5_combat(self, game, robot, rfid, pos, radar):
        """Phase 5: 满血 + 有敌方目标 -> EngageCombat"""
        if self.phase != 5:
            self.phase = 5
            self.get_logger().info('▶ Phase 5: 遭遇战 (Engage Combat)')

        # GameStatus
        game.game_progress = 4
        
        # RobotStatus (状态良好)
        robot.current_hp = 500
        robot.max_hp = 600
        robot.ammo_left = 100
        robot.is_dead = False
        robot.shooter_power_output = True
        
        # Radar (发现敌人)
        radar.enemy_count = 1
        radar.enemy_robot_id = [1]
        radar.enemy_x = [6.0]
        radar.enemy_y = [6.0]
        radar.enemy_confidence = [0.9]

def main(args=None):
    rclpy.init(args=args)
    node = RMUCTestPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


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
        msg.shooter_power_output = True
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
        msg.shooter_power_output = False   # 复活后虚弱
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
        msg.shooter_power_output = True
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
        msg.shooter_power_output = True
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
