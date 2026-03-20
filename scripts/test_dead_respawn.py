#!/usr/bin/env python3
"""
test_dead_respawn.py — 模拟裁判系统话题，测试 IsDeadAndDispelDebuff 子树逻辑
========================================================================
用法:
    source install/setup.bash
    python3 scripts/test_dead_respawn.py

本脚本发布 BT 订阅的三个话题 (/robot_status, /game_status, /rfid_status)，
按时间线模拟:  存活 → 死亡(hp=0) → 复活(hp恢复) → 检测复活沿 → 回补给区
"""

import time
import rclpy
from rclpy.node import Node
from rm_decision_interfaces.msg import RMUL
from std_msgs.msg import Header


class DeadRespawnTestPublisher(Node):
    def __init__(self):
        super().__init__("test_dead_respawn_pub")

        # 三个 BT 订阅的话题
        self.robot_status_pub = self.create_publisher(RMUL, "/robot_status", 10)
        self.game_status_pub = self.create_publisher(RMUL, "/game_status", 10)
        self.rfid_status_pub = self.create_publisher(RMUL, "/rfid_status", 10)

        self.get_logger().info("=== DeadRespawnTestPublisher 启动 ===")
        self.get_logger().info("话题: /robot_status, /game_status, /rfid_status")

    def make_msg(
        self,
        hp: int = 400,
        game_progress: int = 4,
        remain_time: int = 200,
        rfid_supply: bool = False,
        rfid_control: bool = False,
        is_detect_enemy: bool = False,
        is_at_nav_goal: bool = False,
    ) -> RMUL:
        msg = RMUL()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        # 比赛状态
        msg.game_progress = game_progress
        msg.stage_remain_time = remain_time
        # 机器人状态
        msg.current_hp = hp
        msg.is_attacked = 0
        msg.shooter_heat = 0
        # RFID
        msg.rfid_supply_arrived = rfid_supply
        msg.rfid_control_arrived = rfid_control
        # 视觉
        msg.is_detect_enemy = is_detect_enemy
        # 导航
        msg.is_at_nav_goal = is_at_nav_goal
        return msg

    def publish_all(self, msg: RMUL):
        """同时发布到三个话题（BT 的各个 Sub* 节点分别订阅不同话题）"""
        self.robot_status_pub.publish(msg)
        self.game_status_pub.publish(msg)
        self.rfid_status_pub.publish(msg)

    def run_scenario(self):
        """按时间线执行测试场景"""
        rate_hz = 10  # 发布频率
        interval = 1.0 / rate_hz

        # ─── 阶段 1: 存活状态 (3秒, hp=400) ─────────────────
        self.get_logger().info("")
        self.get_logger().info("=" * 60)
        self.get_logger().info("【阶段 1】存活状态 hp=400 (持续 3s)")
        self.get_logger().info("  预期: IsDead=FAILURE → 跳过死亡分支 → 进入战斗/占点逻辑")
        self.get_logger().info("=" * 60)
        for _ in range(3 * rate_hz):
            msg = self.make_msg(hp=400, game_progress=4, remain_time=200)
            self.publish_all(msg)
            time.sleep(interval)
            rclpy.spin_once(self, timeout_sec=0)

        # ─── 阶段 2: 死亡 (5秒, hp=0) ────────────────────────
        self.get_logger().info("")
        self.get_logger().info("=" * 60)
        self.get_logger().info("【阶段 2】死亡 hp=0 (持续 5s)")
        self.get_logger().info("  预期: IsDead=SUCCESS → 停车等待 → RobotControl(stop_gimbal)")
        self.get_logger().info("  预期: DetectRespawn 设 was_dead=true")
        self.get_logger().info("=" * 60)
        for _ in range(5 * rate_hz):
            msg = self.make_msg(hp=0, game_progress=4, remain_time=195)
            self.publish_all(msg)
            time.sleep(interval)
            rclpy.spin_once(self, timeout_sec=0)

        # ─── 阶段 3: 复活 (5秒, hp=200) ──────────────────────
        self.get_logger().info("")
        self.get_logger().info("=" * 60)
        self.get_logger().info("【阶段 3】复活 hp=200 (持续 5s)")
        self.get_logger().info("  预期: DetectRespawn 检测复活沿 → need_recovery=true")
        self.get_logger().info("  预期: 打印 '检测到机器人复活沿！当前血量：200'")
        self.get_logger().info("  预期: IsRecoveryNeeded=SUCCESS → 进入 RecoveryFlow")
        self.get_logger().info("  预期: 导航到补给区 (SendGoal)")
        self.get_logger().info("=" * 60)
        for _ in range(5 * rate_hz):
            msg = self.make_msg(hp=200, game_progress=4, remain_time=190)
            self.publish_all(msg)
            time.sleep(interval)
            rclpy.spin_once(self, timeout_sec=0)

        # ─── 阶段 4: 到达补给区 + 刷到RFID卡 (5秒) ──────────
        self.get_logger().info("")
        self.get_logger().info("=" * 60)
        self.get_logger().info("【阶段 4】到达补给区，RFID=true (持续 5s)")
        self.get_logger().info("  预期: IsAtNavGoal=SUCCESS → IsSupplyCardDetected=SUCCESS")
        self.get_logger().info("  预期: WaitAndHeal 开始回血等待")
        self.get_logger().info("=" * 60)
        for i in range(5 * rate_hz):
            # 模拟逐渐回血
            hp_now = min(200 + i * 4, 400)
            msg = self.make_msg(
                hp=hp_now,
                game_progress=4,
                remain_time=185,
                rfid_supply=True,
                is_at_nav_goal=True,
            )
            self.publish_all(msg)
            if i % 10 == 0:
                self.get_logger().info(f"  回血中... hp={hp_now}")
            time.sleep(interval)
            rclpy.spin_once(self, timeout_sec=0)

        # ─── 阶段 5: 回血完成，恢复正常 (3秒, hp=400) ────────
        self.get_logger().info("")
        self.get_logger().info("=" * 60)
        self.get_logger().info("【阶段 5】回血完成 hp=400 (持续 3s)")
        self.get_logger().info("  预期: ClearRecoveryFlag → need_recovery=false")
        self.get_logger().info("  预期: 退出 IsDeadAndDispelDebuff 子树 → 恢复正常战斗逻辑")
        self.get_logger().info("=" * 60)
        for _ in range(3 * rate_hz):
            msg = self.make_msg(hp=400, game_progress=4, remain_time=180)
            self.publish_all(msg)
            time.sleep(interval)
            rclpy.spin_once(self, timeout_sec=0)

        # ─── 测试完成 ────────────────────────────────────────
        self.get_logger().info("")
        self.get_logger().info("=" * 60)
        self.get_logger().info("✅ 测试场景完成！请检查 BT 节点的日志输出。")
        self.get_logger().info("  关键日志搜索: '检测到机器人复活沿'")
        self.get_logger().info("  关键日志搜索: 'need_recovery'")
        self.get_logger().info("=" * 60)


def main():
    rclpy.init()
    node = DeadRespawnTestPublisher()
    try:
        node.run_scenario()
    except KeyboardInterrupt:
        node.get_logger().info("用户中断")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
