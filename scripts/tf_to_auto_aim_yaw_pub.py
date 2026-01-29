#!/usr/bin/env python3

import math
import sys
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.parameter import Parameter

from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float32

try:
    from gimbal_yaw_interfaces.msg import Float32Stamped  # type: ignore

    HAS_STAMPED = True
except Exception:
    Float32Stamped = None  # type: ignore
    HAS_STAMPED = False

import tf2_ros


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    # yaw (Z) from quaternion, consistent with REP-103 (ENU, Z-up)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_rad(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class TfToAutoAimYawPub(Node):
    def __init__(self) -> None:
        super().__init__("tf_to_auto_aim_yaw_pub")

        # Parameters (allow setting via --ros-args -p ...)
        self.declare_parameter("use_sim_time", True)
        self.declare_parameter("parent_frame", "chassis")
        self.declare_parameter("child_frame", "gimbal_yaw")
        self.declare_parameter("output_topic", "auto_aim_yaw")
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("use_stamped_msg", False)

        self.declare_parameter("scale", 1.0)
        self.declare_parameter("offset", 0.0)
        self.declare_parameter("invert_sign", False)

        # Ensure sim time is actually enabled
        use_sim_time = bool(self.get_parameter("use_sim_time").value)
        self.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, use_sim_time)])

        self.parent_frame = str(self.get_parameter("parent_frame").value)
        self.child_frame = str(self.get_parameter("child_frame").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        requested_stamped = bool(self.get_parameter("use_stamped_msg").value)

        self.scale = float(self.get_parameter("scale").value)
        self.offset = float(self.get_parameter("offset").value)
        self.invert_sign = bool(self.get_parameter("invert_sign").value)

        self.use_stamped_msg = bool(requested_stamped and HAS_STAMPED)
        if requested_stamped and not HAS_STAMPED:
            self.get_logger().warn(
                "use_stamped_msg=true requested, but gimbal_yaw_interfaces/msg/Float32Stamped is not available; "
                "falling back to std_msgs/msg/Float32"
            )

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        if self.use_stamped_msg:
            self.pub_stamped = self.create_publisher(Float32Stamped, self.output_topic, 10)  # type: ignore[arg-type]
            self.pub_plain = None
        else:
            self.pub_plain = self.create_publisher(Float32, self.output_topic, 10)
            self.pub_stamped = None

        period = 1.0 / max(1e-3, self.publish_rate_hz)
        self.timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f"publishing {self.output_topic} from TF {self.parent_frame} -> {self.child_frame} @ {self.publish_rate_hz:.1f}Hz"
        )

    def _lookup(self) -> Optional[TransformStamped]:
        try:
            # Latest available
            return self.tf_buffer.lookup_transform(
                self.parent_frame,
                self.child_frame,
                rclpy.time.Time(),
            )
        except Exception as ex:
            self.get_logger().debug(f"TF lookup failed: {ex}")
            return None

    def _on_timer(self) -> None:
        tf_msg = self._lookup()
        if tf_msg is None:
            return

        q = tf_msg.transform.rotation
        yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        yaw = normalize_rad(yaw)

        if self.invert_sign:
            yaw = -yaw
        yaw = yaw * self.scale + self.offset
        yaw = normalize_rad(yaw)

        if self.use_stamped_msg and self.pub_stamped is not None:
            msg = Float32Stamped()  # type: ignore[call-arg]
            msg.header = tf_msg.header
            msg.data = float(yaw)
            self.pub_stamped.publish(msg)
        elif self.pub_plain is not None:
            msg = Float32()
            msg.data = float(yaw)
            self.pub_plain.publish(msg)


def main() -> None:
    rclpy.init(args=sys.argv)
    node = TfToAutoAimYawPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
