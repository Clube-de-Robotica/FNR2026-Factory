#!/usr/bin/env python3
"""
mecanum_driver.py
=================
ROS 2 Jazzy node for a 4-wheel mecanum robot.

Subscriptions
-------------
  /cmd_vel              geometry_msgs/Twist   – desired robot velocity
  /motor{0-3}/encoder   std_msgs/Int16        – accumulated encoder ticks

Publications
------------
  /motor{0-3}/cmd       std_msgs/Int16        – PWM command  [-255, 255]
  /odom                 nav_msgs/Odometry     – dead-reckoning odometry
  /tf                   (odom → base_link)    – TF broadcast

Wheel layout (top view)
-----------------------
        FRONT
   FL(0)     FR(1)
   RL(2)     RR(3)
        REAR

Mecanum roller convention: front-left & rear-right have rollers at +45°,
front-right & rear-left have rollers at −45°.

Parameters (all settable via ROS 2 parameter server)
-----------------------------------------------------
  wheel_radius        [m]     0.0325   (Ø65 mm wheel)
  wheel_separation_x  [m]     0.160    half track-width  (left ↔ right)
  wheel_separation_y  [m]     0.140    half wheelbase    (front ↔ rear)
  max_motor_speed     [rad/s] 10.0     speed that maps to PWM ±255
  encoder_ppr         [-]     234.3    pulses per wheel revolution
  odom_frame_id       [str]   "odom"
  base_frame_id       [str]   "base_link"
  publish_tf          [bool]  True
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16
from tf2_ros import TransformBroadcaster


# ---------------------------------------------------------------------------
# Helper
# ---------------------------------------------------------------------------

def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class MecanumDriverNode(Node):

    # ---- encoder constants ------------------------------------------------
    MOTOR_ENCODER_PPR = 11       # encoder pulses per motor-shaft revolution
    GEAR_RATIO        = 21.3     # gearbox reduction
    WHEEL_PPR         = MOTOR_ENCODER_PPR * GEAR_RATIO   # = 234.3 pulses / wheel rev
    TWO_PI            = 2.0 * math.pi

    def __init__(self):
        super().__init__('mecanum_driver')

        # ------------------------------------------------------------------ #
        # Parameters                                                           #
        # ------------------------------------------------------------------ #
        self._declare_params()

        r   = self.get_parameter('wheel_radius').value          # [m]
        lx  = self.get_parameter('wheel_separation_x').value    # half track-width [m]
        ly  = self.get_parameter('wheel_separation_y').value    # half wheelbase   [m]
        self._max_speed  = self.get_parameter('max_motor_speed').value   # [rad/s]
        self._wheel_ppr  = self.get_parameter('encoder_ppr').value
        self._odom_frame = self.get_parameter('odom_frame_id').value
        self._base_frame = self.get_parameter('base_frame_id').value
        self._pub_tf     = self.get_parameter('publish_tf').value

        self._r  = r
        self._lx = lx
        self._ly = ly
        self._l  = lx + ly          # combined for mecanum kinematics

        self.get_logger().info(
            f"Mecanum driver initialised  r={r:.4f} m  "
            f"lx={lx:.3f} m  ly={ly:.3f} m  "
            f"PPR={self._wheel_ppr:.1f}"
        )

        # ------------------------------------------------------------------ #
        # State                                                                #
        # ------------------------------------------------------------------ #
        # Encoder: last raw tick values (Int16, wraps at ±32767)
        self._enc_last   = [None, None, None, None]   # None = not yet received
        self._enc_delta  = [0.0,  0.0,  0.0,  0.0]   # wheel revolutions since last odom update

        # Odometry pose
        self._x   = 0.0
        self._y   = 0.0
        self._yaw = 0.0

        self._last_enc_time = None

        # ------------------------------------------------------------------ #
        # Publishers                                                           #
        # ------------------------------------------------------------------ #
        self._motor_pubs = [
            self.create_publisher(Int16, f'/motor{i}/cmd', 10)
            for i in range(4)
        ]
        self._odom_pub = self.create_publisher(Odometry, '/odom', 10)

        if self._pub_tf:
            self._tf_broadcaster = TransformBroadcaster(self)

        # ------------------------------------------------------------------ #
        # Subscribers                                                          #
        # ------------------------------------------------------------------ #
        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb, 10)

        for i in range(4):
            self.create_subscription(
                Int16,
                f'/motor{i}/encoder',
                lambda msg, idx=i: self._encoder_cb(msg, idx),
                10
            )

        # ------------------------------------------------------------------ #
        # Odom publish timer (20 Hz)                                           #
        # ------------------------------------------------------------------ #
        self._odom_timer = self.create_timer(0.05, self._publish_odom)

        self.get_logger().info("mecanum_driver node ready.")

    # ======================================================================= #
    # Parameter declaration                                                    #
    # ======================================================================= #

    def _declare_params(self):
        def _d(desc): return ParameterDescriptor(description=desc)

        self.declare_parameter('wheel_radius',        0.0325, _d('Wheel radius [m]'))
        self.declare_parameter('wheel_separation_x',  0.160,  _d('Half track-width – left to right centre [m]'))
        self.declare_parameter('wheel_separation_y',  0.140,  _d('Half wheelbase – front to rear centre [m]'))
        self.declare_parameter('max_motor_speed',     10.0,   _d('Wheel angular speed [rad/s] that maps to PWM 255'))
        self.declare_parameter('encoder_ppr',         234.3,  _d('Encoder pulses per wheel revolution (11 × 21.3)'))
        self.declare_parameter('odom_frame_id',       'odom', _d('Odometry frame id'))
        self.declare_parameter('base_frame_id', 'base_link',  _d('Robot base frame id'))
        self.declare_parameter('publish_tf',           True,  _d('Broadcast odom→base_link TF'))

    # ======================================================================= #
    # cmd_vel callback  →  motor commands                                      #
    # ======================================================================= #

    def _cmd_vel_cb(self, msg: Twist):
        """
        Mecanum inverse kinematics
        --------------------------
        w_FL =  (vx - vy - l·ωz) / r
        w_FR =  (vx + vy + l·ωz) / r
        w_RL =  (vx + vy - l·ωz) / r
        w_RR =  (vx - vy + l·ωz) / r

        where l = lx + ly  (combined half-distance)
        """
        vx  = msg.linear.x
        vy  = msg.linear.y
        wz  = msg.angular.z

        r  = self._r
        l  = self._l

        # Angular velocities [rad/s] for each wheel
        w = [
            (vx - vy - l * wz) / r,   # FL  (motor 0)
            (vx + vy + l * wz) / r,   # FR  (motor 1)
            (vx + vy - l * wz) / r,   # RL  (motor 2)
            (vx - vy + l * wz) / r,   # RR  (motor 3)
        ]

        # Scale to [-255, 255] and publish
        for i, wi in enumerate(w):
            pwm = wi / self._max_speed * 255.0
            pwm_int = int(clamp(round(pwm), -255.0, 255.0))
            cmd_msg = Int16()
            cmd_msg.data = pwm_int
            self._motor_pubs[i].publish(cmd_msg)

    # ======================================================================= #
    # Encoder callbacks                                                        #
    # ======================================================================= #

    def _encoder_cb(self, msg: Int16, idx: int):
        """
        Accumulate wheel displacement from incremental encoder ticks.

        Int16 wraps at ±32767; we handle rollover by assuming the inter-sample
        displacement is always < 16384 ticks (half the range).
        """
        now = self.get_clock().now()
        raw = msg.data  # signed 16-bit, accumulated on the hardware side

        if self._enc_last[idx] is None:
            self._enc_last[idx] = raw
            self._last_enc_time = now
            return

        # Compute delta with Int16 rollover compensation
        diff = raw - self._enc_last[idx]
        # Rollover handling for signed 16-bit arithmetic
        if diff > 32767:
            diff -= 65536
        elif diff < -32768:
            diff += 65536

        self._enc_last[idx] = raw

        # Convert ticks → radians
        delta_rad = (diff / self._wheel_ppr) * self.TWO_PI
        self._enc_delta[idx] += delta_rad

        self._last_enc_time = now

    # ======================================================================= #
    # Odometry publish                                                         #
    # ======================================================================= #

    def _publish_odom(self):
        """
        Mecanum forward kinematics from wheel displacements:
        -------------------------------------------------------
        Δvx  = r/4 · ( Δw_FL + Δw_FR + Δw_RL + Δw_RR )
        Δvy  = r/4 · (-Δw_FL + Δw_FR + Δw_RL - Δw_RR )
        Δωz  = r/(4·l) · (-Δw_FL + Δw_FR - Δw_RL + Δw_RR )
        """
        # Consume accumulated wheel displacements [rad]
        dw = self._enc_delta[:]
        self._enc_delta = [0.0, 0.0, 0.0, 0.0]

        fl, fr, rl, rr = dw          # angular displacement [rad]

        r = self._r
        l = self._l

        d_x_body = r / 4.0 * ( fl + fr + rl + rr)
        d_y_body = r / 4.0 * (-fl + fr + rl - rr)
        d_yaw    = r / (4.0 * l) * (-fl + fr - rl + rr)

        # Integrate in world frame
        cos_yaw = math.cos(self._yaw + d_yaw / 2.0)
        sin_yaw = math.sin(self._yaw + d_yaw / 2.0)

        self._x   += d_x_body * cos_yaw - d_y_body * sin_yaw
        self._y   += d_x_body * sin_yaw + d_y_body * cos_yaw
        self._yaw += d_yaw

        # Normalise yaw to (-π, π]
        self._yaw = math.atan2(math.sin(self._yaw), math.cos(self._yaw))

        now = self.get_clock().now().to_msg()

        # ---- Odometry message -------------------------------------------- #
        odom = Odometry()
        odom.header.stamp    = now
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id  = self._base_frame

        # Pose
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.position.z = 0.0

        half_yaw = self._yaw / 2.0
        odom.pose.pose.orientation.z = math.sin(half_yaw)
        odom.pose.pose.orientation.w = math.cos(half_yaw)

        # Twist (body-frame velocities) – approximate from last delta / dt
        dt = 0.05  # matches timer period; refine with actual timestamps if needed
        odom.twist.twist.linear.x  = d_x_body / dt
        odom.twist.twist.linear.y  = d_y_body / dt
        odom.twist.twist.angular.z = d_yaw    / dt

        # Covariance (diagonal, conservative estimates)
        # Indices: [x, y, z, roll, pitch, yaw]  →  6×6 row-major
        pose_cov = [0.0] * 36
        pose_cov[0]  = 1e-3   # x
        pose_cov[7]  = 1e-3   # y
        pose_cov[35] = 1e-2   # yaw
        odom.pose.covariance = pose_cov

        twist_cov = [0.0] * 36
        twist_cov[0]  = 1e-3   # vx
        twist_cov[7]  = 1e-3   # vy
        twist_cov[35] = 1e-2   # ωz
        odom.twist.covariance = twist_cov

        self._odom_pub.publish(odom)

        # ---- TF broadcast ------------------------------------------------- #
        if self._pub_tf:
            tf_msg = TransformStamped()
            tf_msg.header.stamp    = now
            tf_msg.header.frame_id = self._odom_frame
            tf_msg.child_frame_id  = self._base_frame
            tf_msg.transform.translation.x = self._x
            tf_msg.transform.translation.y = self._y
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation.z    = math.sin(half_yaw)
            tf_msg.transform.rotation.w    = math.cos(half_yaw)
            self._tf_broadcaster.sendTransform(tf_msg)


# ---------------------------------------------------------------------------
# Entry-point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = MecanumDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()