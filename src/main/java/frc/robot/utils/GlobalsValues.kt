@file:Suppress("unused")

package frc.robot.utils

import com.ctre.phoenix6.signals.InvertedValue
import com.pathplanner.lib.util.HolonomicPathFollowerConfig
import com.pathplanner.lib.util.PIDConstants
import com.pathplanner.lib.util.ReplanningConfig
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveDriveKinematics

object GlobalsValues {

  object MotorGlobalValues {
    // Motor CAN ID Values
    const val FRONT_LEFT_STEER_ID = 1
    const val FRONT_LEFT_DRIVE_ID = 2
    const val FRONT_RIGHT_STEER_ID = 3
    const val FRONT_RIGHT_DRIVE_ID = 4
    const val BACK_LEFT_STEER_ID = 5
    const val BACK_LEFT_DRIVE_ID = 6
    const val BACK_RIGHT_STEER_ID = 7
    const val BACK_RIGHT_DRIVE_ID = 8
    const val FRONT_LEFT_CAN_CODER_ID = 9
    const val FRONT_RIGHT_CAN_CODER_ID = 10
    const val BACK_LEFT_CAN_CODER_ID = 11
    const val BACK_RIGHT_CAN_CODER_ID = 12

    const val PIDGEY_ID = 16

    // Motor Property Values
    const val MAX_SPEED = 5.76
    const val MAX_ANGULAR_SPEED = (14 * Math.PI) / 3
    const val ENCODER_COUNTS_PER_ROTATION = 1.0 // 2048 for v5, 1 for v6 (rotations)
    const val STEER_MOTOR_GEAR_RATIO = 150.0 / 7 // 24
    const val DRIVE_MOTOR_GEAR_RATIO = 5.9
    const val WHEEL_DIAMETER = 0.106
    const val SPEED_CONSTANT = 0.6 // 0.4
    const val AACORN_SPEED = 0.95
    const val SLOW_SPEED = 0.3
    const val TURN_CONSTANT = 0.3 // 0.3
    const val METERS_PER_REVOLUTION = WHEEL_DIAMETER * Math.PI
    var HEADING = 0.0

    // Motor Speed Manipulation Values
    var SLOW_MODE: Boolean = false
    var AACORN_MODE: Boolean = true
  }

  object SwerveGlobalValues {
    object BasePIDGlobal {
      val STEER_PID = PID(13.0, 0.00085, 0.008, 0.0)
      val DRIVE_PID = PID(0.7, 0.0, 0.0)

      val HORIZONTAL_PID = PID(0.05, 0.075, 0.03, 0.0)
      val VERTICAL_PID = PID(0.25, 0.0085, 0.03)
      val ROTATIONAL_PID = PID(0.05509, 0.00, 0.09, 0.0)

      var pathFollower: HolonomicPathFollowerConfig =
        HolonomicPathFollowerConfig(
          PIDConstants(0.15, 0.0001, 0.00), // translation
          PIDConstants(0.1, 0.0, 0.0), // rotation
          4.96824, // Max module speed, in m/s
          ROBOT_SIZE, // Drive base radius in meters. Distance from robot center to the furthest module.
          ReplanningConfig(false, false),
        )
    }

    const val ROBOT_SIZE = 0.43105229381 // Keep constant *ideally*

    // Motor Locations (Relative to the center in meters)
    val FRONT_LEFT: Translation2d = Translation2d(0.3048, -0.3048)
    val FRONT_RIGHT: Translation2d = Translation2d(0.3048, 0.3048)
    val BACK_LEFT: Translation2d = Translation2d(-0.3048, -0.3048)
    val BACK_RIGHT: Translation2d = Translation2d(-0.3048, 0.3048)
    val kinematics: SwerveDriveKinematics =
      SwerveDriveKinematics(FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT)

    const val STATE_SPEED_THRESHOLD = 0.05

    // The values of the can coders when the wheels are straight according to Mr. Wright
    const val CANCODER_VALUE9 = -0.419189
    const val CANCODER_VALUE10 = -0.825928 - 0.5
    const val CANCODER_VALUE11 = -0.475098
    const val CANCODER_VALUE12 = -0.032959 + 0.5

    // Whether the motors are inverted
    val DRIVE_MOTOR_INVERETED: InvertedValue = InvertedValue.CounterClockwise_Positive
    val STEER_MOTOR_INVERTED: InvertedValue = InvertedValue.Clockwise_Positive

    // The deadband of the joystick to combat drift
    const val JOYSTICK_DEADBAND = 0.05

    const val USING_VISION: Boolean = false
    const val FIELD_ORIENTATED: Boolean = false

    // Whether the limelight auto aligns and its deadband
    const val AUTO_ALIGN: Boolean = false
    const val LIMELIGHT_DEADBAND = 3.0

    const val MOTOR_DEADBAND = 0.05
    const val IS_FIELD_ORIENTATED: Boolean = true
    const val ENCODER_OFFSET = (0 / 360).toDouble()

    // RGB Values for LED
    val GREEN_LEDArray = intArrayOf(0, 255, 0)
    val ORANGE_LEDArray = intArrayOf(255, 165, 0)
    val HIGHTIDE_LEDArray = intArrayOf(0, 182, 174)

    const val OFF_BALANCE_ANGLE_THRESHOLD = 10.0
    const val ON_BALANCE_ANGLE_THRESHOLD = 5.0
  }

  object IntakeGlobalValues {
    // Intake Motor Values
    const val IS_INVERTED: Boolean = false

    const val INTAKE_MOTOR_ID = 17

    const val INTAKE_SPEED = 50.0

    // Intake PID Values
    const val INTAKE_PID_V = 0.1
    const val INTAKE_PID_P = 0.0002
    const val INTAKE_PID_I = 0.0
    const val INTAKE_PID_D = 0.0

    // Reverse Intake Values
    const val REVERSE_INTAKE_SPEED = 20.0
  }

  object PivotGlobalValues {
    // Pivot Softlimit Values
    var is_SOFTLIMIT = true
    var offset = 0.0
    var manual_offset = 1045.0

    // Pivot Motor Values
    const val IS_INVERTED = false
    const val PIVOT_GEAR_RATIO = 198.0

    // Pivot Motor CAN ID Values
    const val PIVOT_MOTOR_LEFT_ID = 14
    const val PIVOT_MOTOR_RIGHT_ID = 15

    // Pivot PID Values
    const val PIVOT_PID_LEFT_P = 0.3
    const val PIVOT_PID_LEFT_I = 0.0005
    const val PIVOT_PID_LEFT_D = 0.0
    const val PIVOT_PID_LEFT_V = 0.5
    const val PIVOT_PID_LEFT_F = 0.1

    const val PIVOT_PID_RIGHT_P = 0.3
    const val PIVOT_PID_RIGHT_I = 0.0005
    const val PIVOT_PID_RIGHT_D = 0.0
    const val PIVOT_PID_RIGHT_V = 0.5
    const val PIVOT_PID_RIGHT_F = 0.1

    val PIVOT_NEUTRAL_ANGLE = (0.5 + offset)
    const val PIVOT_AMP_ANGLE = 50.2
    val PIVOT_SUBWOOFER_ANGLE = (13 + offset)
    const val PIVOT_SOURCE = 93.0
    val PIVOT_FENDER_ANGLE = (305 + offset)
    val PIVOT_MAX_ANGLE = (1000 + offset)
    val PIVOT_MIN_ANGLE = (0 + offset)

    // Pivot Motor Speed Values
    var IS_NEUTRAL: Boolean = true
    var soft_limit_enabled: Boolean = true

    // Pivot Motor Encoder ID
    const val ENCODER_ID = 9
  }

  object ShooterGlobalValues {
    // Shooter CAN ID Values
    const val FALCON_LEFT_ID = 18
    const val FALCON_RIGHT_ID = 13

    const val KRAKEN_ID = 20

    // Shooter PID Values
    const val SHOOTER_PID_LEFT_P = 0.0002
    const val SHOOTER_PID_LEFT_I = 0.0
    const val SHOOTER_PID_LEFT_D = 0.0
    const val SHOOTER_PID_LEFT_V = 0.5

    const val SHOOTER_PID_RIGHT_P = 0.0002
    const val SHOOTER_PID_RIGHT_I = 0.0
    const val SHOOTER_PID_RIGHT_D = 0.0
    const val SHOOTER_PID_RIGHT_V = 0.5

    const val KRAKEN_P = 0.0002
    const val KRAKEN_I = 0.0
    const val KRAKEN_D = 0.0
    const val KRAKEN_V = 0.1

    // Shooter Motor Speed Values
    const val SHOOTER_SPEED = 35.0
    var AMP_SPEED = 9.87
    var AMP_SPEED_JESSICA: Boolean = true
    const val KRAKEN_SPEED = 15.0

    // Shooter Motor Speeds
    const val SHOOTER_RPS = -40.0 // Positive is shooting, negative is intake
    const val STAGE_PASS_RPS = -10.0
    const val PASSTHROUGH_RPS = -30.0 // Positive is shooting, negative is intake
    const val AUTO_PASSTHROUGH_RPS = -35.0 // Positive is shooting, negative is intake
    const val PUSH_RING_AMP_RPS = -30.0
    const val PUSH_RING_RPS = -30.0

    // Shooter Misc Values
    var HAS_PIECE: Boolean = false
    var SHOOTING_DEADBAND = 0.3
    var RPM_THRESHOLD = 25.0

    const val RING_SENSOR_PORT = 8
  }

  object LimelightGlobalValues {
    // Offset Values
    var tx = 0.0
    var ty = 0.0
    var ta = 0.0
    var tv = 0.0

    // Limelight Misc Values
    var robotPoseTargetSpaceArray = DoubleArray(6)
    var tagIDAvailable = 0.0

    var hasTarget: Boolean = false

    var distance = 0.0
  }

  object PhotonVisionConstants {
    // Offset Values
    var tx = 0.0
    var ty = 0.0
    var ta = 0.0
    var tv = 0.0
    // Camera One
    const val CAMERA_ONE_HEIGHT = 0.0
    const val CAMERA_ONE_ANGLE = 0.0 // up is positive
    // Camera Two
    const val CAMERA_TWO_HEIGHT = 0.0
    const val CAMERA_TWO_ANGLE = 0.0 // up is positive
  }
}