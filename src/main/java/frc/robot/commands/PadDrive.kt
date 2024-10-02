// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.SwerveSubsystem
import frc.robot.utils.GlobalsValues.MotorGlobalValues
import frc.robot.utils.LogitechGamingPad

class PadDrive(
  private val swerveSubsystem: SwerveSubsystem,
  private val pad: LogitechGamingPad,
  private val isFieldOriented: Boolean,
) : Command() {

  /** Creates a new SwerveJoystick. */
  init {
    addRequirements(this.swerveSubsystem)
  }

  // Called every time the scheduler runs while the command is scheduled.
  override fun execute() {
    val y = -pad.getLeftAnalogYAxis() * MotorGlobalValues.MAX_SPEED
    val x = -pad.getLeftAnalogXAxis() * MotorGlobalValues.MAX_SPEED
    val rotation = pad.getRightAnalogXAxis() * MotorGlobalValues.MAX_ANGULAR_SPEED

    SmartDashboard.putNumber("Y Joystick", y)
    SmartDashboard.putNumber("X Joystick", x)

    swerveSubsystem.getDriveSpeeds(y, x, rotation, isFieldOriented)
  }

  // Returns true when the command should end.
  override fun isFinished(): Boolean {
    return false
  }
}
