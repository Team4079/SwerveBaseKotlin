// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import java.util.ArrayList

/**
* The Robot class extends the TimedRobot class and serves as the main entry point for the robot code.
* It contains methods that are called periodically and during different robot modes.
*/
class Robot : TimedRobot() {
  private var autonomousCommand: Command? = null
  private var robotContainer: RobotContainer? = null
  private val timer = Timer()
  private val avgTimer = Timer()
  private var timesRan = 0
  private val timesRanArr = ArrayList<Int?>()

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  override fun robotInit() {
    // Start the timers and instantiate the RobotContainer
    avgTimer.start()
    timer.start()
    robotContainer = RobotContainer()
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * This runs after the mode specific periodic functions, but before LiveWindow and SmartDashboard
   * integrated updating.
   */
  override fun robotPeriodic() {
    // Run the CommandScheduler
    CommandScheduler.getInstance().run()
    timesRan++

    // Check if 1 second has passed
    if (timer.advanceIfElapsed(1.0)) {
      println("1 second has passed, times ran is $timesRan")
      SmartDashboard.putNumber("Hurtz Ketchup", timesRan.toDouble())
      timesRanArr.add(timesRan)
      timesRan = 0
    }

    // Check if 15 seconds have passed
    if (avgTimer.advanceIfElapsed(15.0)) {
      val avgTimesRan =
        timesRanArr
          .stream()
          .mapToInt { a: Int? -> a!! }
          .average()
          .orElse(404.0) // Something's wrong
      println("15 seconds have passed, times ran is $avgTimesRan")
      SmartDashboard.putNumber("Hurtz Ketchup Average", avgTimesRan)
      timesRanArr.clear()
    }
  }

  /** This function is called once each time the robot enters Disabled mode. */
  override fun disabledInit() = Unit

  /** This function is called periodically during Disabled mode. */
  override fun disabledPeriodic() = Unit

  /** This autonomous runs the autonomous command selected by your [RobotContainer] class. */
  override fun autonomousInit() {
    // Retrieve the autonomous command from the RobotContainer
    autonomousCommand = robotContainer!!.getAutonomousCommand()

    // Schedule the autonomous command if it is not null
    if (autonomousCommand != null) {
      autonomousCommand!!.schedule()
    }
  }

  /** This function is called periodically during autonomous. */
  override fun autonomousPeriodic() = Unit

  /**
   * This function is called once when teleop mode is initialized.
   * It ensures that the autonomous command is cancelled when teleop starts.
   */
  override fun teleopInit() {
    // Cancel the autonomous command if it is running
    if (autonomousCommand != null) {
      autonomousCommand!!.cancel()
    }
  }

  /** This function is called periodically during operator control. */
  override fun teleopPeriodic() = Unit

  /**
   * This function is called once when test mode is initialized.
   * It cancels all running commands at the start of test mode.
   */
  override fun testInit() {
    // Cancel all running commands
    CommandScheduler.getInstance().cancelAll()
  }

  /** This function is called periodically during test mode. */
  override fun testPeriodic() = Unit

  /** This function is called once when the robot is first started up in simulation. */
  override fun simulationInit() = Unit

  /** This function is called periodically whilst in simulation. */
  override fun simulationPeriodic() = Unit
}
