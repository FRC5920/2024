////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2024 FIRST and other WPILib contributors.
// http://github.com/FRC5920
// Open Source Software; you can modify and/or share it under the terms of the
// license given in WPILib-License.md in the root directory of this project.
////////////////////////////////////////////////////////////////////////////////

/*-----------------------------------------------------------------------------\
|                                                                              |
|                       ================================                       |
|                       **    TEAM 5920 - Vikotics    **                       |
|                       ================================                       |
|                                                                              |
|                            °        #°                                       |
|                            *O       °@o                                      |
|                            O@ °o@@#° o@@                                     |
|                           #@@@@@@@@@@@@@@                                    |
|                           @@@@@@@@@@@@@@@                                    |
|                           @@@@@@@@@@@@@@°                                    |
|                             #@@@@@@@@@@@@@O....   .                          |
|                             o@@@@@@@@@@@@@@@@@@@@@o                          |
|                             O@@@@@@@@@@@@@@@@@@@#°                    *      |
|                             O@@@@@@@@@@@@@@@@@@@@@#O                O@@    O |
|                            .@@@@@@@@°@@@@@@@@@@@@@@@@#            °@@@    °@@|
|                            #@@O°°°°  @@@@@@@@@@@@@@@@@@°          @@@#*   @@@|
|                         .#@@@@@  o#oo@@@@@@@@@@@@@@@@@@@@@.       O@@@@@@@@@@|
|                        o@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@°     @@@@@@@@@°|
|                        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@   .@@@@@o°   |
|          °***          @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@  @@@@@o     |
|     o#@@@@@@@@@@@@.   *@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@o@@@@@@      |
|OOo°@@@@@@@@@@@@O°#@#   @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@       |
|@@@@@@@@@@@@@@@@    o°  .@@@@@@@@@@@@@@@@@@@@@@@@#*@@@@@@@@@@@@@@@@@@@@       |
|@@@@@@@@@@@@@@@*         O@@@@@@@@@@@@@@@@@@@@@@@   °@@@@@@@@@@@@@@@@@@o      |
|@@@@#@@@@@@@@@            @@@@@@@@@@@@@@@@@@@@@@       .*@@@@@@@@@@@@@@.      |
|@@@°      @@@@O           @@@@@@@@@@@@@@@@@@@@o           °@@@@@@@@@@@o       |
|          @@@@@          .@@@@@@@@@@@@@@@@@@@*               O@@@@@@@*        |
|           @@@@@        o@@@@@@@@@@@@@@@@@@@@.               #@@@@@O          |
|           *@@@@@@@*  o@@@@@@@@@@@@@@@@@@@@@@°              o@@@@@            |
|           @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.              @@@@@#            |
|          @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@O             #@@@@@             |
|          .@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@#           .@@@@@°             |
|           @@@@@@@@@@O*    @@@@@@@@@@@@@@@@@@@@@°         °O@@@°              |
|            °O@@@@@@       @@@@@@@@@@@@@@@@@@@@@@@                            |
|              o@@@@@°      @@@@@@@@@@@@@@@@@@@@@@@@                           |
|               @@@@@@.     @@@@@@@@@@@@@@@@@@@@@@@@@o                         |
|                @@@@@@*    @@@@@@@@@@@@@@@@@@@@@@@@@@                         |
|                o@@@@@@.  o@@@@@@@@@@@@@@@@@@@@@@@@@@@                        |
|                 #@@@@@@  *@@@@@@@@@@@@@@@@@@@@@@@@@@@@                       |
|                  °***    @@@@@@@@@@@@@@@@@@@@@@@@@@@@@O                      |
|                         .OOOOOOOOOOOOOOOOOOOOOOOOOOOOOO                      |
\-----------------------------------------------------------------------------*/
package frc.robot.subsystems.flywheel;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.logging.BotLog;
import frc.lib.logging.LoggableMotorInputs;
import frc.robot.Constants.CANDevice;
import frc.robot.Constants.RobotCANBus;
import org.littletonrobotics.junction.Logger;

// Reference Phoenix6 example:

/**
 * Subsystem for controlling the flywheel portion of the intake assembly
 *
 * @remarks Motor config is based on the CTRE Phoenix 5 library PositionClosedLoop example
 *     https://github.com/CrossTheRoadElec/Phoenix5-Examples/blob/master/Java%20General/PositionClosedLoop/src/main/java/frc/robot/Robot.java
 */
public class FlywheelSubsystem extends SubsystemBase {

  ////////////////////////////////////
  // CONSTANTS
  ////////////////////////////////////

  /** CAN bus used to communicate with the subsystem */
  public static final RobotCANBus kCANBus = RobotCANBus.Rio;

  ////////////////////////////////////
  // Flywheel Motor Configuration
  ////////////////////////////////////

  /** CAN device ID of the flywheel motor */
  public static final CANDevice kFlywheelMotorDevice = CANDevice.IntakeFlywheelMotor;

  /** Gear ratio between the flywheel motor and the flywheel mechanism */
  public static final double kFlywheelMotorGearRatio = 1.0 / 1.0;

  /** Set to true if the direction of the indexer motor should be reversed */
  public static final boolean kFlywheelMotorInverted = false;

  /** Maximum velocity (rotations per second) that the flywheel motor should run at */
  public static final double kMaxFlywheelMotorVelocity = 4000.0;

  /** Subsystem I/O to use */
  private final FlywheelSubsystemIO m_io;

  /** Logged subsystem inputs */
  private final LoggableMotorInputs m_inputs = new LoggableMotorInputs("");

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Creates a new instance of the subsystem and initializes its I/O
   *
   * @param io Subsystem I/O to use
   */
  public FlywheelSubsystem(FlywheelSubsystemIO io) {
    m_io = io;
    m_io.initialize();

    // By default, the subsystem will automatically stop the flywheel
    this.setDefaultCommand(
        run(
            () -> {
              if (Math.abs(m_inputs.targetVelocity) > 0.0) {
                BotLog.Debug("Flywheel auto-stop");
                this.setFlywheelVelocity(0.0);
              }
            }));
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Sets the desired velocity of the flywheel mechanism
   *
   * @param rotPerSec Desired velocity in rotations per second
   */
  public void setFlywheelVelocity(double rotPerSec) {
    m_inputs.targetVelocity = rotPerSec;
    m_io.setFlywheelVelocity(rotPerSec);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Returns the current velocity of the flywheel mechanism
   *
   * @return The velocity of the flywheel mechanism in rotations per second
   */
  public double getFlywheelVelocity() {
    return m_inputs.velocity;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  @Override
  public void periodic() {
    // Update measurements
    m_io.processInputs(m_inputs);

    // Send input data to the logging framework (or update from the log during replay)
    Logger.processInputs("Flywheel", m_inputs);

    // Display velocities on dashboard
    SmartDashboard.putNumber("Flywheel/setVelocity", m_inputs.targetVelocity);
    SmartDashboard.putNumber("Flywheel/velocity", m_inputs.velocity);
  }
}
