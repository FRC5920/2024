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
package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.logging.BotLog;
import frc.robot.Constants.CANDevice;
import frc.robot.Constants.RobotCANBus;
import org.littletonrobotics.junction.Logger;

// Reference Phoenix6 example:

/**
 * Subsystem for controlling the indexer assembly and gamepiece sensor
 *
 * @remarks Motor config is based on the CTRE Phoenix 5 library PositionClosedLoop example
 *     https://github.com/CrossTheRoadElec/Phoenix5-Examples/blob/master/Java%20General/PositionClosedLoop/src/main/java/frc/robot/Robot.java
 */
public class IndexerSubsystem extends SubsystemBase {

  ////////////////////////////////////
  // CONSTANTS
  ////////////////////////////////////

  /** CAN bus used to communicate with the subsystem */
  public static final RobotCANBus kCANBus = RobotCANBus.Rio;

  ////////////////////////////////////
  // Indexer Motor Configuration
  ////////////////////////////////////

  /** CAN device ID of the flywheel motor */
  public static final CANDevice kIndexerMotorDevice = CANDevice.IntakeIndexerMotor;

  /** Gear ratio between the indexer motor and the indexer mechanism */
  public static final double kIndexerMotorGearRatio = 10.0 / 1.0;

  /** Maximum speed (0.0 to 1.0) that the indexer should run at */
  public static final double kMaxIndexerSpeed = 1.0;

  /** Set to -1.0 to invert the direction of the indexer motor */
  public static final double kMotorInvert = -1.0;

  /** Subsystem I/O to use */
  private final IndexerSubsystemIO m_io;

  /** Logged subsystem inputs */
  private final IndexerSubsystemInputs m_inputs = new IndexerSubsystemInputs("");

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Creates a new instance of the subsystem and initializes its I/O
   *
   * @param io Subsystem I/O to use
   */
  public IndexerSubsystem(IndexerSubsystemIO io) {
    m_io = io;
    m_io.initialize();

    // By default, the subsystem will automatically stop the indexer
    this.setDefaultCommand(
        run(
            () -> {
              if (Math.abs(this.getIndexerSpeed()) > 0.0) {
                BotLog.Debug("Flywheel auto-stop");
                this.setIndexerSpeed(0.0);
              }
            }));
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Sets the desired speed of the indexer mechanism as a normalized percentage of full scale
   *
   * @param percent Normalized percentage of full speed (-1.0 to 1.0) negative values pull a
   *     gamepiece in; positive values push a gamepiece out
   */
  public void setIndexerSpeed(double percent) {
    percent = (percent > kMaxIndexerSpeed) ? (kMaxIndexerSpeed * Math.signum(percent)) : percent;
    m_inputs.indexer.targetVelocity = kMotorInvert * percent;
    m_io.setIndexerSpeed(percent);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Returns the current speed of the indexer mechanism as a percentage of full speed
   *
   * @return Normalized percentage of full speed (0.0 to 1.0) negative values pull a gamepiece in;
   *     positive values push a gamepiece out
   */
  public double getIndexerSpeed() {
    return m_inputs.indexer.velocity * kMotorInvert;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Returns true if a gamepiece is detected inside the intake assembly
   *
   * @return true if a gamepiece is detected inside the intake assembly; else false
   */
  public boolean gamepieceIsDetected() {
    return m_inputs.limitSwitch;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  @Override
  public void periodic() {
    // Update measurements
    m_io.processInputs(m_inputs);

    // Send input data to the logging framework (or update from the log during replay)
    Logger.processInputs("Indexer", m_inputs);

    // Display velocities on dashboard
    SmartDashboard.putNumber("Indexer/setSpeed", m_inputs.indexer.targetVelocity);
    SmartDashboard.putNumber("Indexer/speed", m_inputs.indexer.velocity);
    SmartDashboard.putBoolean("Indexer/limitSwitch", m_inputs.limitSwitch);
  }
}
