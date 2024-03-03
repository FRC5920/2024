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
package frc.robot.subsystems.pivot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevice;
import frc.robot.Constants.RobotCANBus;
import org.littletonrobotics.junction.Logger;

/** Subsystem for controlling the pivot mechanism on the robot arm */
public class PivotSubsystem extends SubsystemBase {

  ////////////////////////////////////
  // CONSTANTS
  ////////////////////////////////////

  /** Set this to true to publish values to the dashboard */
  public static final boolean kPublishToDashboard = false;

  /** CAN bus used to communicate with the subsystem */
  public static final RobotCANBus kCANBus = RobotCANBus.Rio;

  ////////////////////////////////////
  // Pivot Motor Configuration
  ////////////////////////////////////

  /** CAN device ID of the pivot leader motor */
  public static final CANDevice kLeaderMotorDevice = CANDevice.PivotLeaderMotor;

  /** CAN device ID of the pivot follower motor */
  public static final CANDevice kFollowerMotorDevice = CANDevice.PivotFollowerMotor;

  /** CAN device ID of the pivot follower motor */
  public static final CANDevice kCANcoderDevice = CANDevice.PivotCANcoder;

  /** Gear ratio between the Falcon motors and the pivot axle */
  public static final double kFalconToPivotGearRatio = 40.0;

  /** Offset of the CANcoder magnet in rotations */
  public static final double kCANcoderMagnetOffsetRot = -0.950; // Measured 02/27/2024

  /** Maximum output (+/-) applied to the pivot motors (for safety) */
  public static final double kPeakPivotMotorOutputVoltage = 2.0;

  /** Minimum pivot angle in degrees */
  public static final double kMinPivotAngleDeg = 0.0;

  /** Maximum pivot angle in degrees */
  public static final double kMaxPivotAngleDeg = 178.5;

  ////////////////////////////////////
  // Attributes
  ////////////////////////////////////

  /** I/O used by the subsystem */
  private final PivotSubsystemIO m_io;

  /** Measured subsystem inputs */
  private final PivotSubsystemInputs m_inputs = new PivotSubsystemInputs();

  /** Creates a new PivotSubsystem. */
  public PivotSubsystem(PivotSubsystemIO io) {
    m_io = io;
    m_io.initialize();
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Sets the desired pivot angle in degrees
   *
   * @param degrees The desired pivot angle in degrees
   */
  public void setAngleDeg(double degrees) {
    // Clamp commanded angle
    degrees = (degrees < kMinPivotAngleDeg) ? kMinPivotAngleDeg : degrees;
    degrees = (degrees > kMaxPivotAngleDeg) ? kMaxPivotAngleDeg : degrees;

    m_inputs.leader.targetPosition = degrees;
    m_inputs.follower.targetPosition = degrees;
    m_io.setAngleDeg(degrees);
  }

  public enum PivotMotorID {
    Leader,
    Follower
  };

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /** Returns the current pivot angle in degrees */
  public double getAngleDeg() {
    return m_io.getAngleDeg();
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  @Override
  public void periodic() {
    // Update measurements
    m_io.processInputs(m_inputs);

    // Send input data to the logging framework (or update from the log during replay)
    Logger.processInputs("Pivot", m_inputs);

    if (kPublishToDashboard) {
      // Display velocities on dashboard
      SmartDashboard.putNumber("pivot/targetAngleDeg", m_inputs.leader.targetPosition);
      SmartDashboard.putNumber("pivot/motorAngleDeg", m_inputs.leader.position);
      SmartDashboard.putNumber("pivot/cancoderAngleRot", m_inputs.cancoderAngleRot);
      SmartDashboard.putNumber("pivot/cancoderAngleDeg", m_inputs.cancoderAngleDeg);
    }
  }
}
