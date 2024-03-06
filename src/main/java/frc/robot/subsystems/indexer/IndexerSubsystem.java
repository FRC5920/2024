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

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.LaserCan.RegionOfInterest;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

  ////////////////////////////////////
  // LaserCAN Configuration
  ////////////////////////////////////

  /** CAN device ID of the flywheel motor */
  public static final CANDevice kLaserCANDevice = CANDevice.IntakeGamepieceSensor;

  /** LaserCAN ranging mode */
  public static final LaserCan.RangingMode kLaserCANRangingMode = LaserCan.RangingMode.SHORT;

  /** LaserCAN Region of Interest */
  public static final RegionOfInterest kLaserCANRegionOfInterest =
      new LaserCan.RegionOfInterest(8, 8, 16, 16);

  /** LaserCAN Timing budget */
  public static final LaserCan.TimingBudget kLaserCANTimingBudget =
      LaserCan.TimingBudget.TIMING_BUDGET_33MS;

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
   * Returns the distance measured by the gamepiece sensor
   *
   * @return The distance measured by the gamepiece sensor in meters
   */
  public double getGamepieceDistance() {
    return (m_inputs.laserCAN.isValid) ? m_inputs.laserCAN.distanceMeters : -1.0;
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
    SmartDashboard.putNumber("Indexer/laserCAN/distance", m_inputs.laserCAN.distanceMeters);
    SmartDashboard.putString("Indexer/laserCAN/status", m_inputs.laserCAN.status);
  }

  public enum IndexerPreset {
    IntakeRing(-0.5),
    ShootRing(0.5);

    public final double indexerSpeed;

    private IndexerPreset(double indexerSpeed) {
      this.indexerSpeed = indexerSpeed;
    }
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  public static class RunIndexerAtSpeed extends Command {
    private final IndexerSubsystem m_intakeSubsystem;
    private final IndexerPreset m_preset;

    /** Creates a new ClimberJoystickTeleOp. */
    public RunIndexerAtSpeed(IndexerSubsystem intakeSubsystem, IndexerPreset preset) {
      m_intakeSubsystem = intakeSubsystem;
      m_preset = preset;
      addRequirements(m_intakeSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      m_intakeSubsystem.setIndexerSpeed(m_preset.indexerSpeed);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      m_intakeSubsystem.setIndexerSpeed(0.0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
  }
}
