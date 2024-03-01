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
package frc.robot.subsystems.intake;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.LaserCan.RegionOfInterest;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.logging.BotLog;
import frc.robot.Constants.CANDevice;
import frc.robot.Constants.RobotCANBus;
import org.littletonrobotics.junction.Logger;

// Reference Phoenix6 example:

/**
 * Subsystem for controlling the extension/retraction of climber mechanisms
 *
 * @remarks Motor config is based on the CTRE Phoenix 5 library PositionClosedLoop example
 *     https://github.com/CrossTheRoadElec/Phoenix5-Examples/blob/master/Java%20General/PositionClosedLoop/src/main/java/frc/robot/Robot.java
 */
public class IntakeSubsystem extends SubsystemBase {

  ////////////////////////////////////
  // CONSTANTS
  ////////////////////////////////////

  /** CAN bus used to communicate with the subsystem */
  public static final RobotCANBus kCANBus = RobotCANBus.Rio;

  ////////////////////////////////////
  // Flywheel Motor Configuration
  ////////////////////////////////////

  /** CAN device ID of the flywheel motor */
  public static final CANDevice kFlywheelMotorCANDevice = CANDevice.IntakeFlywheelMotor;

  /** Gear ratio between the flywheel motor and the flywheel mechanism */
  public static final double kFlywheelMotorGearRatio = 1.0 / 1.0;

  /** Set to true if the direction of the indexer motor should be reversed */
  public static final boolean kFlywheelMotorInverted = false;

  /** Maximum velocity (rotations per second) that the flywheel motor should run at */
  public static final double kMaxFlywheelMotorVelocity = 4000.0;

  ////////////////////////////////////
  // Indexer Motor Configuration
  ////////////////////////////////////

  /** CAN device ID of the flywheel motor */
  public static final CANDevice kIndexerMotorCANDevice = CANDevice.IntakeIndexerMotor;

  /** Gear ratio between the indexer motor and the indexer mechanism */
  public static final double kIndexerMotorGearRatio = 10.0 / 1.0;

  /** Set to true if the direction of the indexer motor should be reversed */
  public static final boolean kIndexerMotorInverted = false;

  /** Maximum speed (0.0 to 1.0) that the indexer should run at */
  public static final double kMaxIndexerSpeed = 1.0;

  ////////////////////////////////////
  // LaserCAN Configuration
  ////////////////////////////////////

  /** CAN device ID of the LaserCAN module */
  public static final CANDevice kGamepieceSensorCANDevice = CANDevice.IntakeGamepieceSensor;

  /** LaserCAN ranging mode */
  public static final LaserCan.RangingMode kLaserCANRangingMode = LaserCan.RangingMode.SHORT;

  /** LaserCAN Region of Interest */
  public static final RegionOfInterest kLaserCANRegionOfInterest =
      new LaserCan.RegionOfInterest(8, 8, 16, 16);

  /** LaserCAN Timing budget */
  public static final LaserCan.TimingBudget kLaserCANTimingBudget =
      LaserCan.TimingBudget.TIMING_BUDGET_33MS;

  /** Subsystem I/O to use */
  private final IntakeSubsystemIO m_io;

  /** Logged subsystem inputs */
  private final IntakeSubsystemInputs m_inputs = new IntakeSubsystemInputs("");

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Creates a new instance of the subsystem and initializes its I/O
   *
   * @param io Subsystem I/O to use
   */
  public IntakeSubsystem(IntakeSubsystemIO io) {
    m_io = io;
    m_io.initialize();
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Sets the desired speed of the indexer mechanism as a normalized percentage of full scale
   *
   * @param percent Normalized percentage of full speed (0.0 to 1.0)
   */
  public void setIndexerSpeed(double percent) {
    m_inputs.indexer.targetVelocity = percent;
    m_io.setIndexerSpeed(percent);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Sets the desired velocity of the flywheel mechanism
   *
   * @param rotPerSec Desired velocity in rotations per second
   */
  public void setFlywheelVelocity(double rotPerSec) {
    m_inputs.flywheel.targetVelocity = rotPerSec;
    m_io.setFlywheelVelocity(rotPerSec);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Returns the current speed of the indexer mechanism as a percentage of full speed
   *
   * @return Normalized percentage of full speed (0.0 to 1.0)
   */
  public double getIndexerSpeed() {
    double percent = m_io.getIndexerSpeed();
    return percent;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Returns the current velocity of the flywheel mechanism
   *
   * @return The velocity of the flywheel mechanism in rotations per second
   */
  public double getFlywheelVelocity() {
    double rotPerSec = m_io.getFlywheelVelocity();
    return rotPerSec;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Returns the distance measured by the gamepiece sensor
   *
   * @return The distance measured by the gamepiece sensor in meters
   */
  public double getGamepieceDistance() {
    return m_io.getGamepieceDistance();
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  @Override
  public void periodic() {
    // Update measurements
    m_io.processInputs(m_inputs);

    // Send input data to the logging framework (or update from the log during replay)
    Logger.processInputs("Intake", m_inputs);

    // Display velocities on dashboard
    SmartDashboard.putNumber("intake/flywheel/setVelocity", m_inputs.flywheel.targetVelocity);
    SmartDashboard.putNumber("intake/flywheel/velocity", m_inputs.flywheel.velocity);
    SmartDashboard.putNumber("intake/indexer/setSpeed", m_inputs.indexer.targetVelocity);
    SmartDashboard.putNumber("intake/indexer/speed", m_inputs.indexer.velocity);
    SmartDashboard.putNumber("intake/laserCAN/distance", m_inputs.laserCAN.distanceMeters);
    SmartDashboard.putString("intake/laserCAN/status", m_inputs.laserCAN.status);
  }

  public enum IntakePreset {
    IntakeRing(10.0, 0.5),
    ShootRing(-3000.0, -0.5),
    ShootNoteAmp(-1000, -0.5),
    ShootNoteSpeaker(-3000, -0.5);

    public final double flywheelRPS;
    public final double indexerSpeed;

    private IntakePreset(double flywheelRPS, double indexerSpeed) {
      this.flywheelRPS = flywheelRPS;
      this.indexerSpeed = indexerSpeed;
    }
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  public static class RunIntakeAtSpeed extends Command {
    private final IntakeSubsystem m_intakeSubsystem;
    private final IntakePreset m_preset;

    /** Creates a new instance of the command */
    public RunIntakeAtSpeed(IntakeSubsystem intakeSubsystem, IntakePreset preset) {
      m_intakeSubsystem = intakeSubsystem;
      m_preset = preset;
      addRequirements(m_intakeSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      m_intakeSubsystem.setFlywheelVelocity(m_preset.flywheelRPS);
      m_intakeSubsystem.setIndexerSpeed(m_preset.indexerSpeed);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      m_intakeSubsystem.setFlywheelVelocity(0.0);
      m_intakeSubsystem.setIndexerSpeed(0.0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
  }
  //////////////////////////////////////////////////////////////////////////////////////////////////
  public static class RunFlywheelAtSpeed extends Command {
    private final IntakeSubsystem m_intakeSubsystem;
    private final IntakePreset m_preset;

    /** Maximum time to run the flywheel */
    private final double m_timeoutSec;

    /** Used to implement timeout */
    private final Timer m_timer;

    /**
     * Creates an instance of the command
     *
     * @param intakeSubsystem The intake subsystem to operate on
     * @param preset Preset giving the speed to run the flywheel at
     * @param seconds Maximum number of seconds to run the flywheel
     */
    public RunFlywheelAtSpeed(
        IntakeSubsystem intakeSubsystem, IntakePreset preset, double seconds) {
      m_intakeSubsystem = intakeSubsystem;
      m_preset = preset;
      m_timeoutSec = seconds;
      m_timer = new Timer();
      addRequirements(m_intakeSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      m_intakeSubsystem.setFlywheelVelocity(m_preset.flywheelRPS);
      m_timer.restart();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted to shut off the flywheel
    @Override
    public void end(boolean interrupted) {
      if (interrupted) {
        BotLog.Debugf("RunFlywheelAtSpeed command was interrupted");
      }
    }

    // Returns true when the flywheel has reached the target speed within a capture range
    @Override
    public boolean isFinished() {
      double velocity = m_intakeSubsystem.getFlywheelVelocity();
      boolean timedOut = m_timer.hasElapsed(m_timeoutSec);

      // NOTE: flywheel velocity is signed.  We are only concerned about magnitude here.
      if (Math.abs(velocity) > Math.abs(m_preset.flywheelRPS)) {
        BotLog.Debugf("RunFlywheelAtSpeed finished: flywheel velocity=%f", velocity);
        return true;
      }

      if (timedOut) {
        BotLog.Debugf(
            "RunFlywheelAtSpeed timed out after %f seconds: velocity=%f", m_timeoutSec, velocity);
        return true;
      }

      return false;
    }
  }
  //////////////////////////////////////////////////////////////////////////////////////////////////
  public static class RunIndexerAtSpeed extends Command {
    private final IntakeSubsystem m_intakeSubsystem;
    private final IntakePreset m_preset;

    /** Maximum time to run the flywheel */
    private final double m_timeoutSec;

    /** Used to implement timeout */
    private final Timer m_timer;

    /**
     * Creates an instance of the command
     *
     * @param intakeSubsystem The intake subsystem to operate on
     * @param preset Preset giving the speed to run the flywheel at
     * @param seconds Maximum number of seconds to run the flywheel
     */
    public RunIndexerAtSpeed(IntakeSubsystem intakeSubsystem, IntakePreset preset, double seconds) {
      m_intakeSubsystem = intakeSubsystem;
      m_preset = preset;
      m_timeoutSec = seconds;
      m_timer = new Timer();
      addRequirements(m_intakeSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      m_intakeSubsystem.setIndexerSpeed(m_preset.indexerSpeed);
      m_timer.restart();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      m_intakeSubsystem.setIndexerSpeed(0.0);
      if (interrupted) {
        BotLog.Debugf("RunIndexerAtSpeed command was interrupted");
      }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      boolean timedOut = m_timer.hasElapsed(m_timeoutSec);
      if (timedOut) {
        BotLog.Debugf("RunIndexerAtSpeed completed after %f seconds", m_timeoutSec);
        return true;
      }

      return timedOut;
    }
  }
}
