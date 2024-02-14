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

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.utility.Alert;
import frc.lib.utility.Phoenix5Util;
import frc.lib.utility.Phoenix5Util.Sensor;
import frc.lib.utility.Phoenix5Util.SensorMeasurement;
import frc.robot.sim.SimDeviceManager;

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

  /** Name of the CAN bus that climber motors are connected to */
  private static final String kCANBusName = "rio";

  /** CAN ID of the motor used to drive the intake flywheels */
  private static final int kFlywheelMotorCANId = 40;

  /** CAN ID of the motor used to drive the gamepiece indexer */
  private static final int kIndexerMotorCANId = 41;

  /** CAN ID of the sensor used to detect the presence of a gamepiece in the intake 
   * @see https://github.com/GrappleRobotics/LaserCAN.git
   */
  private static final int kGamepieceSensorCANId = 42;



  ////////////////////////////////////
  // Flywheel Motor Configuration
  ////////////////////////////////////

  /** Gear ratio between the flywheel motor and the flywheel mechanism */
  private static final double kFlywheelMotorGearRatio = 1.0 / 1.0;

  /** Maximum velocity (rotations per second) that the flywheel motor should run at */
  private static final double kMaxFlywheelMotorVelocity = 1.0;


  ////////////////////////////////////
  // Indexer Motor Configuration
  ////////////////////////////////////

  /** Gear ratio between the indexer motor and the indexer mechanism */
  private static final double kIndexerMotorGearRatio = 5.0 / 1.0;

  /** Maximum velocity (rotations per second) that the indexer motor should run at */
  private static final double kMaxIndexerMotorVelocity = 1.0;

  /** Index of the PID gains to use in the indexer motor */
  public static final int kIndexerPIDSlotIdx = 0;

  /** Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops. For now we just want the
   * primary one */
  public static final int kPIDLoopIdx = 0;

  /** Set to zero to skip waiting for confirmation, set to nonzero to wait and report to DS if action
   * fails */
  public static final int kTimeoutMs = 30;


  ////////////////////////////////////
  // Attributes
  ////////////////////////////////////

  /** Motor used to drive the flywheels at the entrnce of the intake */
  private final TalonFX m_flywheelMotor = new TalonFX(kFlywheelMotorCANId, kCANBusName);

  /** Motor used to drive the indexer (internal) wheels of the intake */
  private final WPI_TalonSRX m_indexerMotor = new WPI_TalonSRX(kGamepieceSensorCANId);


  /** Object used to process measurements from the Talon SRX controllers */
  private final SensorMeasurement m_sensorConverter =
      new SensorMeasurement(Sensor.CTREMagEncoderAbsolute.unitsPerRotation, 1.0, false);

  /** Alert displayed on failure to configure motor controllers */
  private static final Alert s_motorConfigFailedAlert =
      new Alert("Failed to configure climber motors", Alert.AlertType.ERROR);

  /** Creates a new climber subsystem */
  public IntakeSubsystem() {
    configureMotors();
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Sets the desired climber position as a normalized percentage of maximum extension
   *
   * @param degrees Normalized percentage of full climber extension (0.0 to 1.0)
   */
  public void setExtensionPercent(double percent) {
    double targetRotations = percent * kMaxRotations;
    double sensorUnits = m_sensorConverter.rotationsToSensorUnits(targetRotations);
    SmartDashboard.putNumber("climber/setExtension/percent", percent);
    SmartDashboard.putNumber("climber/setExtension/rotations", targetRotations);
    SmartDashboard.putNumber("climber/setExtension/sensorUnits", sensorUnits);
    m_climberLeader.set(ControlMode.Position, sensorUnits);
  }

  public enum ClimberMotorID {
    Leader,
    Follower
  };

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Returns the current extension of a climber motor as a normalized percentage of maximum
   *
   * @motorID Climber motor whose extension is to be returned
   * @return normalized percentage of maximum climber extension (0.0 to 1.0)
   */
  public double getExtensionPercent(ClimberMotorID motorID) {
    double sensorUnits =
        (motorID == ClimberMotorID.Leader)
            ? m_climberLeader.getSelectedSensorPosition(0)
            : m_climberFollower.getSelectedSensorPosition(0);
    double rotations = m_sensorConverter.sensorUnitsToRotations(sensorUnits);
    double percent = rotations / kMaxRotations;

    String motorName = (motorID == ClimberMotorID.Leader) ? "leader" : "follower";
    SmartDashboard.putNumber(String.format("climber/%s/percent", motorName), percent);
    SmartDashboard.putNumber(String.format("climber/%s/rotations", motorName), rotations);
    SmartDashboard.putNumber(String.format("climber/%s/sensorUnits", motorName), sensorUnits);
    return percent;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Returns the current extension of the climber as a normalized percentage of maximum
   *
   * @return normalized percentage of maximum climber extension (0.0 to 1.0)
   */
  public double getExtensionPercent() {
    return getExtensionPercent(ClimberMotorID.Leader);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * This method gets called once when initializing simulation mode
   *
   * @param physicsSim Physics simulator engine for motors, etc.
   */
  public void simulationInit(SimDeviceManager simDeviceMgr) {
    simDeviceMgr.addTalonSRX(m_climberLeader, 0.001);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  private void configureMotors() {
    ///////////////////////////////////
    // Configure CTRE SRX controllers
    ///////////////////////////////////
    m_climberLeader.configFactoryDefault();

    // Set based on what direction you want forward/positive to be. This does not affect sensor
    // phase.
    // Choose based on what direction you want to be positive, this does not affect motor invert
    final boolean kMotorInvert = false;
    m_climberLeader.setInverted(kMotorInvert);

    // Set motor to brake when not commanded
    m_climberLeader.setNeutralMode(NeutralMode.Brake);

    // Set neutral deadband to super small 0.001 (0.1 %) because the default deadband is 0.04 (4 %)
    m_climberLeader.configNeutralDeadband(0.01, kTimeoutMs);

    /* Config the peak and nominal outputs, 12V means full */
    m_climberLeader.configNominalOutputForward(0, kTimeoutMs);
    m_climberLeader.configNominalOutputReverse(0, kTimeoutMs);
    m_climberLeader.configPeakOutputForward(kMaxMotorOutputPercent, kTimeoutMs);
    m_climberLeader.configPeakOutputReverse(-1.0 * kMaxMotorOutputPercent, kTimeoutMs);

    // Configure the SRX controller to use an attached CTRE magnetic encoder's absolute
    // position measurement
    FeedbackDevice feedBackDevice =
        RobotBase.isReal()
            ? FeedbackDevice.CTRE_MagEncoder_Absolute
            : FeedbackDevice.PulseWidthEncodedPosition;
    m_climberLeader.configSelectedFeedbackSensor(feedBackDevice, kPIDLoopIdx, kTimeoutMs);

    // Ensure sensor is positive when output is positive
    // Choose so that Talon does not report sensor out of phase
    final boolean kSensorPhase = false;
    m_climberLeader.setSensorPhase(kSensorPhase);

    configureClosedLoopControl();
    // configureMotionMagicControl();

    /**
     * Grab the 360 degree position of the MagEncoder's absolute position, and intitally set the
     * relative sensor to match.
     */
    int absolutePosition = m_climberLeader.getSensorCollection().getPulseWidthPosition();

    /* Mask out overflows, keep bottom 12 bits */
    absolutePosition &= 0xFFF;
    if (kSensorPhase) {
      absolutePosition *= -1;
    }
    if (kMotorInvert) {
      absolutePosition *= -1;
    }

    /* Set the quadrature (absolute) sensor to match absolute */
    m_climberLeader.setSelectedSensorPosition(absolutePosition, kPIDLoopIdx, kTimeoutMs);

    // Configure follower motor to follow leader motor
    m_climberFollower.follow(m_climberLeader);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /** Configure to use PID-based closed-loop control */
  void configureClosedLoopControl() {
    // Configure the allowable closed-loop error, Closed-Loop output will be neutral within this
    // range.
    // See Table in Section 17.2.1 for native units per rotation.

    // final double deadbandRotations = kMaxRotations * 0.01; // TODO: set up allowable deadband
    // final double deadbandSensorUnits =
    // m_sensorConverter.rotationsToSensorUnits(deadbandRotations);
    // m_climberLeader.configAllowableClosedloopError(0, deadbandSensorUnits, kTimeoutMs);

    /* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
    m_climberLeader.config_kF(kPIDLoopIdx, 0.0, kTimeoutMs);
    m_climberLeader.config_kP(kPIDLoopIdx, 0.15, kTimeoutMs);
    m_climberLeader.config_kI(kPIDLoopIdx, 0.0, kTimeoutMs);
    m_climberLeader.config_kD(kPIDLoopIdx, 1.0, kTimeoutMs);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /** Configure to use Motion Magic trapezoidal profile motor control */
  void configureMotionMagicControl() {
    // Select a motion profile slot
    int kSlotIdx = 0;
    m_climberLeader.selectProfileSlot(kSlotIdx, kPIDLoopIdx);

    // Set up PID gains
    m_climberLeader.config_kF(kPIDLoopIdx, 0.3, kTimeoutMs);
    m_climberLeader.config_kP(kPIDLoopIdx, 0.25, kTimeoutMs);
    m_climberLeader.config_kI(kPIDLoopIdx, 0.0, kTimeoutMs);
    m_climberLeader.config_kD(kPIDLoopIdx, 0.005, kTimeoutMs);
    m_climberLeader.config_IntegralZone(kPIDLoopIdx, 0.01);

    // Set acceleration and cruise velocity
    m_climberLeader.configMotionCruiseVelocity(Phoenix5Util.degreesToFalconTicks(7.6), kTimeoutMs);
    m_climberLeader.configMotionAcceleration(Phoenix5Util.degreesToFalconTicks(4.94), kTimeoutMs);
    m_climberLeader.configMotionSCurveStrength(6);
  }
}
