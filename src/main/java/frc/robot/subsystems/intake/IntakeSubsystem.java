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

  /**
   * CAN ID of the sensor used to detect the presence of a gamepiece in the intake
   *
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

  /**
   * Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops. For now we just want the
   * primary one
   */
  public static final int kPIDLoopIdx = 0;

  /**
   * Set to zero to skip waiting for confirmation, set to nonzero to wait and report to DS if action
   * fails
   */
  public static final int kTimeoutMs = 30;

  ////////////////////////////////////
  // Attributes
  ////////////////////////////////////

  /** Motor used to drive the flywheels at the entrnce of the intake */
  private final TalonFX m_flywheelMotor = new TalonFX(kFlywheelMotorCANId, kCANBusName);

  /** Motor used to drive the indexer (internal) wheels of the intake */
  private final WPI_TalonSRX m_indexerMotor = new WPI_TalonSRX(kGamepieceSensorCANId);

  /** Object used to process measurements from the Talon SRX controllers */
  private final SensorMeasurement m_indexerConverter =
      new SensorMeasurement(Sensor.CTREMagEncoderAbsolute.unitsPerRotation, 1.0, false);

  /** Alert displayed on failure to configure motor controllers */
  private static final Alert s_indexerMotorConfigFailedAlert =
      new Alert("Failed to configure indexer motor", Alert.AlertType.ERROR);

  /** Alert displayed on failure to configure motor controllers */
  private static final Alert s_flywheelMotorConfigFailedAlert =
      new Alert("Failed to configure flywheel motor", Alert.AlertType.ERROR);

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /** Creates a new instance of the subsystem */
  public IntakeSubsystem() {
    configureIndexer();
    configureFlywheel();
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Sets the desired velocity of the indexer mechanism
   *
   * @param rotPerSec Desired velocity in rotations per second
   */
  public void setIndexerVelocity(double rotPerSec) {
    SmartDashboard.putNumber("intake/indexer/setVelocity", rotPerSec);
    // TODO: convert rotPerSec to sensor velocity
    double targetSensorVelocity = 0.0;
    m_indexerMotor.set(ControlMode.Velocity, targetSensorVelocity);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Sets the desired velocity of the flywheel mechanism
   *
   * @param rotPerSec Desired velocity in rotations per second
   */
  public void setFlywheelVelocity(double rotPerSec) {
    SmartDashboard.putNumber("intake/flywheel/setVelocity", rotPerSec);
    // TODO: request Falcon velocity
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Returns the current velocity of the indexer mechanism
   *
   * @return The velocity of the indexer mechanism in rotations per second
   */
  public double getIndexerVelocity() {
    double sensorVelocity = m_indexerMotor.getSelectedSensorPosition(0);
    double rotPerSec = m_indexerConverter.sensorUnitsToRotations(sensorVelocity);

    SmartDashboard.putNumber("intake/indexer/velocity", rotPerSec);
    return rotPerSec;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Returns the current velocity of the flywheel mechanism
   *
   * @return The velocity of the flywheel mechanism in rotations per second
   */
  public double getFlywheelVelocity() {
    // TODO: read velocity from Falcon
    double rotPerSec = 0.0;
    SmartDashboard.putNumber("intake/flywheel/velocity", rotPerSec);
    return rotPerSec;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * This method gets called once when initializing simulation mode
   *
   * @param physicsSim Physics simulator engine for motors, etc.
   */
  public void simulationInit(SimDeviceManager simDeviceMgr) {
    simDeviceMgr.addTalonSRX(m_indexerMotor, 0.001);
    simDeviceMgr.addTalonFX(m_flywheelMotor, 0.001);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  private void configureIndexer() {
    ///////////////////////////////////
    // Configure CTRE SRX controllers
    ///////////////////////////////////
    m_indexerMotor.configFactoryDefault();

    // Set based on what direction you want forward/positive to be. This does not affect sensor
    // phase.
    // Choose based on what direction you want to be positive, this does not affect motor invert
    final boolean kMotorInvert = false;
    m_indexerMotor.setInverted(kMotorInvert);

    // Set motor to brake when not commanded
    m_indexerMotor.setNeutralMode(NeutralMode.Brake);

    // Set neutral deadband to super small 0.001 (0.1 %) because the default deadband is 0.04 (4 %)
    m_indexerMotor.configNeutralDeadband(0.01, kTimeoutMs);

    /* Config the peak and nominal outputs, 12V means full */
    m_indexerMotor.configNominalOutputForward(0, kTimeoutMs);
    m_indexerMotor.configNominalOutputReverse(0, kTimeoutMs);
    // m_indexerMotor.configPeakOutputForward(kMaxMotorOutputPercent, kTimeoutMs);
    // m_indexerMotor.configPeakOutputReverse(-1.0 * kMaxMotorOutputPercent, kTimeoutMs);

    // Configure the SRX controller to use an attached CTRE magnetic encoder's absolute
    // position measurement
    FeedbackDevice feedBackDevice =
        RobotBase.isReal()
            ? FeedbackDevice.CTRE_MagEncoder_Absolute
            : FeedbackDevice.PulseWidthEncodedPosition;
    m_indexerMotor.configSelectedFeedbackSensor(feedBackDevice, kPIDLoopIdx, kTimeoutMs);

    // Ensure sensor is positive when output is positive
    // Choose so that Talon does not report sensor out of phase
    final boolean kSensorPhase = false;
    m_indexerMotor.setSensorPhase(kSensorPhase);

    configureClosedLoopControl();

    /**
     * Grab the 360 degree position of the MagEncoder's absolute position, and intitally set the
     * relative sensor to match.
     */
    int absolutePosition = m_indexerMotor.getSensorCollection().getPulseWidthPosition();

    /* Mask out overflows, keep bottom 12 bits */
    absolutePosition &= 0xFFF;
    if (kSensorPhase) {
      absolutePosition *= -1;
    }
    if (kMotorInvert) {
      absolutePosition *= -1;
    }

    /* Set the quadrature (absolute) sensor to match absolute */
    m_indexerMotor.setSelectedSensorPosition(absolutePosition, kPIDLoopIdx, kTimeoutMs);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  private void configureFlywheel() {}

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /** Configure to use PID-based closed-loop control */
  void configureClosedLoopControl() {
    // Configure the allowable closed-loop error, Closed-Loop output will be neutral within this
    // range.
    // See Table in Section 17.2.1 for native units per rotation.

    // final double deadbandRotations = kMaxRotations * 0.01; // TODO: set up allowable deadband
    // final double deadbandSensorUnits =
    // m_sensorConverter.rotationsToSensorUnits(deadbandRotations);
    // m_indexerMotor.configAllowableClosedloopError(0, deadbandSensorUnits, kTimeoutMs);

    /* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
    m_indexerMotor.config_kF(kPIDLoopIdx, 0.0, kTimeoutMs);
    m_indexerMotor.config_kP(kPIDLoopIdx, 0.15, kTimeoutMs);
    m_indexerMotor.config_kI(kPIDLoopIdx, 0.0, kTimeoutMs);
    m_indexerMotor.config_kD(kPIDLoopIdx, 1.0, kTimeoutMs);
  }
}
