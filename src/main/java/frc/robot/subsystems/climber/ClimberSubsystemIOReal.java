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
package frc.robot.subsystems.climber;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.logging.BotLog;
import frc.lib.logging.LoggableMotorInputs;
import frc.lib.utility.Alert;
import frc.lib.utility.Phoenix5Util;
import frc.lib.utility.Phoenix5Util.Sensor;
import frc.lib.utility.Phoenix5Util.SensorMeasurement;
import frc.robot.subsystems.climber.ClimberSubsystem.ClimberMotorID;
import java.util.ArrayList;

/** Implementation of the ClimberSubsystemIO interface using real hardware */
public class ClimberSubsystemIOReal implements ClimberSubsystemIO {

  // Use the primary PID index
  private static final int kPIDLoopIdx = 0;

  // Timeout used when configuring motor controllers
  private static final int kTimeoutMs = 30;

  /** I/O configuration */
  protected final ClimberSubsystemIO.Config m_config;

  /** Master motor used to control climber extension */
  protected final WPI_TalonSRX m_climberLeader;
  /** Slave motor used to control climber extension */
  protected final WPI_TalonSRX m_climberFollower;

  /** Object used to process measurements from the Talon SRX controllers */
  private final SensorMeasurement m_sensorConverter =
      new SensorMeasurement(Sensor.CTREMagEncoderAbsolute.unitsPerRotation, 1.0, false);

  /** Alert displayed on failure to configure motor controllers */
  private static final Alert s_motorConfigFailedAlert =
      new Alert("Failed to configure climber motors", Alert.AlertType.ERROR);

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Creates an instance of the I/O implementation
   *
   * @config I/O configuration
   */
  public ClimberSubsystemIOReal(ClimberSubsystemIO.Config config) {
    m_config = config;
    m_climberLeader = new WPI_TalonSRX(config.leaderMotorDevice.id);
    m_climberFollower = new WPI_TalonSRX(config.followerMotorDevice.id);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /** Initializes and configures the I/O implementation */
  public void initialize() {
    configureMotors();
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /** Update logged input quantities */
  public void processInputs(ClimberSubsystemInputs inputs) {
    // Get input measurements for leader motor
    LoggableMotorInputs leader = inputs.leader;
    leader.position = getExtensionPercent(ClimberMotorID.Leader);
    leader.voltage = m_climberLeader.getMotorOutputVoltage();
    leader.current = m_climberLeader.getStatorCurrent();
    leader.tempCelsius = m_climberLeader.getTemperature();

    // Get input measurements for follower motor
    LoggableMotorInputs follower = inputs.follower;
    follower.position = getExtensionPercent(ClimberMotorID.Follower);
    follower.voltage = m_climberFollower.getMotorOutputVoltage();
    follower.current = m_climberFollower.getStatorCurrent();
    follower.tempCelsius = m_climberFollower.getTemperature();
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Sets the desired climber position as a normalized percentage of maximum extension
   *
   * @param degrees Normalized percentage of full climber extension (0.0 to 1.0)
   */
  public void setExtensionPercent(double percent) {
    double targetRotations = percent * m_config.maxRotations;
    double sensorUnits = m_sensorConverter.rotationsToSensorUnits(targetRotations);
    m_climberLeader.set(ControlMode.Position, sensorUnits);
  }

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
    double percent = rotations / m_config.maxRotations;

    return percent;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  private void configureMotors() {
    ArrayList<ErrorCode> errors = new ArrayList<>();

    ///////////////////////////////////
    // Configure CTRE SRX controllers
    ///////////////////////////////////
    m_climberLeader.configFactoryDefault();

    // Set based on what direction you want forward/positive to be. This does not affect sensor
    // phase.
    // Choose based on what direction you want to be positive, this does not affect motor invert
    final boolean kMotorInvert = true;
    m_climberLeader.setInverted(kMotorInvert);

    // Set motor to brake when not commanded
    m_climberLeader.setNeutralMode(NeutralMode.Brake);

    // Set neutral deadband to super small 0.001 (0.1 %) because the default deadband is 0.04 (4 %)
    errors.add(m_climberLeader.configNeutralDeadband(0.01, kTimeoutMs));

    /* Config the peak and nominal outputs, 12V means full */
    errors.add(m_climberLeader.configNominalOutputForward(0, kTimeoutMs));
    errors.add(m_climberLeader.configNominalOutputReverse(0, kTimeoutMs));
    errors.add(m_climberLeader.configPeakOutputForward(m_config.maxMotorOutputPercent, kTimeoutMs));
    errors.add(
        m_climberLeader.configPeakOutputReverse(-1.0 * m_config.maxMotorOutputPercent, kTimeoutMs));

    // Configure the SRX controller to use an attached CTRE magnetic encoder's absolute
    // position measurement
    FeedbackDevice feedBackDevice =
        RobotBase.isReal()
            ? FeedbackDevice.CTRE_MagEncoder_Absolute
            : FeedbackDevice.PulseWidthEncodedPosition;
    errors.add(
        m_climberLeader.configSelectedFeedbackSensor(feedBackDevice, kPIDLoopIdx, kTimeoutMs));

    // Ensure sensor is positive when output is positive
    // Choose so that Talon does not report sensor out of phase
    final boolean kSensorPhase = true;
    m_climberLeader.setSensorPhase(kSensorPhase);

    errors.addAll(configureClosedLoopControl());
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
    errors.add(
        m_climberLeader.setSelectedSensorPosition(absolutePosition, kPIDLoopIdx, kTimeoutMs));

    // Configure follower motor to follow leader motor
    m_climberFollower.follow(m_climberLeader);

    // Handle errors encountered during configuration
    for (ErrorCode err : errors) {
      if (err != ErrorCode.OK) {
        s_motorConfigFailedAlert.set(true);
        BotLog.Errorf("Could not configure climber motors. Error: " + err.toString());
      }
    }
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /** Configure to use PID-based closed-loop control */
  ArrayList<ErrorCode> configureClosedLoopControl() {
    ArrayList<ErrorCode> errors = new ArrayList<>();

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

    return errors;
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
