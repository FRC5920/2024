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

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.utility.Alert;
import frc.lib.utility.Phoenix5Util.CTREGains;
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
public class ClimberSubsystem extends SubsystemBase {

  ////////////////////////////////////
  // CONSTANTS
  ////////////////////////////////////

  /** Name of the CAN bus that climber motors are connected to */
  private static final String kCANBusName = "rio";

  /** CAN ID of the climber lead motor */
  private static final int kClimberLeaderCANId = 30;
  /** CAN ID of the climber follower motor */
  private static final int kClimberFollowerCANId = 21;

  /** Gear ratio between the Falcon motors and the climber mechanism */
  private static final double kFalconToClimberGearRatio = 20.0 / 1.0;

  /** Max number of rotations to achieve full extension */
  private static final double kMaxRotations = 100.0; // TODO: set this value based on mechanisms

  /**
   * Which PID slot to pull gains from. Starting 2018, you can choose from 0,1,2 or 3. Only the
   * first two (0,1) are visible in web-based configuration.
   */
  public static final int kSlotIdx = 0;

  /**
   * Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops. For now we just want the
   * primary one.
   */
  public static final int kPIDLoopIdx = 0;

  /**
   * Set to zero to skip waiting for confirmation, set to nonzero to wait and report to DS if action
   * fails.
   */
  public static final int kTimeoutMs = 30;

  /* Choose so that Talon does not report sensor out of phase */
  public static boolean kSensorPhase = true;

  /** Choose based on what direction you want to be positive, this does not affect motor invert. */
  public static boolean kMotorInvert = false;

  /**
   * Gains used in Positon Closed Loop, to be adjusted accordingly Gains(kp, ki, kd, kf, izone, peak
   * output);
   */
  static final CTREGains kGains = new CTREGains(0.15, 0.0, 1.0, 0.0, 0, 1.0);

  ////////////////////////////////////
  // Attributes
  ////////////////////////////////////

  /** Master motor used to control climber extension */
  private final WPI_TalonSRX m_climberLeader = new WPI_TalonSRX(kClimberLeaderCANId);
  /** Slave motor used to control climber extension */
  private final WPI_TalonSRX m_climberFollower = new WPI_TalonSRX(kClimberFollowerCANId);

  /** Object used to process measurements from the Talon SRX controllers */
  private final SensorMeasurement m_sensorConverter =
      new SensorMeasurement(Sensor.CTREMagEncoderAbsolute.unitsPerRotation, 1.0, false);

  /** Alert displayed on failure to configure motor controllers */
  private static final Alert s_motorConfigFailedAlert =
      new Alert("Failed to configure climber motors", Alert.AlertType.ERROR);

  /** Creates a new climber subsystem */
  public ClimberSubsystem() {
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
    simDeviceMgr.addTalonSRX(m_climberLeader, 0.75, 4000, kSensorPhase);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  private void configureMotors() {
    ///////////////////////////////////
    // Configure CTRE SRX controllers
    ///////////////////////////////////
    m_climberLeader.configFactoryDefault();

    // Configure the SRX controller to use an attached CTRE magnetic encoder's absolute
    // position measurement
    m_climberLeader.configSelectedFeedbackSensor(
        FeedbackDevice.CTRE_MagEncoder_Absolute, kPIDLoopIdx, kTimeoutMs);

    /* Ensure sensor is positive when output is positive */
    m_climberLeader.setSensorPhase(kSensorPhase);

    /**
     * Set based on what direction you want forward/positive to be. This does not affect sensor
     * phase.
     */
    m_climberLeader.setInverted(kMotorInvert);

    /* Config the peak and nominal outputs, 12V means full */
    m_climberLeader.configNominalOutputForward(0, kTimeoutMs);
    m_climberLeader.configNominalOutputReverse(0, kTimeoutMs);
    m_climberLeader.configPeakOutputForward(1, kTimeoutMs);
    m_climberLeader.configPeakOutputReverse(-1, kTimeoutMs);

    /**
     * Config the allowable closed-loop error, Closed-Loop output will be neutral within this range.
     * See Table in Section 17.2.1 for native units per rotation.
     */
    final double deadbandRotations = kMaxRotations * 0.01; // TODO: set up allowable deadband
    final double deadbandSensorUnits = m_sensorConverter.rotationsToSensorUnits(deadbandRotations);
    m_climberLeader.configAllowableClosedloopError(0, deadbandSensorUnits, kTimeoutMs);

    /* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
    m_climberLeader.config_kF(kPIDLoopIdx, kGains.kF, kTimeoutMs);
    m_climberLeader.config_kP(kPIDLoopIdx, kGains.kP, kTimeoutMs);
    m_climberLeader.config_kI(kPIDLoopIdx, kGains.kI, kTimeoutMs);
    m_climberLeader.config_kD(kPIDLoopIdx, kGains.kD, kTimeoutMs);

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
}
