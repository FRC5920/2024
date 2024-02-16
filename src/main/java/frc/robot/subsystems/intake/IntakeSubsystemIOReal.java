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
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.utility.Alert;
import frc.lib.utility.Phoenix5Util;

/** Implementation of the IntakeSubsystemIO interface using real hardware */
public class IntakeSubsystemIOReal implements IntakeSubsystemIO {

  private static final int kIndexerPIDSlot = 0;
  private static final int kFlywheelPIDSlot = 0;

  /** Motor used to drive the flywheels at the entrnce of the intake */
  protected final TalonFX m_flywheelMotor;

  /** Motor used to drive the indexer (internal) wheels of the intake */
  protected final WPI_TalonSRX m_indexerMotor;

  /**
   * LaserCAN module used to detect a gamepiece in the intake
   *
   * @see https://github.com/GrappleRobotics/LaserCAN.git
   */
  protected final LaserCan m_gamepieceSensor;

  /**
   * Request used to issue a velocity setpoint to the flywheel motor: Start at velocity 0, enable
   * FOC, no feed forward, use slot 0
   */
  private final VelocityVoltage m_velocityReq =
      new VelocityVoltage(0, 0, true, 0, kFlywheelPIDSlot, false, false, false);

  /** Status signal used to read the velocity of the flywheel motor */
  private final StatusSignal<Double> m_velocityStatus;

  /** Alert displayed on failure to configure the indexer motor controller */
  private static final Alert s_indexerMotorConfigFailedAlert =
      new Alert("Failed to configure indexer motor", Alert.AlertType.ERROR);

  /** Alert displayed on failure to configure the flywheel motor controller */
  private static final Alert s_flywheelMotorConfigFailedAlert =
      new Alert("Failed to configure flywheel motor", Alert.AlertType.ERROR);

  /** Alert displayed on failure to configure the gamepiece sensor */
  private static final Alert s_gamepieceSensorConfigFailedAlert =
      new Alert("Failed to configure intake gamepiece sensor", Alert.AlertType.ERROR);

  public static class Config {
    /** CAN bus attached to the subsystem */
    String CANBus = "";
    /** CAN ID of the flywheel motor */
    int flywheelMotorCANId = -1;
    /** CAN ID of the indexer motor */
    int indexerMotorCANId = -1;
    /** CAN ID of the gamepiece detector */
    int gamepieceSensorCANId = -1;

    public Config(String busName, int flywheelID, int indexerID, int gamepieceID) {
      CANBus = busName;
      flywheelMotorCANId = flywheelID;
      indexerMotorCANId = indexerID;
      gamepieceSensorCANId = gamepieceID;
    }
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Creates an instance of the I/O implementation
   *
   * @param config Configuration values for the I/O implementation
   */
  public IntakeSubsystemIOReal(Config config) {
    m_flywheelMotor = new TalonFX(config.flywheelMotorCANId, config.CANBus);
    m_indexerMotor = new WPI_TalonSRX(config.indexerMotorCANId);
    m_velocityStatus = m_flywheelMotor.getVelocity();

    m_gamepieceSensor = new LaserCan(config.gamepieceSensorCANId);

    configureFlywheel();
    configureIndexer();
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  public void processInputs(IntakeSubsystemInputs inputs) {}

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Sets the desired velocity of the indexer mechanism
   *
   * @param rotPerSec Desired velocity in rotations per second
   */
  public void setIndexerVelocity(double rotPerSec) {
    m_indexerMotor.set(ControlMode.Velocity, Phoenix5Util.rotationsToFalconTicks(rotPerSec));
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Sets the desired velocity of the flywheel mechanism
   *
   * @param rotPerSec Desired velocity in rotations per second
   */
  public void setFlywheelVelocity(double rotPerSec) {
    m_flywheelMotor.setControl(m_velocityReq.withVelocity(rotPerSec));
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Returns the current velocity of the indexer mechanism
   *
   * @return The velocity of the indexer mechanism in rotations per second
   */
  public double getIndexerVelocity() {
    double sensorVelocity = m_indexerMotor.getSelectedSensorPosition(0);
    double rotPerSec = Phoenix5Util.falconTicksToRotations(sensorVelocity);
    return rotPerSec;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Returns the current velocity of the flywheel mechanism
   *
   * @return The velocity of the flywheel mechanism in rotations per second
   */
  public double getFlywheelVelocity() {
    double rotPerSec = m_velocityStatus.refresh().getValueAsDouble();
    return rotPerSec;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  private void configureFlywheel() {}

  //////////////////////////////////////////////////////////////////////////////////////////////////
  private void configureIndexer() {

    final int kTimeoutMs =
        30; // Time to wait for confirmation when configuring the motor controller

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
    m_indexerMotor.configSelectedFeedbackSensor(feedBackDevice, kIndexerPIDSlot, kTimeoutMs);

    // Ensure sensor is positive when output is positive
    // Choose so that Talon does not report sensor out of phase
    final boolean kSensorPhase = false;
    m_indexerMotor.setSensorPhase(kSensorPhase);

    // Configure the allowable closed-loop error, Closed-Loop output will be neutral within this
    // range.
    // See Table in Section 17.2.1 for native units per rotation.

    // final double deadbandRotations = kMaxRotations * 0.01; // TODO: set up allowable deadband
    // final double deadbandSensorUnits =
    // m_sensorConverter.rotationsToSensorUnits(deadbandRotations);
    // m_indexerMotor.configAllowableClosedloopError(0, deadbandSensorUnits, kTimeoutMs);

    /* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
    m_indexerMotor.config_kF(kIndexerPIDSlot, 0.0, kTimeoutMs);
    m_indexerMotor.config_kP(kIndexerPIDSlot, 0.15, kTimeoutMs);
    m_indexerMotor.config_kI(kIndexerPIDSlot, 0.0, kTimeoutMs);
    m_indexerMotor.config_kD(kIndexerPIDSlot, 1.0, kTimeoutMs);

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
    m_indexerMotor.setSelectedSensorPosition(absolutePosition, kIndexerPIDSlot, kTimeoutMs);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /** Configure to use PID-based closed-loop control */
  void configureClosedLoopControl() {}
}
