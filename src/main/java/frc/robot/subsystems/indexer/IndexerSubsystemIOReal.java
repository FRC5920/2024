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

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.LaserCan.Measurement;
import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.lib.logging.BotLog;
import frc.lib.utility.Alert;
import java.util.ArrayList;

/** Implementation of the IntakeSubsystemIO interface using real hardware */
public class IndexerSubsystemIOReal implements IndexerSubsystemIO {

  /** PID slot used for Flywheel voltage control requests */
  private static final int kFlywheelVoltsPIDSlot = 0;

  /** PID slot used for Flywheel voltage control requests */
  // private static final int kFlywheelTorquePIDSlot = 1;

  /** I/O configuration */
  protected final Config m_config;

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
  private final VelocityVoltage m_voltsVelocityReq =
      new VelocityVoltage(0, 0, true, 0, kFlywheelVoltsPIDSlot, false, false, false);

  /* Start at velocity 0, no feed forward, use slot 1 */
  // private final VelocityTorqueCurrentFOC m_torqueVelocityReq =
  //     new VelocityTorqueCurrentFOC(0, 0, 0, kFlywheelTorquePIDSlot, false, false, false);

  /* Keep a neutral out so we can disable the motor */
  private final NeutralOut m_coast = new NeutralOut();

  /** Status signal used to read the velocity of the flywheel motor */
  private final StatusSignal<Double> m_flywheelVelocitySignal;
  /** Status signal used to read the voltage of the flywheel motor */
  private final StatusSignal<Double> m_flywheelVoltageSignal;
  /** Status signal used to read the current of the flywheel motor */
  private final StatusSignal<Double> m_flywheelCurrentSignal;
  /** Status signal used to read the temperature of the flywheel motor */
  private final StatusSignal<Double> m_flywheelTempSignal;

  /** Alert displayed on failure to configure the indexer motor controller */
  private static final Alert s_indexerMotorConfigFailedAlert =
      new Alert("Failed to configure indexer motor", Alert.AlertType.ERROR);

  /** Alert displayed on failure to configure the flywheel motor controller */
  private static final Alert s_flywheelMotorConfigFailedAlert =
      new Alert("Failed to configure flywheel motor", Alert.AlertType.ERROR);

  /** Alert displayed on failure to configure the gamepiece sensor */
  private static final Alert s_laserCANConfigFailedAlert =
      new Alert("Failed to configure intake LaserCAN sensor", Alert.AlertType.ERROR);

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Creates an instance of the I/O implementation
   *
   * @param config Configuration values for the I/O implementation
   */
  public IndexerSubsystemIOReal(IndexerSubsystemIO.Config config) {
    m_config = config;
    m_flywheelMotor = new TalonFX(config.flywheelMotorDevice.id(), config.canBus.name);
    m_indexerMotor = new WPI_TalonSRX(config.indexerMotorDevice.id());

    m_flywheelVelocitySignal = m_flywheelMotor.getVelocity();
    m_flywheelVoltageSignal = m_flywheelMotor.getMotorVoltage();
    m_flywheelCurrentSignal = m_flywheelMotor.getStatorCurrent();
    m_flywheelTempSignal = m_flywheelMotor.getDeviceTemp();

    m_gamepieceSensor = new LaserCan(config.gamepieceSensorDevice.id());
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /** Initializes and configures the I/O implementation */
  public void initialize() {
    configureFlywheel();
    configureIndexer();
    configureLaserCAN();
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * This method is called each robot cycle to process inputs to the subsystem
   *
   * @param inputs Object to populate with subsystem input values to be logged
   */
  public void processInputs(IndexerSubsystemInputs inputs) {
    // Get input measurements for flywheel
    inputs.flywheel.velocity = getFlywheelVelocity();
    inputs.flywheel.voltage = m_flywheelVoltageSignal.refresh().getValueAsDouble();
    inputs.flywheel.current = m_flywheelCurrentSignal.refresh().getValueAsDouble();
    inputs.flywheel.tempCelsius = m_flywheelTempSignal.refresh().getValueAsDouble();

    // Get input measurements for indexer
    inputs.indexer.velocity = getIndexerSpeed();
    inputs.indexer.voltage = m_indexerMotor.getMotorOutputVoltage();
    inputs.indexer.current = m_indexerMotor.getStatorCurrent();
    inputs.indexer.tempCelsius = m_indexerMotor.getTemperature();

    // Get input measurements for LaserCAN
    inputs.laserCAN.fromMeasurement(m_gamepieceSensor.getMeasurement());
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Sets the desired velocity of the flywheel mechanism
   *
   * @param rotPerSec Desired velocity in rotations per second
   */
  public void setFlywheelVelocity(double rotPerSec) {
    if (rotPerSec != 0.0) {
      m_flywheelMotor.setControl(
          m_voltsVelocityReq.withVelocity(rotPerSec * m_config.flywheelGearRatio));
      // m_flywheelMotor.setControl(
      //     m_torqueVelocityReq.withVelocity(rotPerSec * m_config.flywheelGearRatio));
    } else {
      m_flywheelMotor.setControl(m_coast);
    }
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Returns the current velocity of the flywheel mechanism
   *
   * @return The velocity of the flywheel mechanism in rotations per second
   */
  public double getFlywheelVelocity() {
    double rotPerSec =
        m_flywheelVelocitySignal.refresh().getValueAsDouble() / m_config.flywheelGearRatio;
    return rotPerSec;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Sets the desired speed of the indexer mechanism as a normalized percentage of full scale
   *
   * @param percent Normalized percentage of full speed (0.0 to 1.0)
   */
  public void setIndexerSpeed(double percent) {
    m_indexerMotor.set(percent);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Returns the current speed of the indexer mechanism as a percentage of full speed
   *
   * @return Normalized percentage of full speed (0.0 to 1.0)
   */
  public double getIndexerSpeed() {
    return m_indexerMotor.getMotorOutputPercent();
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Returns the distance measured by the gamepiece sensor
   *
   * @return The distance measured by the gamepiece sensor in meters
   */
  public double getGamepieceDistance() {
    double distance = -1.0;
    Measurement measurement = m_gamepieceSensor.getMeasurement();
    if ((measurement != null)
        && (measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT)) {
      distance = measurement.distance_mm * 0.001;
    }

    return distance;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  private void configureFlywheel() {
    TalonFXConfiguration configs = new TalonFXConfiguration();

    /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
    configs.Slot0.kP = 5.0;
    configs.Slot0.kI = 0.0;
    configs.Slot0.kD = 0.001;
    configs.Slot0.kV = 0.0;
    // Might want to set peak output of voltage-based commands using measured values
    configs.Voltage.PeakForwardVoltage = 12;
    configs.Voltage.PeakReverseVoltage = -12;

    // Torque-based velocity does not require a feed forward, as torque will accelerate the rotor up
    // to the desired velocity by itself
    configs.Slot1.kP = 5;
    configs.Slot1.kI = 0.1;
    configs.Slot1.kD = 0.001;

    // Might want to set peak output of torque-based commands using measured values
    configs.TorqueCurrent.PeakForwardTorqueCurrent = 40;
    configs.TorqueCurrent.PeakReverseTorqueCurrent = -40;

    // Implement motor invert
    configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    configs.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // Ramp flywheel speed slightly to avoid current spikes
    // Set the time required to ramp from zero to full speed (12V) output
    configs.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;

    // Retry config apply up to 5 times, Alert on failure
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_flywheelMotor.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      s_flywheelMotorConfigFailedAlert.set(true);
      BotLog.Errorf("Failed to configure flywheel. Error: " + status.toString());
    }
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  private void configureIndexer() {

    ArrayList<ErrorCode> errors = new ArrayList<>();

    final int kTimeoutMs =
        30; // Time to wait for confirmation when configuring the motor controller

    ///////////////////////////////////
    // Configure CTRE SRX controllers
    ///////////////////////////////////
    m_indexerMotor.configFactoryDefault();

    // Implement motor direction invert
    m_indexerMotor.setInverted(false);

    // Set motor to brake when not commanded
    m_indexerMotor.setNeutralMode(NeutralMode.Brake);

    // Set neutral deadband to super small 0.001 (0.1 %) because the default deadband is 0.04 (4 %)
    errors.add(m_indexerMotor.configNeutralDeadband(0.01, kTimeoutMs));

    /* Config the peak and nominal outputs, 12V means full */
    errors.add(m_indexerMotor.configNominalOutputForward(0, kTimeoutMs));
    errors.add(m_indexerMotor.configNominalOutputReverse(0, kTimeoutMs));
    errors.add(m_indexerMotor.configPeakOutputForward(m_config.maxIndexerSpeed, kTimeoutMs));
    errors.add(m_indexerMotor.configPeakOutputReverse(-1.0 * m_config.maxIndexerSpeed, kTimeoutMs));

    // Configure the motor to ramp speed up to avoid current spikes
    errors.add(m_indexerMotor.configOpenloopRamp(0.25));

    // Configure motor to limit current draw after 100 ms
    // Might want to set indexer motor current limits using measured values
    // errors.add(m_indexerMotor.configPeakCurrentDuration(100));
    // errors.add(m_indexerMotor.configPeakCurrentLimit(20));
    // errors.add(m_indexerMotor.configContinuousCurrentLimit(20));

    // Handle errors encountered during configuration
    for (ErrorCode err : errors) {
      if (err != ErrorCode.OK) {
        s_indexerMotorConfigFailedAlert.set(true);
        BotLog.Errorf("Could not configure indexer. Error: " + err.toString());
      }
    }
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /** Configure the LaserCAN gamepiece sensor */
  private void configureLaserCAN() {
    // Initialise the settings of the LaserCAN
    try {
      m_gamepieceSensor.setRangingMode(IndexerSubsystem.kLaserCANRangingMode);
      m_gamepieceSensor.setRegionOfInterest(IndexerSubsystem.kLaserCANRegionOfInterest);
      m_gamepieceSensor.setTimingBudget(IndexerSubsystem.kLaserCANTimingBudget);
    } catch (ConfigurationFailedException e) {
      s_laserCANConfigFailedAlert.set(true);
      BotLog.Errorf("LaserCAN configuration failed. Error: " + e);
    }
  }
}
