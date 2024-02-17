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
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.lib.utility.Alert;
import frc.robot.Constants.CANDevice;
import frc.robot.Constants.RobotCANBus;

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
  private final VelocityVoltage m_voltsVelocityReq =
      new VelocityVoltage(0, 0, true, 0, kFlywheelPIDSlot, false, false, false);

  /* Start at velocity 0, no feed forward, use slot 1 */
  private final VelocityTorqueCurrentFOC m_torqueVelocityReq =
      new VelocityTorqueCurrentFOC(0, 0, 0, 1, false, false, false);

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
  public IntakeSubsystemIOReal(
      RobotCANBus bus, CANDevice flywheelCAN, CANDevice indexerCAN, CANDevice gamepieceSensorCAN) {
    m_flywheelMotor = new TalonFX(flywheelCAN.id, bus.name);
    m_indexerMotor = new WPI_TalonSRX(indexerCAN.id);

    m_flywheelVelocitySignal = m_flywheelMotor.getVelocity();
    m_flywheelVoltageSignal = m_flywheelMotor.getMotorVoltage();
    m_flywheelCurrentSignal = m_flywheelMotor.getStatorCurrent();
    m_flywheelTempSignal = m_flywheelMotor.getDeviceTemp();

    m_gamepieceSensor = new LaserCan(gamepieceSensorCAN.id);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /** Initializes and configures the I/O implementation */
  public void initialize() {
    configureFlywheel();
    configureIndexer();
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * This method is called each robot cycle to process inputs to the subsystem
   *
   * @param inputs Object to populate with subsystem input values to be logged
   */
  public void processInputs(IntakeSubsystemInputs inputs) {
    // Get input measurements for flywheel
    inputs.flywheel.velocity = m_flywheelVelocitySignal.refresh().getValueAsDouble();
    inputs.flywheel.voltage = m_flywheelVoltageSignal.refresh().getValueAsDouble();
    inputs.flywheel.current = m_flywheelCurrentSignal.refresh().getValueAsDouble();
    inputs.flywheel.tempCelsius = m_flywheelTempSignal.refresh().getValueAsDouble();

    // Get input measurements for indexer
    inputs.indexer.velocity = m_indexerMotor.getSelectedSensorVelocity();
    inputs.indexer.voltage = m_indexerMotor.getMotorOutputVoltage();
    inputs.indexer.current = m_indexerMotor.getStatorCurrent();
    inputs.indexer.tempCelsius = m_indexerMotor.getTemperature();

    // Get input measurements for LaserCAN
    inputs.laserCAN.fromMeasurement(m_gamepieceSensor.getMeasurement());
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
   * Sets the desired velocity of the flywheel mechanism
   *
   * @param rotPerSec Desired velocity in rotations per second
   */
  public void setFlywheelVelocity(double rotPerSec) {
    m_flywheelMotor.setControl(m_voltsVelocityReq.withVelocity(rotPerSec));
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Returns the current speed of the indexer mechanism as a percentage of full speed
   *
   * @return Normalized percentage of full speed (0.0 to 1.0)
   */
  public double getIndexerSpeed() {
    return m_indexerMotor.get();
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Returns the current velocity of the flywheel mechanism
   *
   * @return The velocity of the flywheel mechanism in rotations per second
   */
  public double getFlywheelVelocity() {
    double rotPerSec = m_flywheelVelocitySignal.refresh().getValueAsDouble();
    return rotPerSec;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  private void configureFlywheel() {
    TalonFXConfiguration configs = new TalonFXConfiguration();

    /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
    configs.Slot0.kP = 0.11; // An error of 1 rotation per second results in 2V output
    configs.Slot0.kI =
        0.5; // An error of 1 rotation per second increases output by 0.5V every second
    configs.Slot0.kD =
        0.0001; // A change of 1 rotation per second squared results in 0.01 volts output
    configs.Slot0.kV =
        0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts /
    // Rotation per second
    // Peak output of 8 volts
    configs.Voltage.PeakForwardVoltage = 8;
    configs.Voltage.PeakReverseVoltage = -8;

    /* Torque-based velocity does not require a feed forward, as torque will accelerate the rotor up to the desired velocity by itself */
    configs.Slot1.kP = 5; // An error of 1 rotation per second results in 5 amps output
    configs.Slot1.kI =
        0.1; // An error of 1 rotation per second increases output by 0.1 amps every second
    configs.Slot1.kD =
        0.001; // A change of 1000 rotation per second squared results in 1 amp output

    // Peak output of 40 amps
    configs.TorqueCurrent.PeakForwardTorqueCurrent = 40;
    configs.TorqueCurrent.PeakReverseTorqueCurrent = -40;

    /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_flywheelMotor.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      s_flywheelMotorConfigFailedAlert.set(true);
      System.out.println("Could not configure device. Error: " + status.toString());
    }
  }

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

    // TODO: configure motor current limits and ramping
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /** Configure to use PID-based closed-loop control */
  void configureClosedLoopControl() {}
}
