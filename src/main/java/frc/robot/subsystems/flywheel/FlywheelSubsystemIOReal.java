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
package frc.robot.subsystems.flywheel;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.lib.logging.BotLog;
import frc.lib.logging.LoggableMotorInputs;
import frc.lib.utility.Alert;
import frc.robot.Constants.CANDevice;

/** Implementation of the IntakeSubsystemIO interface using real hardware */
public class FlywheelSubsystemIOReal implements FlywheelSubsystemIO {

  /** PID slot used for Flywheel voltage control requests */
  private static final int kFlywheelVoltsPIDSlot = 0;

  /** Motor used to drive the flywheels at the entrnce of the intake */
  protected final TalonFX m_flywheelMotor;

  /**
   * Request used to issue a velocity setpoint to the flywheel motor: Start at velocity 0, enable
   * FOC, no feed forward, use slot 0
   */
  private final VelocityVoltage m_voltsVelocityReq =
      new VelocityVoltage(0, 0, true, 0, kFlywheelVoltsPIDSlot, false, false, false);

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

  /** Alert displayed on failure to configure the flywheel motor controller */
  private static final Alert s_flywheelMotorConfigFailedAlert =
      new Alert("Failed to configure flywheel motor", Alert.AlertType.ERROR);

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Creates an instance of the I/O implementation
   *
   * @param flywheelMotorDevice CAN device used for the flywheel motor
   */
  public FlywheelSubsystemIOReal(CANDevice flywheelMotorDevice) {
    m_flywheelMotor = new TalonFX(flywheelMotorDevice.id(), FlywheelSubsystem.kCANBus.name);
    m_flywheelVelocitySignal = m_flywheelMotor.getVelocity();
    m_flywheelVoltageSignal = m_flywheelMotor.getMotorVoltage();
    m_flywheelCurrentSignal = m_flywheelMotor.getStatorCurrent();
    m_flywheelTempSignal = m_flywheelMotor.getDeviceTemp();
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /** Initializes and configures the I/O implementation */
  public void initialize() {
    configureFlywheel();
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * This method is called each robot cycle to process inputs to the subsystem
   *
   * @param inputs Object to populate with subsystem input values to be logged
   */
  @Override
  public void processInputs(LoggableMotorInputs inputs) {
    // Get input measurements for flywheel
    inputs.velocity =
        m_flywheelVelocitySignal.refresh().getValueAsDouble()
            / FlywheelSubsystem.kFlywheelMotorGearRatio;
    inputs.voltage = m_flywheelVoltageSignal.refresh().getValueAsDouble();
    inputs.current = m_flywheelCurrentSignal.refresh().getValueAsDouble();
    inputs.tempCelsius = m_flywheelTempSignal.refresh().getValueAsDouble();
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Sets the desired velocity of the flywheel mechanism
   *
   * @param rotPerSec Desired velocity in rotations per second
   */
  @Override
  public void setFlywheelVelocity(double rotPerSec) {
    if (rotPerSec != 0.0) {
      m_flywheelMotor.setControl(
          m_voltsVelocityReq.withVelocity(rotPerSec * FlywheelSubsystem.kFlywheelMotorGearRatio));
    } else {
      m_flywheelMotor.setControl(m_coast);
    }
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
}
