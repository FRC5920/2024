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
package frc.lib.logging;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import org.littletonrobotics.junction.Logger;

//////////////////////////////////////////////////////////////////////////////////////////////////
/** Logged output measurements for a motor */
public class TalonFXOutputLogger {
  // Log keys for measurements
  private final String keyPosition;
  private final String keyVelocity;
  private final String keyMotorVoltage;
  private final String keySupplyVoltage;
  private final String keyMotorCurrent;
  private final String keySupplyCurrent;
  private final String keyTempCelcius;

  private final StatusSignal positionSignal;
  /** Motor position in rotations (0..1) */
  private final StatusSignal velocitySignal;
  /** Velocity in rotations per second */
  private final StatusSignal motorVoltageSignal;
  /** Motor voltage in Volts */
  private final StatusSignal supplyVoltageSignal;
  /** Supply voltage in Volts */
  private final StatusSignal motorCurrentSignal;
  /** Motor current in Amps */
  private final StatusSignal supplyCurrentSignal;
  /** Supply current in Amps */
  private final StatusSignal temperatureSignal;
  /** Temperature in degrees Celcius */

  /**
   * Creates an instance of the object
   *
   * @param prefix Prefix motor outputs will be logged under
   * @param motor Motor whose measurements are to be logged
   */
  public TalonFXOutputLogger(String prefix, TalonFX motor) {
    keyPosition = prefix + "/position";
    keyVelocity = prefix + "/velocity";
    keyMotorVoltage = prefix + "/voltage";
    keySupplyVoltage = prefix + "/supplyVoltage";
    keyMotorCurrent = prefix + "/current";
    keySupplyCurrent = prefix + "/supplyCurrent";
    keyTempCelcius = prefix + "/tempCelcius";

    positionSignal = motor.getPosition();
    velocitySignal = motor.getVelocity();
    motorVoltageSignal = motor.getMotorVoltage();
    supplyVoltageSignal = motor.getSupplyVoltage();
    motorCurrentSignal = motor.getStatorCurrent();
    supplyCurrentSignal = motor.getSupplyCurrent();
    temperatureSignal = motor.getDeviceTemp();
  }

  /**
   * Writes motor measurements to the log as outputs
   *
   * @param motor Motor whose measurements are to be logged
   */
  public void toLog() {
    // Refresh status signals
    BaseStatusSignal.refreshAll(
        positionSignal,
        velocitySignal,
        motorVoltageSignal,
        supplyVoltageSignal,
        motorCurrentSignal,
        supplyCurrentSignal,
        temperatureSignal);

    // Record measurements
    Logger.recordOutput(keyPosition, positionSignal.getValueAsDouble());
    Logger.recordOutput(keyVelocity, velocitySignal.getValueAsDouble());
    Logger.recordOutput(keyMotorVoltage, motorVoltageSignal.getValueAsDouble());
    Logger.recordOutput(keySupplyVoltage, supplyVoltageSignal.getValueAsDouble());
    Logger.recordOutput(keyMotorCurrent, motorCurrentSignal.getValueAsDouble());
    Logger.recordOutput(keySupplyCurrent, supplyCurrentSignal.getValueAsDouble());
    Logger.recordOutput(keyTempCelcius, temperatureSignal.getValueAsDouble());
  }
}
