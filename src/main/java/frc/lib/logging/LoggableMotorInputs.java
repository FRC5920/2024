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

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

//////////////////////////////////////////////////////////////////////////////////////////////////
/** Logged input measurements for a motor */
public class LoggableMotorInputs implements LoggableInputs {
  // Log keys for measurements
  private String keyTargetPosition;
  private String keyTargetVelocity;
  private String keyPosition;
  private String keyVelocity;
  private String keyVoltage;
  private String keyCurrent;
  private String keyTempCelcius;

  /** Target position in rotations */
  public double targetPosition = 0.0;
  /** Target velocity in rotations per second */
  public double targetVelocity = 0.0;
  /** Position of the mechanism in rotations */
  public double position = 0.0;
  /** Velocity of the mechanism in rotations per second */
  public double velocity = 0.0;
  /** Voltage applied to the motor in Volts */
  public double voltage = 0.0;
  /** Motor current in Amps */
  public double current = 0.0;
  /** Motor temperature in degrees Celcius */
  public double tempCelsius = 0.0;

  /**
   * Creates an instance of the inputs
   *
   * @param prefix Prefix the inputs will be logged under
   */
  public LoggableMotorInputs(String prefix) {
    keyTargetPosition = prefix + "/targetPosition";
    keyTargetVelocity = prefix + "/targetVelocity";
    keyPosition = prefix + "/position";
    keyVelocity = prefix + "/velocity";
    keyVoltage = prefix + "/voltage";
    keyCurrent = prefix + "/current";
    keyTempCelcius = prefix + "/tempCelcius";
  }

  /** Creates an instance of the loggable object during clone() calls */
  private LoggableMotorInputs() {}

  /** Write inputs to the log */
  public void toLog(LogTable table) {
    table.put(keyTargetPosition, targetPosition);
    table.put(keyTargetVelocity, targetVelocity);
    table.put(keyPosition, position);
    table.put(keyVelocity, velocity);
    table.put(keyVoltage, voltage);
    table.put(keyCurrent, current);
    table.put(keyTempCelcius, tempCelsius);
  }

  /** Read inputs from the log */
  public void fromLog(LogTable table) {
    targetPosition = table.get(keyTargetPosition, targetPosition);
    targetVelocity = table.get(keyTargetVelocity, targetVelocity);
    position = table.get(keyPosition, position);
    velocity = table.get(keyVelocity, velocity);
    voltage = table.get(keyVoltage, voltage);
    current = table.get(keyCurrent, current);
    tempCelsius = table.get(keyTempCelcius, tempCelsius);
  }

  /** Create a clone of input values */
  public LoggableMotorInputs clone() {
    LoggableMotorInputs copy = new LoggableMotorInputs();
    // Copy keys
    copy.keyTargetPosition = this.keyTargetPosition;
    copy.keyTargetVelocity = this.keyTargetVelocity;
    copy.keyPosition = this.keyPosition;
    copy.keyVelocity = this.keyVelocity;
    copy.keyVoltage = this.keyVoltage;
    copy.keyCurrent = this.keyCurrent;
    copy.keyTempCelcius = this.keyTempCelcius;

    // Copy measurements
    copy.targetPosition = this.targetPosition;
    copy.targetVelocity = this.targetVelocity;
    copy.position = this.position;
    copy.velocity = this.velocity;
    copy.voltage = this.voltage;
    copy.current = this.current;
    copy.tempCelsius = this.tempCelsius;
    return copy;
  }
}
