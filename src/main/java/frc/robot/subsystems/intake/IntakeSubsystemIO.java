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

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** I/O abstraction for the IntakeSubsystem */
public interface IntakeSubsystemIO {

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /** Update logged input quantities */
  default void processInputs(IntakeSubsystemInputs inputs) {}

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Sets the desired velocity of the indexer mechanism
   *
   * @param rotPerSec Desired velocity in rotations per second
   */
  default void setIndexerVelocity(double rotPerSec) {}

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Sets the desired velocity of the flywheel mechanism
   *
   * @param rotPerSec Desired velocity in rotations per second
   */
  default void setFlywheelVelocity(double rotPerSec) {}

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Returns the current velocity of the indexer mechanism
   *
   * @return The velocity of the indexer mechanism in rotations per second
   */
  default double getIndexerVelocity() {
    return 0.0;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Returns the current velocity of the flywheel mechanism
   *
   * @return The velocity of the flywheel mechanism in rotations per second
   */
  default double getFlywheelVelocity() {
    return 0.0;
  }

  /** Input measurements for the indexer mechanism */
  @AutoLog
  public static class IndexerInputs {
    /** Target velocity in rotations per second */
    public double targetVelocity = 0.0;
    /** Measured velocity of the indexer mechanism in rotations per second */
    public double velocity = 0.0;
    /** Voltage applied to the indexer motor in Volts */
    public double voltage = 0.0;
    /** Indexer motor current in Amps */
    public double current = 0.0;
    /** Indexer motor temperature in degrees Celcius */
    public double tempCelsius = 0.0;
  }

  /** Input measurements for the flywheel mechanism */
  @AutoLog
  public static class FlywheelInputs {
    /** Target velocity in rotations per second */
    public double targetVelocity = 0.0;
    /** Velocity of the flywheel mechanism in rotations per second */
    public double velocityRotPerSec = 0.0;
    /** Voltage applied to the flywheel motor in Volts */
    public double voltage = 0.0;
    /** Flywheel motor current in Amps */
    public double current = 0.0;
    /** Flywheel motor temperature in degrees Celcius */
    public double tempCelsius = 0.0;
  }

  /** Input measurements for the intake subsystem */
  public static class IntakeSubsystemInputs implements LoggableInputs {
    final FlywheelInputsAutoLogged flywheel = new FlywheelInputsAutoLogged();
    final IndexerInputsAutoLogged indexer = new IndexerInputsAutoLogged();

    public void toLog(LogTable table) {
      flywheel.toLog(table);
      indexer.toLog(table);
    }

    public void fromLog(LogTable table) {
      flywheel.fromLog(table);
      indexer.fromLog(table);
    }
  }
}
