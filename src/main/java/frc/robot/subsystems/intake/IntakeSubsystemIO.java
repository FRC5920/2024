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

import au.grapplerobotics.LaserCan.RegionOfInterest;
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

  /** Input measurements for the flywheel mechanism */
  public static class LoggableMotorInputs implements LoggableInputs {
    /** Keys to log fields under */
    private String targetVelocityKey;

    private String velocityKey;
    private String voltageKey;
    private String currentKey;
    private String tempCelciusKey;

    /** Target velocity in rotations per second */
    public double targetVelocity = 0.0;
    /** Velocity of the mechanism in rotations per second */
    public double velocity = 0.0;
    /** Voltage applied to the motor in Volts */
    public double voltage = 0.0;
    /** Motor current in Amps */
    public double current = 0.0;
    /** Motor temperature in degrees Celcius */
    public double tempCelsius = 0.0;

    /**
     * Creates an instance of the loggable object
     *
     * @param prefix Log prefix the object's fields will appear under
     */
    public LoggableMotorInputs(String prefix) {
      targetVelocityKey = prefix + "/targetVelocity";
      velocityKey = prefix + "/velocity";
      voltageKey = prefix + "/voltage";
      currentKey = prefix + "/current";
      tempCelciusKey = prefix + "/tempCelcius";
    }

    /** Creates an instance of the loggable object during clone() calls */
    private LoggableMotorInputs() {}

    public void toLog(LogTable table) {
      table.put(targetVelocityKey, targetVelocity);
      table.put(velocityKey, velocity);
      table.put(voltageKey, voltage);
      table.put(currentKey, current);
      table.put(tempCelciusKey, tempCelsius);
    }

    public void fromLog(LogTable table) {
      targetVelocity = table.get(targetVelocityKey, targetVelocity);
      velocity = table.get(velocityKey, velocity);
      voltage = table.get(voltageKey, voltage);
      current = table.get(currentKey, current);
      tempCelsius = table.get(tempCelciusKey, tempCelsius);
    }

    public LoggableMotorInputs clone() {
      LoggableMotorInputs copy = new LoggableMotorInputs();
      copy.targetVelocityKey = this.targetVelocityKey;
      copy.velocityKey = this.velocityKey;
      copy.voltageKey = this.voltageKey;
      copy.currentKey = this.currentKey;
      copy.tempCelciusKey = this.tempCelciusKey;

      copy.targetVelocity = this.targetVelocity;
      copy.velocity = this.velocity;
      copy.voltage = this.voltage;
      copy.current = this.current;
      copy.tempCelsius = this.tempCelsius;
      return copy;
    }
  }

  /** Input measurements for the LaserCAN module used to detect a gamepiece in the intake */
  public static class LoggableLaserCANInputs implements LoggableInputs {
    private String statusKey;
    private String distanceKey;
    private String ambientLevelKey;
    private String isLongKey;
    private String timingBudgetKey;

    /** String-ified representation of LaserCAN status */
    public String status = "";

    /** Measured distance in meters */
    public int distanceMeters;

    /** Approximate ambient light level */
    public int ambientLevel;

    /** true if this measurement was taken with the "long" distance mode */
    public boolean isLong;

    /** Timing budget the measurement was taken with in milliseconds */
    public int timingBudgetMsec;

    /** Region of interest the measurement was taken with */
    public RegionOfInterest roi = new RegionOfInterest(0, 0, 0, 0);

    /**
     * Creates an instance of the object
     *
     * @param prefix Key prefix to log under
     */
    public LoggableLaserCANInputs(String prefix) {
      statusKey = prefix + "/status";
      distanceKey = prefix + "/distanceMeters";
      ambientLevelKey = prefix + "/ambientLevel";
      isLongKey = prefix + "/isLong";
      timingBudgetKey = prefix + "/timingBudgetMsec";
    }

    /** Creates an instance of the loggable object during clone() calls */
    private LoggableLaserCANInputs() {}

    public void toLog(LogTable table) {
      // TODO: write fields to log
    }

    public void fromLog(LogTable table) {
      // TODO: read fields from log
    }

    public LoggableLaserCANInputs clone() {
      LoggableLaserCANInputs copy = new LoggableLaserCANInputs();

      copy.statusKey = this.statusKey;
      copy.distanceKey = this.distanceKey;
      copy.ambientLevelKey = this.ambientLevelKey;
      copy.isLongKey = this.isLongKey;
      copy.timingBudgetKey = this.timingBudgetKey;
      copy.status = this.status;
      copy.distanceMeters = this.distanceMeters;
      copy.ambientLevel = this.ambientLevel;
      copy.isLong = this.isLong;
      copy.timingBudgetMsec = this.timingBudgetMsec;
      copy.roi = this.roi;

      return copy;
    }
  }

  /** Input measurements for the intake subsystem */
  public static class IntakeSubsystemInputs implements LoggableInputs {
    public LoggableMotorInputs flywheel;
    public LoggableMotorInputs indexer;
    public LoggableLaserCANInputs laserCAN;

    public IntakeSubsystemInputs(String key) {
      flywheel = new LoggableMotorInputs("Flywheel");
      indexer = new LoggableMotorInputs("Indexer");
      laserCAN = new LoggableLaserCANInputs("LaserCAN");
    }

    /** Creates an instance of the loggable object during clone() calls */
    private IntakeSubsystemInputs() {}

    public void toLog(LogTable table) {
      flywheel.toLog(table);
      indexer.toLog(table);
      laserCAN.toLog(table);
    }

    public void fromLog(LogTable table) {
      flywheel.fromLog(table);
      indexer.fromLog(table);
      laserCAN.fromLog(table);
    }

    public IntakeSubsystemInputs clone() {
      IntakeSubsystemInputs copy = new IntakeSubsystemInputs();
      copy.flywheel = this.flywheel;
      copy.indexer = this.indexer;
      copy.laserCAN = this.laserCAN;
      return copy;
    }
  }
}
