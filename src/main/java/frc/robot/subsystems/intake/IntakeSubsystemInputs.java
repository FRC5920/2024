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
import au.grapplerobotics.LaserCan.Measurement;
import au.grapplerobotics.LaserCan.RegionOfInterest;
import au.grapplerobotics.LaserCan.TimingBudget;
import java.util.Map;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** Logged inputs for the IntakeSubsystem */
public class IntakeSubsystemInputs implements LoggableInputs {
  /** Flywheel motor inputs */
  public LoggableMotorInputs flywheel;
  /** Indexer motor inputs */
  public LoggableMotorInputs indexer;
  /** Gamepiece sensor inputs */
  public LoggableLaserCANInputs laserCAN;

  /**
   * Creates an instance of the inputs and sets the prefix to log them under
   *
   * @param prefix Prefix the inputs will be logged under
   */
  public IntakeSubsystemInputs(String prefix) {
    flywheel = new LoggableMotorInputs("Flywheel");
    indexer = new LoggableMotorInputs("Indexer");
    laserCAN = new LoggableLaserCANInputs("LaserCAN");
  }

  /** Creates an instance of the loggable object during clone() calls */
  private IntakeSubsystemInputs() {}

  /** Write input values to log */
  public void toLog(LogTable table) {
    flywheel.toLog(table);
    indexer.toLog(table);
    laserCAN.toLog(table);
  }

  /** Read input values from log */
  public void fromLog(LogTable table) {
    flywheel.fromLog(table);
    indexer.fromLog(table);
    laserCAN.fromLog(table);
  }

  /** Create a clone of input values */
  public IntakeSubsystemInputs clone() {
    IntakeSubsystemInputs copy = new IntakeSubsystemInputs();
    copy.flywheel = this.flywheel;
    copy.indexer = this.indexer;
    copy.laserCAN = this.laserCAN;
    return copy;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /** Logged input measurements for a motor */
  public static class LoggableMotorInputs implements LoggableInputs {
    // Log keys for measurements
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
     * Creates an instance of the inputs
     *
     * @param prefix Prefix the inputs will be logged under
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

    /** Write inputs to the log */
    public void toLog(LogTable table) {
      table.put(targetVelocityKey, targetVelocity);
      table.put(velocityKey, velocity);
      table.put(voltageKey, voltage);
      table.put(currentKey, current);
      table.put(tempCelciusKey, tempCelsius);
    }

    /** Read inputs from the log */
    public void fromLog(LogTable table) {
      targetVelocity = table.get(targetVelocityKey, targetVelocity);
      velocity = table.get(velocityKey, velocity);
      voltage = table.get(voltageKey, voltage);
      current = table.get(currentKey, current);
      tempCelsius = table.get(tempCelciusKey, tempCelsius);
    }

    /** Create a clone of input values */
    public LoggableMotorInputs clone() {
      LoggableMotorInputs copy = new LoggableMotorInputs();
      // Copy keys
      copy.targetVelocityKey = this.targetVelocityKey;
      copy.velocityKey = this.velocityKey;
      copy.voltageKey = this.voltageKey;
      copy.currentKey = this.currentKey;
      copy.tempCelciusKey = this.tempCelciusKey;

      // Copy measurements
      copy.targetVelocity = this.targetVelocity;
      copy.velocity = this.velocity;
      copy.voltage = this.voltage;
      copy.current = this.current;
      copy.tempCelsius = this.tempCelsius;
      return copy;
    }
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /** Logged input measurements for a LaserCAN module */
  public static class LoggableLaserCANInputs implements LoggableInputs {
    // Map of LaserCAN status ID's to corresponding strings
    private static final Map<Integer, String> kStatusMap =
        Map.of(
            LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT, "LASERCAN_STATUS_VALID_MEASUREMENT",
            LaserCan.LASERCAN_STATUS_NOISE_ISSUE, "LASERCAN_STATUS_NOISE_ISSUE",
            LaserCan.LASERCAN_STATUS_WEAK_SIGNAL, "LASERCAN_STATUS_WEAK_SIGNAL",
            LaserCan.LASERCAN_STATUS_OUT_OF_BOUNDS, "LASERCAN_STATUS_OUT_OF_BOUNDS",
            LaserCan.LASERCAN_STATUS_WRAPAROUND, "LASERCAN_STATUS_WRAPAROUND");

    // Map of LaserCAN TimingBudget ID's to corresponding millisecond counts
    private static final Map<TimingBudget, Double> kTimingBudgetMap =
        Map.of(
            TimingBudget.TIMING_BUDGET_20MS, 0.020,
            TimingBudget.TIMING_BUDGET_33MS, 0.033,
            TimingBudget.TIMING_BUDGET_50MS, 0.050,
            TimingBudget.TIMING_BUDGET_100MS, 0.100);

    // Log keys for measurements
    private String statusKey;
    private String distanceKey;
    private String ambientLevelKey;
    private String isLongKey;
    private String timingBudgetKey;

    /** String-ified representation of LaserCAN status */
    public String status = "";

    /** Measured distance in meters */
    public double distanceMeters = 0.0;

    /** Approximate ambient light level */
    public int ambientLevel = 0;

    /** true if this measurement was taken with the "long" distance mode */
    public boolean isLongDistance = false;

    /** Timing budget the measurement was taken with in milliseconds */
    public double timingBudgetSec = 0.0;

    /** Region of interest the measurement was taken with */
    public LoggableRegionOfInterest roi;

    /**
     * Creates an instance of the inputs
     *
     * @param prefix Prefix the inputs will be logged under
     */
    public LoggableLaserCANInputs(String prefix) {
      statusKey = prefix + "/status";
      distanceKey = prefix + "/distanceMeters";
      ambientLevelKey = prefix + "/ambientLevel";
      isLongKey = prefix + "/isLongDistance";
      timingBudgetKey = prefix + "/timingBudgetMsec";
      roi = new LoggableRegionOfInterest((prefix + "/roi"), 0, 0, 0, 0);
    }

    /** Creates an instance of the loggable object during clone() calls */
    private LoggableLaserCANInputs() {}

    /** Copies values from a LaserCAN measurement */
    public void fromMeasurement(Measurement measurement) {
      if (measurement != null) {
        status =
            kStatusMap.getOrDefault(measurement.status, String.format("%d", measurement.status));
        distanceMeters = measurement.distance_mm * 0.001;
        ambientLevel = measurement.ambient;
        isLongDistance = measurement.is_long;
        timingBudgetSec = kTimingBudgetMap.getOrDefault(measurement.budget_ms, -1.0);
        roi.x = measurement.roi.x;
        roi.y = measurement.roi.y;
        roi.w = measurement.roi.w;
        roi.h = measurement.roi.h;
      } else {
        status = "None";
        distanceMeters = -1.0;
        ambientLevel = -1;
        isLongDistance = false;
        timingBudgetSec = -1.0;
        roi.x = 0;
        roi.y = 0;
        roi.w = 0;
        roi.h = 0;
      }
    }

    public void toLog(LogTable table) {
      table.put(statusKey, status);
      table.put(distanceKey, distanceMeters);
      table.put(ambientLevelKey, ambientLevel);
      table.put(isLongKey, isLongDistance);
      table.put(timingBudgetKey, timingBudgetSec);
      roi.toLog(table);
    }

    public void fromLog(LogTable table) {
      status = table.get(statusKey, status);
      distanceMeters = table.get(distanceKey, distanceMeters);
      ambientLevel = table.get(ambientLevelKey, ambientLevel);
      isLongDistance = table.get(isLongKey, isLongDistance);
      timingBudgetSec = table.get(timingBudgetKey, timingBudgetSec);
      roi.fromLog(table);
    }

    public LoggableLaserCANInputs clone() {
      LoggableLaserCANInputs copy = new LoggableLaserCANInputs();
      // Copy log keys
      copy.statusKey = this.statusKey;
      copy.distanceKey = this.distanceKey;
      copy.ambientLevelKey = this.ambientLevelKey;
      copy.isLongKey = this.isLongKey;
      copy.timingBudgetKey = this.timingBudgetKey;

      // Copy measurements
      copy.status = this.status;
      copy.distanceMeters = this.distanceMeters;
      copy.ambientLevel = this.ambientLevel;
      copy.isLongDistance = this.isLongDistance;
      copy.timingBudgetSec = this.timingBudgetSec;
      copy.roi = this.roi;

      return copy;
    }
  }

  /** A loggable RegionOfInterest object */
  public static class LoggableRegionOfInterest extends RegionOfInterest implements LoggableInputs {
    private String xKey;
    private String yKey;
    private String wKey;
    private String hKey;

    /**
     * Creates an instance of the object
     *
     * @param prefix Key prefix to log under
     */
    public LoggableRegionOfInterest(String prefix, int x, int y, int w, int h) {
      super(x, y, w, h);
      xKey = prefix + "/x";
      yKey = prefix + "/y";
      wKey = prefix + "/w";
      hKey = prefix + "/h";
    }

    private LoggableRegionOfInterest() {
      super(0, 0, 0, 0);
    }

    public void toLog(LogTable table) {
      table.put(xKey, x);
      table.put(yKey, y);
      table.put(wKey, w);
      table.put(hKey, h);
    }

    public void fromLog(LogTable table) {
      x = table.get(xKey, x);
      y = table.get(yKey, y);
      w = table.get(wKey, w);
      h = table.get(hKey, h);
    }

    public LoggableRegionOfInterest clone() {
      LoggableRegionOfInterest copy = new LoggableRegionOfInterest();
      copy.x = this.x;
      copy.y = this.y;
      copy.w = this.w;
      copy.h = this.h;
      return copy;
    }
  }
}
