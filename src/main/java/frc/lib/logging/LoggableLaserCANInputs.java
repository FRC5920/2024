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

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.LaserCan.Measurement;
import au.grapplerobotics.LaserCan.RegionOfInterest;
import au.grapplerobotics.LaserCan.TimingBudget;
import java.util.Map;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

//////////////////////////////////////////////////////////////////////////////////////////////////
/** Logged input measurements for a LaserCAN module */
public class LoggableLaserCANInputs implements LoggableInputs {
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
  private String isValidKey;
  private String statusKey;
  private String distanceKey;
  private String ambientLevelKey;
  private String isLongKey;
  private String timingBudgetKey;

  /** true if a valid measurement is present; else false */
  public boolean isValid = false;

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
    isValidKey = prefix + "/isValid";
    statusKey = prefix + "/status";
    distanceKey = prefix + "/distanceMeters";
    ambientLevelKey = prefix + "/ambientLevel";
    isLongKey = prefix + "/isLongDistance";
    timingBudgetKey = prefix + "/timingBudgetMsec";
    roi = new LoggableRegionOfInterest((prefix + "/roi"), 0, 0, 0, 0);
  }

  /** Creates an instance of the loggable object during clone() calls */
  private LoggableLaserCANInputs(LoggableLaserCANInputs other) {
    // Copy log keys
    this.statusKey = other.statusKey;
    this.distanceKey = other.distanceKey;
    this.ambientLevelKey = other.ambientLevelKey;
    this.isLongKey = other.isLongKey;
    this.timingBudgetKey = other.timingBudgetKey;

    // Copy measurements
    this.status = other.status;
    this.distanceMeters = other.distanceMeters;
    this.ambientLevel = other.ambientLevel;
    this.isLongDistance = other.isLongDistance;
    this.timingBudgetSec = other.timingBudgetSec;
    this.roi = other.roi;
  }

  /** Copies values from a LaserCAN measurement */
  public void fromMeasurement(Measurement measurement) {
    if (measurement != null) {
      isValid = (measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT);
      status = kStatusMap.getOrDefault(measurement.status, String.format("%d", measurement.status));
      distanceMeters = measurement.distance_mm * 0.001;
      ambientLevel = measurement.ambient;
      isLongDistance = measurement.is_long;
      timingBudgetSec = kTimingBudgetMap.getOrDefault(measurement.budget_ms, -1.0);
      roi.x = measurement.roi.x;
      roi.y = measurement.roi.y;
      roi.w = measurement.roi.w;
      roi.h = measurement.roi.h;
    } else {
      isValid = false;
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
    table.put(isValidKey, isValid);
    table.put(statusKey, status);
    table.put(distanceKey, distanceMeters);
    table.put(ambientLevelKey, ambientLevel);
    table.put(isLongKey, isLongDistance);
    table.put(timingBudgetKey, timingBudgetSec);
    roi.toLog(table);
  }

  public void fromLog(LogTable table) {
    isValid = table.get(isValidKey, isValid);
    status = table.get(statusKey, status);
    distanceMeters = table.get(distanceKey, distanceMeters);
    ambientLevel = table.get(ambientLevelKey, ambientLevel);
    isLongDistance = table.get(isLongKey, isLongDistance);
    timingBudgetSec = table.get(timingBudgetKey, timingBudgetSec);
    roi.fromLog(table);
  }

  public LoggableLaserCANInputs clone() {
    return new LoggableLaserCANInputs(this);
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
