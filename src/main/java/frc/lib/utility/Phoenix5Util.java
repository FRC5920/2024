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
package frc.lib.utility;

/** Utility functions and constants used in conjunction with the CTRE Phoenix 5 library */
public class Phoenix5Util {

  // Falcon motors report position in sensor units (ticks)
  // 1 revolution = 2048 counts
  private static final double kFalconTicksPerRevolution = 2048.0;

  // Number of degrees per Falcon motor sensor tick
  private static final double kDegreesPerFalconTick = 360.0 / kFalconTicksPerRevolution;
  // Number of Falcon motor sensor ticks per degree
  private static final double kFalconTicksPerDegree = kFalconTicksPerRevolution / 360.0;

  /**
   * Converts an angle in degrees to Falcon sensor units (ticks)
   *
   * @param degrees Angle in degrees to convert
   * @return Equivalent value in Falcon ticks assuming a 1:1 coupling ratio between sensor and motor
   *     shaft
   */
  public static final double degreesToFalconTicks(double degrees) {
    return degrees / kFalconTicksPerDegree;
  }

  /**
   * Converts a value in Falcon sensor units (ticks) to the equivalent motor shaft position in
   * degrees
   *
   * @param falconTicks Falcon ticks value to convert
   * @return Equivalent motor shaft position in degrees
   */
  public static final double falconTicksToDegrees(double falconTicks) {
    return falconTicks * kDegreesPerFalconTick;
  }

  /**
   * Common sensor resolutions
   *
   * @see https://v5.docs.ctr-electronics.com/en/stable/ch14_MCSensor.html#sensor-resolution
   */
  public enum Sensor {
    QuadratureEncoder(4096),
    CTREMagEncoderRelative(4096),
    CTREMagEncoderAbsolute(4096),
    TalonFXIntegratedSensor(2048),
    CANCoder(4096),
    PWMEncoder(4096),
    AndyMarkCIMCoder(80),
    Analog(1024);

    /** Number of units per sensor rotation */
    public final double unitsPerRotation;

    private Sensor(double unitsPerRotation) {
      this.unitsPerRotation = unitsPerRotation;
    }
  }

  /** Base class for objects that convert CTRE sensor measurements */
  public static class SensorMeasurement {
    /** Number of sensor units per rotation of the sensor shaft */
    public final double kSensorUnitsPerRotation;
    /** Gear ratio from the motor shaft to the rotation sensor */
    public final double kMotorToSensorGearRatio;
    /**
     * Coefficient representing the relationship of motor shaft and sensor rotation direction: 1.0
     * if the same; -1.0 if opposite
     */
    public final double kMotorToSensorDirection;

    /**
     * Creates an instance of the measurement object using given attributes
     *
     * @param sensorUnitsPerRotation Number of sensor units per rotation of the sensor shaft
     * @param motorToSensorGearRatio Gear ratio from the motor shaft to the rotation sensor
     * @param sensorDirectionIsInverted True if the motor rotates in the opposite direction of the
     *     sensor; else false
     */
    public SensorMeasurement(
        double sensorUnitsPerRotation,
        double motorToSensorGearRatio,
        boolean sensorDirectionIsInverted) {
      kSensorUnitsPerRotation = sensorUnitsPerRotation;
      kMotorToSensorGearRatio = motorToSensorGearRatio;
      kMotorToSensorDirection = sensorDirectionIsInverted ? -1.0 : 1.0;
    }

    /**
     * Converts a measurement in sensor units to a number of rotations
     *
     * @param sensorUnits Measurement in sensor units
     * @return Number of rotations of the sensor
     */
    public double sensorUnitsToRotations(double sensorUnits) {
      return sensorUnits
          / kSensorUnitsPerRotation
          * kMotorToSensorDirection
          / kMotorToSensorGearRatio;
    }

    /**
     * Converts a number of rotations to a corresponding measurement in sensor units
     *
     * @param rotations Number of rotations of the sensor
     * @return Corresponding measurement in sensor units
     */
    public double rotationsToSensorUnits(double rotations) {
      return rotations * kMotorToSensorGearRatio * kSensorUnitsPerRotation;
    }
  }

  /** An object that wraps closed-loop gains for a Phoenix5-based CTRE motor controller */
  public static class CTREGains {
    public final double kP;
    public final double kI;
    public final double kD;
    public final double kF;
    public final int kIzone;
    public final double kPeakOutput;

    public CTREGains(
        double _kP, double _kI, double _kD, double _kF, int _kIzone, double _kPeakOutput) {
      kP = _kP;
      kI = _kI;
      kD = _kD;
      kF = _kF;
      kIzone = _kIzone;
      kPeakOutput = _kPeakOutput;
    }
  }
}
