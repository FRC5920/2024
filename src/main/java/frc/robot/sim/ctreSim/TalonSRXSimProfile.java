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
package frc.robot.sim.ctreSim;

import com.ctre.phoenix.motorcontrol.can.*;
import frc.robot.sim.SimDeviceManager.SimProfile;

/**
 * Simulated device profile for a CTRE Talon SRX motor controller
 *
 * @remarks This profile was copied from the MotionMagic example provided with Phoenix6 Java
 *     examples:
 *     https://github.com/CrossTheRoadElec/Phoenix5-Examples/blob/master/Java%20General/PositionClosedLoop/src/main/java/frc/robot/sim/TalonSRXSimProfile.java
 */
class TalonSRXSimProfile implements SimProfile {
  private final TalonSRX _talon;
  private final double _accelToFullTime;
  private final double _fullVel;
  private final boolean _sensorPhase;

  /** The current position */
  // private double _pos = 0;
  /** The current velocity */
  private double _vel = 0;

  /**
   * Creates a new simulation profile for a TalonSRX device.
   *
   * @param talon The TalonSRX device
   * @param accelToFullTime The time the motor takes to accelerate from 0 to full, in seconds
   * @param fullVel The maximum motor velocity, in ticks per 100ms
   * @param sensorPhase The phase of the TalonSRX sensors
   */
  public TalonSRXSimProfile(
      final TalonSRX talon,
      final double accelToFullTime,
      final double fullVel,
      final boolean sensorPhase) {
    this._talon = talon;
    this._accelToFullTime = accelToFullTime;
    this._fullVel = fullVel;
    this._sensorPhase = sensorPhase;
  }

  /**
   * Calculates the simulated device state
   *
   * @param elapsedSeconds Number of seconds that have elapsed since the last time calculateState()
   *     was calculated. This value is guaranteed to be non-negative and will be exactly equal to
   *     zero when the initial state is being calculated.
   */
  public void calculateState(double elapsedSeconds) {
    final double accelAmount = _fullVel / _accelToFullTime * elapsedSeconds / 1000;

    /// DEVICE SPEED SIMULATION

    double outPerc = _talon.getSimCollection().getMotorOutputLeadVoltage() / 12;
    if (_sensorPhase) {
      outPerc *= -1;
    }
    // Calculate theoretical velocity with some randomness
    double theoreticalVel = outPerc * _fullVel * random(0.95, 1);
    // Simulate motor load
    if (theoreticalVel > _vel + accelAmount) {
      _vel += accelAmount;
    } else if (theoreticalVel < _vel - accelAmount) {
      _vel -= accelAmount;
    } else {
      _vel += 0.9 * (theoreticalVel - _vel);
    }
    // _pos += _vel * period / 100;

    /// SET SIM PHYSICS INPUTS

    _talon.getSimCollection().addQuadraturePosition((int) (_vel * elapsedSeconds / 100));
    _talon.getSimCollection().setQuadratureVelocity((int) _vel);

    double supplyCurrent = Math.abs(outPerc) * 30 * random(0.95, 1.05);
    double statorCurrent = outPerc == 0 ? 0 : supplyCurrent / Math.abs(outPerc);
    _talon.getSimCollection().setSupplyCurrent(supplyCurrent);
    _talon.getSimCollection().setStatorCurrent(statorCurrent);

    _talon.getSimCollection().setBusVoltage(12 - outPerc * outPerc * 3 / 4 * random(0.95, 1.05));
  }

  /* scales a random domain of [0, 2pi] to [min, max] while prioritizing the peaks */
  static double random(double min, double max) {
    return (max - min) / 2 * Math.sin(Math.IEEEremainder(Math.random(), 2 * 3.14159))
        + (max + min) / 2;
  }

  static double random(double max) {
    return random(0, max);
  }
}
