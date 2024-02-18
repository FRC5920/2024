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

import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.sim.ctreSim.SimulatedDevice.SimProfile;

/**
 * Simulated device profile for a CTRE Talon SRX motor controller
 *
 * @remarks This profile was copied from the MotionMagic example provided with Phoenix6 Java
 *     examples:
 *     https://github.com/CrossTheRoadElec/Phoenix5-Examples/blob/master/Java%20General/PositionClosedLoop/src/main/java/frc/robot/sim/TalonSRXSimProfile.java
 */
public class TalonSRXSimProfile implements SimProfile {
  private static final double kRotorInertia = 0.001;
  private static final double kSensorUnitsPerRotation = 4096;
  private final TalonSRX _talon;
  private final DCMotorSim _motorSim;

  /** The current position */
  // private double _pos = 0;
  /** The current velocity */
  private double _vel = 0;

  /**
   * Creates a new simulation profile for a TalonSRX device.
   *
   * @param talon The TalonSRX device
   * @param motorModel Motor model to simulate (e.g. returned by {@code DCMotor.getCIM(1)})
   * @param rotorInertia Rotational Inertia of the mechanism at the rotor
   */
  public TalonSRXSimProfile(final TalonSRX talon, DCMotor motorModel, final double rotorInertia) {
    this._talon = talon;
    this._motorSim = new DCMotorSim(motorModel, 1.0, rotorInertia);
  }

  /**
   * Calculates the simulated device state
   *
   * @param elapsedSeconds Number of seconds that have elapsed since the last time calculateState()
   *     was calculated. This value is guaranteed to be non-negative and will be exactly equal to
   *     zero when the initial state is being calculated.
   */
  public void calculateState(double elapsedSeconds) {
    /// DEVICE SPEED SIMULATION

    TalonSRXSimCollection simCollection = _talon.getSimCollection();

    double motorInputVoltage = simCollection.getMotorOutputLeadVoltage();

    // Run the DC motor simulation with the applied motor voltage and elapsed time
    _motorSim.setInputVoltage(motorInputVoltage);
    _motorSim.update(elapsedSeconds);

    final double position_rot = _motorSim.getAngularPositionRotations();
    final double velocity_rps = Units.radiansToRotations(_motorSim.getAngularVelocityRadPerSec());

    /// SET SIM PHYSICS INPUTS

    // Set position in native units
    int rawPosition = (int) (position_rot * kSensorUnitsPerRotation);
    simCollection.setQuadratureRawPosition(rawPosition);
    simCollection.setPulseWidthPosition(rawPosition);
    simCollection.setAnalogPosition(rawPosition);

    // Set velocity in native units per 100ms
    int rawVelocity = (int) (velocity_rps * kSensorUnitsPerRotation * 0.1);
    simCollection.setQuadratureVelocity(rawVelocity);
    simCollection.setPulseWidthVelocity(rawVelocity);
    simCollection.setAnalogVelocity(rawVelocity);

    double outPerc = motorInputVoltage / 12.0;
    double supplyCurrent = Math.abs(outPerc) * 30 * random(0.95, 1.05);
    double statorCurrent = outPerc == 0 ? 0 : supplyCurrent / Math.abs(outPerc);
    simCollection.setSupplyCurrent(supplyCurrent);
    simCollection.setStatorCurrent(statorCurrent);
    simCollection.setBusVoltage(12 - outPerc * outPerc * 3 / 4 * random(0.95, 1.05));
  }

  /* scales a random domain of [0, 2pi] to [min, max] while prioritizing the peaks */
  static double random(double min, double max) {
    return (max - min) / 2 * Math.sin(Math.IEEEremainder(Math.random(), 2 * 3.14159))
        + (max + min) / 2;
  }
}
