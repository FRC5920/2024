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

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.sim.ctreSim.SimulatedDevice.SimProfile;

/**
 * Simulated device profile for a CTRE TalonFX motor controller configured for FusedCANcoder
 * operation
 *
 * @remarks This profile was copied from the FusedCANcoder example provided with Phoenix6 Java
 *     examples: https://github.com/CrossTheRoadElec/Phoenix6-Examples/tree/main/java/FusedCANcoder
 */
public class TalonFXFusedCANcoderProfile implements SimProfile {
  private static final double kMotorResistance =
      0.002; // Assume 2mOhm resistance for voltage drop calculation
  private final TalonFX _falcon;
  private final CANcoder _canCoder;
  private final double _gearRatio;

  private final DCMotorSim _motorSim;

  /**
   * Creates a new simulation profile for a TalonFX device.
   *
   * @param falcon The TalonFX device
   * @param canCoder The CANcoder associated with the TalonFX
   * @param gearRatio The gear ratio from the TalonFX to the mechanism
   * @param rotorInertia Rotational Inertia of the mechanism at the rotor
   */
  public TalonFXFusedCANcoderProfile(
      final TalonFX falcon,
      final CANcoder canCoder,
      final double gearRatio,
      final double rotorInertia) {
    this._falcon = falcon;
    this._canCoder = canCoder;
    this._gearRatio = gearRatio;
    this._motorSim = new DCMotorSim(DCMotor.getFalcon500Foc(1), gearRatio, rotorInertia);
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

    _motorSim.setInputVoltage(_falcon.getSimState().getMotorVoltage());

    _motorSim.update(elapsedSeconds);

    /// SET SIM PHYSICS INPUTS
    final double position_rot = _motorSim.getAngularPositionRotations();
    final double velocity_rps = Units.radiansToRotations(_motorSim.getAngularVelocityRadPerSec());

    _falcon.getSimState().setRawRotorPosition(position_rot);
    _falcon.getSimState().setRotorVelocity(velocity_rps);

    _falcon
        .getSimState()
        .setSupplyVoltage(12 - _falcon.getSimState().getSupplyCurrent() * kMotorResistance);

    // The fused CANcoder is coupled to the Falcon via a gearbox.  Therefore, the CANcoder angle
    // equals the simulated motor rotations times the (motor-to-mechanism) gear ratio
    _canCoder.getSimState().setRawPosition(position_rot * _gearRatio);
    _canCoder.getSimState().setVelocity(velocity_rps * _gearRatio);
  }
}
