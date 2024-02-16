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
package frc.robot.sim;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.sim.ctreSim.SimulatedDevice;
import frc.robot.sim.ctreSim.TalonFXFusedCANcoderProfile;
import frc.robot.sim.ctreSim.TalonFXProfile;
import frc.robot.sim.ctreSim.TalonSRXSimProfile;
import java.util.ArrayList;

/** An object that tracks and manages recalculation of a collection of simulated devices */
public class SimDeviceManager {

  /** Profiles of devices to be simulated */
  private final ArrayList<SimulatedDevice> m_devices = new ArrayList<>();

  /** Recalculates the state of all simulated devices */
  public void calculateSimStates() {
    for (SimulatedDevice device : m_devices) {
      device.calculate();
    }
  }

  /**
   * Adds a TalonFX controller that is configured to track a CANcoder
   *
   * @param falcon The TalonFX device
   * @param can The CANcoder device
   * @param gearRatio The gear reduction of the TalonFX
   * @param rotorInertia Rotational Inertia of the mechanism at the rotor
   */
  public void addTalonFX(TalonFX falcon, final double rotorInertia) {
    if (falcon != null) {
      m_devices.add(new SimulatedDevice(new TalonFXProfile(falcon, rotorInertia)));
    }
  }

  /**
   * Adds a TalonFX controller that is configured to track a CANcoder
   *
   * @param falcon The TalonFX device
   * @param can The CANcoder device
   * @param gearRatio The gear reduction of the TalonFX
   * @param rotorInertia Rotational Inertia of the mechanism at the rotor
   */
  public void addTalonFXWithFusedCANcoder(
      TalonFX falcon, CANcoder can, double gearRatio, final double rotorInertia) {
    if (falcon != null) {
      m_devices.add(
          new SimulatedDevice(
              new TalonFXFusedCANcoderProfile(falcon, can, gearRatio, rotorInertia)));
    }
  }

  /**
   * Adds a simulated TalonSRX controller
   *
   * @param talon The TalonSRX device
   * @param accelToFullTime The time the motor takes to accelerate from 0 to full, in seconds
   * @param fullVel The maximum motor velocity, in ticks per 100ms
   * @param sensorPhase The phase of the TalonSRX sensors
   */
  public void addTalonSRX(TalonSRX talon, double rotorInertia) {
    if (talon != null) {
      m_devices.add(new SimulatedDevice(new TalonSRXSimProfile(talon, rotorInertia)));
    }
  }
}
