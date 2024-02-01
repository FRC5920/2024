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

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.sim.ctreSim.TalonFXFusedCANcoderProfile;
import frc.robot.sim.ctreSim.TalonFXProfile;
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

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /** Interface implemented by simulated devices */
  public interface SimProfile {

    /**
     * Calculates the simulated device state
     *
     * @param elapsedSeconds Number of seconds that have elapsed since the last time
     *     calculateState() was calculated. This value is guaranteed to be non-negative and will be
     *     exactly equal to zero when the initial state is being calculated.
     */
    public void calculateState(double elapsedSeconds);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /** Inner class used to track simulated devices */
  private static class SimulatedDevice {
    /** The simulated device implementation */
    private final SimProfile m_profile;

    /** Flag used to indicate the initial simulated device state calculation */
    private boolean m_isInitialState = true;

    /** Time in seconds when calculate() was last called */
    private double m_lastSimTime = 0.0;

    public SimulatedDevice(SimProfile profile) {
      m_profile = profile;
    }

    /** Runs the simulation profile. Implemented by device-specific profiles. */
    public void calculate() {
      double currentTimeSec = Timer.getFPGATimestamp();

      // Handle initial state calculation
      if (m_isInitialState) {
        m_lastSimTime = currentTimeSec;
        m_isInitialState = false;
      }

      // Calculate the device's state using the elapsed time in seconds
      double elapsedSeconds = currentTimeSec - m_lastSimTime;

      if (elapsedSeconds < 0.0) {
        System.out.println("*** WARNING *** skipping negative time step in simulation!");
      } else {
        m_profile.calculateState(elapsedSeconds);
      }

      m_lastSimTime = currentTimeSec;
    }
  }
}
