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
package frc.robot.subsystems.climber;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.utility.Alert;
import frc.robot.sim.SimDeviceManager;

// Reference Phoenix6 example:

/** Subsystem for controlling the extension/retraction of climber mechanisms */
public class ClimberSubsystem extends SubsystemBase {

  ////////////////////////////////////
  // CONSTANTS
  ////////////////////////////////////

  /** Name of the CAN bus that climber motors are connected to */
  private static final String kCANBusName = "canivore";

  /** CAN ID of the climber lead motor */
  private static final int kClimberLeaderCANId = 30;
  /** CAN ID of the climber follower motor */
  private static final int kClimberFollowerCANId = 21;

  /** Gear ratio between the Falcon motors and the climber mechanism */
  private static final double kFalconToClimberGearRatio = 20.0 / 1.0;

  /** Max number of rotations to achieve full extension */
  private static final double kMaxRotations = 100.0; // TODO: set this value based on mechanisms

  ////////////////////////////////////
  // Attributes
  ////////////////////////////////////

  /** Master motor used to control climber extension */
  private final TalonFX m_climberLeader = new TalonFX(kClimberLeaderCANId, kCANBusName);
  /** Slave motor used to control climber extension */
  private final TalonFX m_climberFollower = new TalonFX(kClimberFollowerCANId, kCANBusName);

  private final StatusSignal<Double> m_leaderPositionSignal = m_climberLeader.getPosition();
  private final StatusSignal<Double> m_followerPositionSignal = m_climberLeader.getPosition();

  /* Be able to switch which control request to use based on a button press */
  /* Start at position 0, enable FOC, no feed forward, use slot 0 */
  private final PositionVoltage m_voltagePositionReq =
      new PositionVoltage(0, 0, true, 0, 0, false, false, false);
  /* Start at position 0, no feed forward, use slot 1 */
  private final PositionTorqueCurrentFOC m_torquePositionReq =
      new PositionTorqueCurrentFOC(0, 0, 0, 1, false, false, false);
  /* Keep a brake request so we can disable the motor */
  private final NeutralOut m_brakeReq = new NeutralOut();

  /** Alert displayed when a logging error occurs */
  private static final Alert s_motorConfigFailedAlert =
      new Alert("Failed to configure climber motors", Alert.AlertType.ERROR);

  /** Creates a new climber subsystem */
  public ClimberSubsystem() {
    configureMotors();
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Sets the desired climber position as a normalized percentage of maximum extension
   *
   * @param degrees Normalized percentage of full climber extension (0.0 to 1.0)
   */
  public void setExtensionPercent(double percent) {
    SmartDashboard.putNumber("climber/setExtension/percent", percent);
    m_climberLeader.setControl(m_voltagePositionReq.withPosition(percent * kMaxRotations));
  }

  public enum ClimberMotorID {
    Leader,
    Follower
  };

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Returns the current extension of a climber motor as a normalized percentage of maximum
   *
   * @motorID Climber motor whose extension is to be returned
   * @return normalized percentage of maximum climber extension (0.0 to 1.0)
   */
  public double getExtensionPercent(ClimberMotorID motorID) {
    double rotations =
        (motorID == ClimberMotorID.Leader)
            ? m_leaderPositionSignal.refresh().getValueAsDouble()
            : m_followerPositionSignal.refresh().getValueAsDouble();
    double percent = rotations / kMaxRotations; // TODO: convert to percentage of max rotations

    String motorName = (motorID == ClimberMotorID.Leader) ? "leader" : "follower";
    SmartDashboard.putNumber(String.format("climber/%s/percent", motorName), percent);
    SmartDashboard.putNumber(String.format("climber/%s/rotations", motorName), rotations);
    return percent;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Returns the current extension of the climber as a normalized percentage of maximum
   *
   * @return normalized percentage of maximum climber extension (0.0 to 1.0)
   */
  public double getExtensionPercent() {
    return getExtensionPercent(ClimberMotorID.Leader);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * This method gets called once when initializing simulation mode
   *
   * @param physicsSim Physics simulator engine for motors, etc.
   */
  public void simulationInit(SimDeviceManager simDeviceMgr) {
    simDeviceMgr.addTalonFX(m_climberLeader, 0.001);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  private void configureMotors() {
    ////////////////////////////////
    // Configure Falcon motors
    ////////////////////////////////

    // Initialize leader and follower to factory configuration
    TalonFXConfiguration falconConfig = new TalonFXConfiguration();

    // // Configure the direction of the leader motor
    MotorOutputConfigs outputConfig = falconConfig.MotorOutput;
    outputConfig.Inverted = InvertedValue.CounterClockwise_Positive;
    outputConfig.withNeutralMode(NeutralModeValue.Brake);

    // set slot gains
    //   Ks - output to overcome static friction (output)
    //   Kv - output per unit of target velocity (output/rps)
    //   Ka - output per unit of target acceleration (output/(rps/s))
    //   Kp - output per unit of error in position (output/rotation)
    //   Ki - output per unit of integrated error in position (output/(rotation*s))
    //   Kd - output per unit of error in velocity (output/rps)

    Slot0Configs slot0 = falconConfig.Slot0;
    slot0.kP = 2.4; // An error of 0.5 rotations results in 1.2 volts output
    slot0.kD = 0.1; // A change of 1 rotation per second results in 0.1 volts output
    // falconConfig.Voltage.PeakForwardVoltage = 8; // Peak output of 8 volts
    // falconConfig.Voltage.PeakReverseVoltage = -8;

    Slot1Configs slot1 = falconConfig.Slot1;
    slot1.kP = 40; // An error of 1 rotations results in 40 amps output
    slot1.kD = 2; // A change of 1 rotation per second results in 2 amps output
    falconConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40; // Peak output of 40 amps
    falconConfig.TorqueCurrent.PeakReverseTorqueCurrent = 40;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_climberLeader.getConfigurator().apply(falconConfig);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      s_motorConfigFailedAlert.set(true);
      System.out.println("Could not configure device. Error: " + status.toString());
    }

    // Configure follower motor to follow leader in the opposite direction
    m_climberFollower.setControl(new Follower(m_climberLeader.getDeviceID(), true));

    // Set the initial motor positions
    // m_climberLeader.setPosition(0);
  }
}
