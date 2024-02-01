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
package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.utility.Alert;
import frc.robot.sim.SimDeviceManager;

// Reference Phoenix6 example:

/**
 * Subsystem for the pivot point of the robot arm
 *
 * @see CTRE Phoenix 6 closed-loop position example:
 *     https://github.com/CrossTheRoadElec/Phoenix6-Examples/tree/main/java/PositionClosedLoop
 */
public class PivotSubsystem extends SubsystemBase {

  ////////////////////////////////////
  // CONSTANTS
  ////////////////////////////////////

  /** Name of the CAN bus that pivot and extender motors are connected to */
  private static final String kCANBusName = "canivore";

  /** CAN ID of the pivot lead motor */
  private static final int kPivotLeaderCANId = 20;
  /** CAN ID of the pivot follower motor */
  private static final int kPivotFollowerCANId = 21;
  /** CAN ID of the CANcoder used to measure pivot angle */
  private static final int kPivotCANcoderCANId = 22;

  /** Offset of the CANcoder magnet in rotations */
  private static final double kCANcoderMagnetOffsetRot = 0.4;

  /** Gear ratio between the Falcon motors and the pivot axle (motor-to-mechanism) */
  private static final double kFalconToPivotGearRatio = 20.0 / 1.0;

  /** Initial position of the motor */
  public static final double kParkAngleDeg = 60.0;

  ////////////////////////////////////
  // Attributes
  ////////////////////////////////////

  /** Master motor used to control the pivot angle */
  private final TalonFX m_pivotLeader = new TalonFX(kPivotLeaderCANId, kCANBusName);
  /** Slave motor used to control the pivot angle */
  private final TalonFX m_pivotFollower = new TalonFX(kPivotFollowerCANId, kCANBusName);

  /** CANcoder used to measure the pivot angle */
  private final CANcoder m_canCoder = new CANcoder(kPivotCANcoderCANId, kCANBusName);

  /** Signal used to read the CANcoder angle */
  private final StatusSignal<Double> m_cancoderAngle = m_canCoder.getPosition();

  /** Motion magic request used to set pivot position */
  private final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0.0);

  /** Alert displayed when a logging error occurs */
  private static final Alert s_motorConfigFailedAlert =
      new Alert("Failed to configure pivot motors", Alert.AlertType.ERROR);

  /** Creates a new PivotSubsystem. */
  public PivotSubsystem() {
    configureCANcoder();
    configureMotors();
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Sets the desired pivot angle in degrees
   *
   * @param degrees The desired pivot angle in degrees
   */
  public void setAngleDeg(double degrees) {
    double rotations = Units.degreesToRotations(degrees);
    SmartDashboard.putNumber("pivot/setAngleDeg/deg", degrees);
    SmartDashboard.putNumber("pivot/setAngleDeg/rot", rotations);
    m_pivotLeader.setControl(m_mmReq.withPosition(rotations).withSlot(0));
  }

  public enum PivotMotorID {
    Leader,
    Follower
  };

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Returns the current angle of a pivot motor in degrees
   *
   * @motorID Pivot motor whose angle is to be returned
   */
  public double getMotorAngleDeg(PivotMotorID motorID) {
    double rotations =
        (motorID == PivotMotorID.Leader)
            ? m_pivotLeader.getPosition().getValueAsDouble()
            : m_pivotFollower.getPosition().getValueAsDouble();
    double degrees = Units.rotationsToDegrees(rotations % 1);

    String motorName = (motorID == PivotMotorID.Leader) ? "leader" : "follower";
    SmartDashboard.putNumber(String.format("pivot/%s/deg", motorName), degrees);
    SmartDashboard.putNumber(String.format("pivot/%s/rot", motorName), rotations);
    return degrees;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /** Returns the current pivot angle in degrees */
  public double getAngleDeg() {
    m_cancoderAngle.refresh();
    double rotations = m_cancoderAngle.getValueAsDouble();
    double degrees = Units.rotationsToDegrees(rotations % 1);

    SmartDashboard.putNumber("pivot/CANcoder/deg", degrees);
    SmartDashboard.putNumber("pivot/CANcoder/rot", rotations);
    return degrees;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * This method gets called once when initializing simulation mode
   *
   * @param physicsSim Physics simulator engine for motors, etc.
   */
  public void simulationInit(SimDeviceManager simDeviceMgr) {
    simDeviceMgr.addTalonFXWithFusedCANcoder(
        m_pivotLeader, m_canCoder, 1.0 / kFalconToPivotGearRatio, 0.001);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /** This method gets called once per scheduler run */
  @Override
  public void periodic() {}

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /** This method gets called once per scheduler run in simulation mode */
  @Override
  public void simulationPeriodic() {}

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /** Configures motors used to control the pivot angle */
  private void configureCANcoder() {
    ////////////////////////////////
    // Configure CANcoder
    ////////////////////////////////

    // Configure CANcoder to zero the magnet appropriately
    // Ref:
    //
    // https://v6.docs.ctr-electronics.com/en/2023-pro/docs/api-reference/api-usage/device-specific/talonfx/remote-sensors.html#fusedcancoder
    CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
    cancoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    cancoderConfig.MagnetSensor.MagnetOffset = kCANcoderMagnetOffsetRot;
    m_canCoder.getConfigurator().apply(cancoderConfig);
  }

  private void configureMotors() {
    ////////////////////////////////
    // Configure Falcon motors
    ////////////////////////////////

    // Initialize leader and follower to factory configuration
    TalonFXConfiguration falconConfig = new TalonFXConfiguration();

    // // Configure the direction of the pivot leader motor
    MotorOutputConfigs outputConfig = falconConfig.MotorOutput;
    outputConfig.Inverted = InvertedValue.CounterClockwise_Positive;
    outputConfig.withNeutralMode(NeutralModeValue.Brake);

    // Configure trapezoidal motion profile using Motion Magic
    MotionMagicConfigs mm = falconConfig.MotionMagic;
    mm.MotionMagicCruiseVelocity = Units.degreesToRotations(360.0 * kFalconToPivotGearRatio) * 2.0;
    mm.MotionMagicAcceleration = Units.degreesToRotations(720.0 * kFalconToPivotGearRatio) * 2.0;
    mm.MotionMagicJerk = Units.degreesToRotations(1000.0 * kFalconToPivotGearRatio);

    // set slot 0 gains
    //   Ks - output to overcome static friction (output)
    //   Kv - output per unit of target velocity (output/rps)
    //   Ka - output per unit of target acceleration (output/(rps/s))
    //   Kp - output per unit of error in position (output/rotation)
    //   Ki - output per unit of integrated error in position (output/(rotation*s))
    //   Kd - output per unit of error in velocity (output/rps)
    Slot0Configs slot0 = falconConfig.Slot0;

    slot0.kP = 400.0;
    slot0.kI = 0;
    slot0.kD = 80.0;
    slot0.kV = 0.12;
    slot0.kS = 0.25; // Approximately 0.25V to get the mechanism moving

    FeedbackConfigs fbCfg = falconConfig.Feedback;
    fbCfg.FeedbackRemoteSensorID = m_canCoder.getDeviceID();
    fbCfg.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    fbCfg.SensorToMechanismRatio = 1.0;
    fbCfg.RotorToSensorRatio = kFalconToPivotGearRatio;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_pivotLeader.getConfigurator().apply(falconConfig);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      s_motorConfigFailedAlert.set(true);
      System.out.println("Could not configure device. Error: " + status.toString());
    }

    // Configure pivot follower motor to follow pivot leader in the opposite direction
    m_pivotFollower.setControl(new Follower(m_pivotLeader.getDeviceID(), true));

    // Set the initial motor position
    // m_pivotLeader.setPosition(0);
  }
}
