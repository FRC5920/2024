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
import frc.lib.logging.LoggableMotorInputs;
import frc.lib.utility.Alert;
import frc.robot.Constants.CANDevice;
import frc.robot.subsystems.pivot.PivotSubsystem.PivotMotorID;

/** Implementation of the PivotSubsystemIO interface using real hardware */
public class PivotSubsystemIOReal implements PivotSubsystemIO {

  /** I/O configuration */
  protected final PivotSubsystemIO.Config m_config;

  /** Master motor used to control the pivot angle */
  protected final TalonFX m_pivotLeader;

  /** Slave motor used to control the pivot angle */
  protected final TalonFX m_pivotFollower;

  /** CANcoder used to measure the pivot angle */
  protected final CANcoder m_canCoder;

  /** Status signal used to read the position of the pivot leader motor */
  private final StatusSignal<Double> m_leaderPositionSignal;
  /** Status signal used to read the voltage of the pivot leader motor */
  private final StatusSignal<Double> m_leaderVoltageSignal;
  /** Status signal used to read the current of the pivot leader motor */
  private final StatusSignal<Double> m_leaderCurrentSignal;
  /** Status signal used to read the temperature of the pivot leader motor */
  private final StatusSignal<Double> m_leaderTempSignal;

  /** Status signal used to read the position of the pivot follower motor */
  private final StatusSignal<Double> m_followerPositionSignal;
  /** Status signal used to read the voltage of the pivot follower motor */
  private final StatusSignal<Double> m_followerVoltageSignal;
  /** Status signal used to read the current of the pivot follower motor */
  private final StatusSignal<Double> m_followerCurrentSignal;
  /** Status signal used to read the temperature of the pivot follower motor */
  private final StatusSignal<Double> m_followerTempSignal;

  /** Signal used to read the CANcoder angle */
  private final StatusSignal<Double> m_cancoderAngle;

  /** Motion magic request used to set pivot position */
  private final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0.0);

  /** Alert displayed on failure to configure pivot motors */
  private static final Alert s_motorConfigFailedAlert =
      new Alert("Failed to configure pivot motors", Alert.AlertType.ERROR);

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Creates an instance of the I/O implementation
   *
   * @config I/O configuration
   */
  public PivotSubsystemIOReal(PivotSubsystemIO.Config config) {
    m_config = config;
    m_pivotLeader = new TalonFX(config.leaderMotorDevice.id(), config.canBus.name);
    m_pivotFollower = new TalonFX(config.followerMotorDevice.id(), config.canBus.name);
    m_canCoder = new CANcoder(config.cancoderDevice.id(), config.canBus.name);

    m_leaderPositionSignal = m_pivotLeader.getPosition();
    m_leaderVoltageSignal = m_pivotLeader.getMotorVoltage();
    m_leaderCurrentSignal = m_pivotLeader.getStatorCurrent();
    m_leaderTempSignal = m_pivotLeader.getDeviceTemp();

    m_followerPositionSignal = m_pivotFollower.getPosition();
    m_followerVoltageSignal = m_pivotFollower.getMotorVoltage();
    m_followerCurrentSignal = m_pivotFollower.getStatorCurrent();
    m_followerTempSignal = m_pivotFollower.getDeviceTemp();

    m_cancoderAngle = m_canCoder.getAbsolutePosition();
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /** Initializes and configures the I/O implementation */
  public void initialize() {
    configureCANcoder();
    configureMotors();
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /** Update logged input quantities */
  public void processInputs(PivotSubsystemInputs inputs) {
    SmartDashboard.putBoolean(
        "pivot/fusedSensorOutOfSync", m_pivotLeader.getFault_FusedSensorOutOfSync().getValue());
    SmartDashboard.putNumber(
        "pivot/closedLoopError", m_pivotLeader.getClosedLoopError().getValueAsDouble());

    // Get input measurements for leader motor
    LoggableMotorInputs leader = inputs.leader;
    leader.position = getMotorAngleDeg(PivotMotorID.Leader);
    leader.voltage = m_leaderVoltageSignal.refresh().getValueAsDouble();
    leader.current = m_leaderCurrentSignal.refresh().getValueAsDouble();
    leader.tempCelsius = m_leaderTempSignal.refresh().getValueAsDouble();

    // Get input measurements for follower motor
    LoggableMotorInputs follower = inputs.follower;
    follower.position = getMotorAngleDeg(PivotMotorID.Follower);
    follower.voltage = m_followerVoltageSignal.refresh().getValueAsDouble();
    follower.current = m_followerCurrentSignal.refresh().getValueAsDouble();
    follower.tempCelsius = m_followerTempSignal.refresh().getValueAsDouble();

    // Get CANcoder angle in degrees
    inputs.cancoderAngleRot = m_cancoderAngle.refresh().getValueAsDouble();
    inputs.cancoderAngleDeg = getAngleDeg();
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Sets the desired pivot angle in degrees
   *
   * @param degrees The desired pivot angle in degrees
   */
  @Override
  public void setAngleDeg(double degrees) {
    double rotations = Units.degreesToRotations(degrees);
    SmartDashboard.putNumber("pivot/targetAngleRot", rotations);
    m_pivotLeader.setControl(m_mmReq.withPosition(rotations).withSlot(0));
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Returns the current angle of a pivot motor in degrees
   *
   * @motorID Pivot motor whose angle is to be returned
   */
  public double getMotorAngleDeg(PivotMotorID motorID) {
    double rotations =
        (motorID == PivotMotorID.Leader)
            ? m_leaderPositionSignal.refresh().getValueAsDouble()
            : m_followerPositionSignal.refresh().getValueAsDouble();
    double degrees = Units.rotationsToDegrees(rotations % 1);
    return degrees;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /** Returns the current pivot angle in degrees */
  @Override
  public double getAngleDeg() {
    m_cancoderAngle.refresh();
    double rotations = m_cancoderAngle.refresh().getValueAsDouble();

    double degrees = Units.rotationsToDegrees(rotations);
    return degrees;
  }

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
    cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    cancoderConfig.MagnetSensor.MagnetOffset = m_config.cancoderOffsetRot;

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

    falconConfig.Voltage.PeakForwardVoltage = PivotSubsystem.kPeakPivotMotorOutputVoltage;
    falconConfig.Voltage.PeakReverseVoltage = -1.0 * PivotSubsystem.kPeakPivotMotorOutputVoltage;

    // Set up the CANcoder as the feedback sensor
    FeedbackConfigs fbCfg = falconConfig.Feedback;
    fbCfg.FeedbackRemoteSensorID = CANDevice.PivotCANcoder.id();
    fbCfg.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    fbCfg.SensorToMechanismRatio = 1.0;
    fbCfg.RotorToSensorRatio = PivotSubsystem.kFalconToPivotGearRatio;
    fbCfg.FeedbackRotorOffset = 0.0;

    // Configure trapezoidal motion profile using Motion Magic
    MotionMagicConfigs mm = falconConfig.MotionMagic;
    mm.MotionMagicAcceleration = 2.0; // acceleration in rotations per second ^2
    mm.MotionMagicCruiseVelocity = 1.0; // velocity in rotations per second
    mm.MotionMagicJerk = 30;

    // set slot 0 gains
    //   Ks - output to overcome static friction (output)
    //   Kv - output per unit of target velocity (output/rps)
    //   Ka - output per unit of target acceleration (output/(rps/s))
    //   Kp - output per unit of error in position (output/rotation)
    //   Ki - output per unit of integrated error in position (output/(rotation*s))
    //   Kd - output per unit of error in velocity (output/rps)
    Slot0Configs slot0 = falconConfig.Slot0;

    slot0.kP = 9;
    slot0.kI = 0.0;
    slot0.kD = 2;
    slot0.kV = 0.0;
    slot0.kS = 0.0; // Approximately 0.25V to get the mechanism moving

    // fbCfg.RotorToSensorRatio = m_config.pivotGearRatio;

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
    m_pivotFollower.setControl(new Follower(CANDevice.PivotLeaderMotor.id(), true));
  }
}
