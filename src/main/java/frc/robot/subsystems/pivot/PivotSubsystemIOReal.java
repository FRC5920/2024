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
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.logging.LoggableMotorInputs;
import frc.lib.thirdparty.LoggedTunableNumber;
import frc.lib.utility.Alert;
import frc.robot.Constants.CANDevice;
import frc.robot.subsystems.pivot.PivotSubsystem.PivotMotorID;

/** Implementation of the PivotSubsystemIO interface using real hardware */
public class PivotSubsystemIOReal implements PivotSubsystemIO {

  // Default motion magic values
  private static final double kDefaultMMAcceleration = 3.0;
  private static final double kDefaultMMCruiseVelocity = 1.5;
  private static final double kDefaultMMJerk = 30.0;

  // Default PID gains
  //   Ks - output to overcome static friction (output)
  //   Kv - output per unit of target velocity (output/rps)
  //   Ka - output per unit of target acceleration (output/(rps/s))
  //   Kp - output per unit of error in position (output/rotation)
  //   Ki - output per unit of integrated error in position (output/(rotation*s))
  //   Kd - output per unit of error in velocity (output/rps)
  private static final double kDefault_kP = 85.0;
  private static final double kDefault_kI = 0.0;
  private static final double kDefault_kD = 1.5;
  private static final double kDefault_kV = 0.0;
  private static final double kDefault_kS = 0.0;

  // Default motion magic values for parking the pivot
  private static final double kDefaultParkMMAcceleration = 0.5;
  private static final double kDefaultParkMMCruiseVelocity = 0.25;
  private static final double kDefaultParkMMJerk = 30.0;

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

  private final PositionVoltage m_closedLoopVoltageReq =
      new PositionVoltage(0, 0, false, 0, 0, false, false, false);

  /** Motion magic configuration applied to pivot motion */
  MotionMagicConfigs m_motionMagicConfigs = new MotionMagicConfigs();

  LoggedTunableNumber m_mmAcceleration =
      new LoggedTunableNumber("Pivot/angle/MotionMagicAcceleration", kDefaultMMAcceleration);
  LoggedTunableNumber m_mmCruiseVelocity =
      new LoggedTunableNumber("Pivot/angle/MotionMagicCruiseVelocity", kDefaultMMCruiseVelocity);
  LoggedTunableNumber m_mmJerk =
      new LoggedTunableNumber("Pivot/angle/MotionMagicJerk", kDefaultMMJerk);

  /** Closed-loop gains applied to pivot motion */
  private Slot0Configs m_slot0Configs = new Slot0Configs();

  LoggedTunableNumber m_kP = new LoggedTunableNumber("Pivot/kP", kDefault_kP);
  LoggedTunableNumber m_kI = new LoggedTunableNumber("Pivot/kI", kDefault_kI);
  LoggedTunableNumber m_kD = new LoggedTunableNumber("Pivot/kD", kDefault_kD);
  LoggedTunableNumber m_kV = new LoggedTunableNumber("Pivot/kV", kDefault_kV);
  LoggedTunableNumber m_kS = new LoggedTunableNumber("Pivot/kS", kDefault_kS);

  /** Motion magic values used when parking the pivot */
  private MotionMagicConfigs m_parkMotionMagicConfig = new MotionMagicConfigs();

  LoggedTunableNumber m_mmParkAcceleration =
      new LoggedTunableNumber("Pivot/park/MotionMagicAcceleration", kDefaultParkMMAcceleration);
  LoggedTunableNumber m_mmParkCruiseVelocity =
      new LoggedTunableNumber("Pivot/park/MotionMagicCruiseVelocity", kDefaultParkMMCruiseVelocity);
  LoggedTunableNumber m_mmParkJerk =
      new LoggedTunableNumber("Pivot/park/MotionMagicJerk", kDefaultParkMMJerk);

  /** Alert displayed on failure to configure pivot motors */
  private static final Alert s_motorConfigFailedAlert =
      new Alert("Failed to configure pivot motors", Alert.AlertType.ERROR);
  /** Alert displayed on failure to configure pivot CANcoder */
  private static final Alert s_cancoderConfigFailedAlert =
      new Alert("Failed to configure pivot CANcoder", Alert.AlertType.ERROR);
  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Creates an instance of the I/O implementation
   *
   * @config I/O configuration
   */
  public PivotSubsystemIOReal() {
    m_pivotLeader =
        new TalonFX(PivotSubsystem.kLeaderMotorDevice.id(), PivotSubsystem.kCANBus.name);
    m_pivotFollower =
        new TalonFX(PivotSubsystem.kFollowerMotorDevice.id(), PivotSubsystem.kCANBus.name);
    m_canCoder = new CANcoder(PivotSubsystem.kCANcoderDevice.id(), PivotSubsystem.kCANBus.name);

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
    inputs.cancoderAngleDeg =
        Units.rotationsToDegrees(m_cancoderAngle.refresh().getValueAsDouble());
    inputs.cancoderAngleDeg =
        Units.rotationsToDegrees(m_cancoderAngle.refresh().getValueAsDouble());
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /** Returns the pivot motor to a parked position */
  @Override
  public void park(double angleDeg) {
    TalonFXConfigurator configurator = m_pivotLeader.getConfigurator();

    // Re-apply closed-loop gains
    m_slot0Configs.kP = m_kP.get();
    m_slot0Configs.kI = m_kI.get();
    m_slot0Configs.kD = m_kD.get();
    m_slot0Configs.kV = m_kV.get();
    m_slot0Configs.kS = m_kS.get();
    configurator.apply(m_slot0Configs);

    // Apply motion magic configs for regular angle set
    MotionMagicConfigs mmConfig = m_parkMotionMagicConfig;
    mmConfig.MotionMagicAcceleration = m_mmParkAcceleration.get();
    mmConfig.MotionMagicCruiseVelocity = m_mmParkCruiseVelocity.get();
    mmConfig.MotionMagicJerk = m_mmParkJerk.get();
    configurator.apply(mmConfig);

    m_pivotLeader.setControl(m_mmReq.withPosition(Units.degreesToRotations(angleDeg)).withSlot(0));
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
    TalonFXConfigurator configurator = m_pivotLeader.getConfigurator();

    // Re-apply closed-loop gains
    m_slot0Configs.kP = m_kP.get();
    m_slot0Configs.kI = m_kI.get();
    m_slot0Configs.kD = m_kD.get();
    m_slot0Configs.kV = m_kV.get();
    m_slot0Configs.kS = m_kS.get();
    configurator.apply(m_slot0Configs);

    // Apply motion magic configs for regular angle set
    m_motionMagicConfigs.MotionMagicAcceleration =
        m_mmAcceleration.get(); // acceleration in rotations per second ^2
    m_motionMagicConfigs.MotionMagicCruiseVelocity =
        m_mmCruiseVelocity.get(); // velocity in rotations per second
    m_motionMagicConfigs.MotionMagicJerk = m_mmJerk.get();
    configurator.apply(m_motionMagicConfigs);

    m_pivotLeader.setControl(m_mmReq.withPosition(rotations).withSlot(0));
    // m_pivotLeader.setControl(m_closedLoopVoltageReq.withPosition(rotations).withSlot(0));
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
    double degrees = Units.rotationsToDegrees(rotations);
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
    cancoderConfig.MagnetSensor.MagnetOffset = PivotSubsystem.kCANcoderMagnetOffsetRot;

    StatusCode status = m_canCoder.getConfigurator().apply(cancoderConfig);
    if (!status.isOK()) {
      s_cancoderConfigFailedAlert.set(true);
      System.err.println("Failed to configure CANcoder. Error: " + status.toString());
    }
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
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

    falconConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.01;

    // Set up the CANcoder as the feedback sensor
    FeedbackConfigs fbCfg = falconConfig.Feedback;
    fbCfg.FeedbackRemoteSensorID = CANDevice.PivotCANcoder.id();
    fbCfg.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    fbCfg.SensorToMechanismRatio = 1.0;
    fbCfg.RotorToSensorRatio = PivotSubsystem.kFalconToPivotGearRatio;
    fbCfg.FeedbackRotorOffset = 0.0;

    // Configure trapezoidal motion profile using Motion Magic
    m_motionMagicConfigs.MotionMagicAcceleration =
        m_mmAcceleration.get(); // acceleration in rotations per second ^2
    m_motionMagicConfigs.MotionMagicCruiseVelocity =
        m_mmCruiseVelocity.get(); // velocity in rotations per second
    m_motionMagicConfigs.MotionMagicJerk = m_mmJerk.get();
    falconConfig.MotionMagic = m_motionMagicConfigs;

    // set slot 0 gains
    //   Ks - output to overcome static friction (output)
    //   Kv - output per unit of target velocity (output/rps)
    //   Ka - output per unit of target acceleration (output/(rps/s))
    //   Kp - output per unit of error in position (output/rotation)
    //   Ki - output per unit of integrated error in position (output/(rotation*s))
    //   Kd - output per unit of error in velocity (output/rps)
    m_slot0Configs.kP = m_kP.get();
    m_slot0Configs.kI = m_kI.get();
    m_slot0Configs.kD = m_kD.get();
    m_slot0Configs.kV = m_kV.get();
    m_slot0Configs.kS = m_kS.get(); // Approximately 0.25V to get the mechanism moving
    m_slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;
    falconConfig.Slot0 = m_slot0Configs;

    // fbCfg.RotorToSensorRatio = m_config.pivotGearRatio;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_pivotLeader.getConfigurator().apply(falconConfig);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      s_motorConfigFailedAlert.set(true);
      System.err.println("Failed to configure Pivot motor. Error: " + status.toString());
    }

    // Configure pivot follower motor to follow pivot leader in the opposite direction
    m_pivotFollower.setControl(new Follower(CANDevice.PivotLeaderMotor.id(), true));

    // Initialize the motor position to the CANCoder position
    do {
      status =
          m_pivotLeader.setPosition(m_canCoder.getAbsolutePosition().refresh().getValueAsDouble());
    } while (!status.isOK());

    m_pivotLeader.clearStickyFaults();
  }
}
