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
package frc.robot.subsystems.indexer;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.lib.logging.BotLog;
import frc.lib.utility.Alert;
import java.util.ArrayList;

/** Implementation of the IntakeSubsystemIO interface using real hardware */
public class IndexerSubsystemIOReal implements IndexerSubsystemIO {

  private static final double kMotorInvert = -1.0;

  /** Motor used to drive the indexer (internal) wheels of the intake assembly */
  protected final WPI_TalonSRX m_indexerMotor;

  /**
   * LaserCAN module used to detect a gamepiece in the intake
   *
   * @see https://github.com/GrappleRobotics/LaserCAN.git
   */
  protected final LaserCan m_gamepieceSensor;

  /** Alert displayed on failure to configure the indexer motor controller */
  private static final Alert s_indexerMotorConfigFailedAlert =
      new Alert("Failed to configure indexer motor", Alert.AlertType.ERROR);

  /** Alert displayed on failure to configure the gamepiece sensor */
  private static final Alert s_laserCANConfigFailedAlert =
      new Alert("Failed to configure intake LaserCAN sensor", Alert.AlertType.ERROR);

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /** Creates an instance of the I/O implementation */
  public IndexerSubsystemIOReal() {
    m_indexerMotor = new WPI_TalonSRX(IndexerSubsystem.kIndexerMotorDevice.id());
    m_gamepieceSensor = new LaserCan(IndexerSubsystem.kLaserCANDevice.id());
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /** Initializes and configures the I/O implementation */
  public void initialize() {
    configureIndexerMotor();
    configureLaserCAN();
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * This method is called each robot cycle to process inputs to the subsystem
   *
   * @param inputs Object to populate with subsystem input values to be logged
   */
  @Override
  public void processInputs(IndexerSubsystemInputs inputs) {
    // Get input measurements for indexer
    inputs.indexer.velocity = m_indexerMotor.getMotorOutputPercent() * kMotorInvert;
    inputs.indexer.voltage = m_indexerMotor.getMotorOutputVoltage();
    inputs.indexer.current = m_indexerMotor.getStatorCurrent();
    inputs.indexer.tempCelsius = m_indexerMotor.getTemperature();

    // Get input measurements for LaserCAN
    inputs.laserCAN.fromMeasurement(m_gamepieceSensor.getMeasurement());
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Sets the desired speed of the indexer mechanism as a normalized percentage of full scale
   *
   * @param percent Normalized percentage of full speed (0.0 to 1.0)
   */
  @Override
  public void setIndexerSpeed(double percent) {
    m_indexerMotor.set(percent * kMotorInvert);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  private void configureIndexerMotor() {

    ArrayList<ErrorCode> errors = new ArrayList<>();

    final int kTimeoutMs =
        30; // Time to wait for confirmation when configuring the motor controller

    ///////////////////////////////////
    // Configure CTRE SRX controllers
    ///////////////////////////////////
    m_indexerMotor.configFactoryDefault();

    // Implement motor direction invert
    m_indexerMotor.setInverted(false);

    // Set motor to brake when not commanded
    m_indexerMotor.setNeutralMode(NeutralMode.Brake);

    // Set neutral deadband to super small 0.001 (0.1 %) because the default deadband is 0.04 (4 %)
    errors.add(m_indexerMotor.configNeutralDeadband(0.01, kTimeoutMs));

    // Configure the peak and nominal outputs, 12V means full
    errors.add(m_indexerMotor.configNominalOutputForward(0, kTimeoutMs));
    errors.add(m_indexerMotor.configNominalOutputReverse(0, kTimeoutMs));

    // Configure the maximum forward and reverse speeds
    errors.add(
        m_indexerMotor.configPeakOutputForward(IndexerSubsystem.kMaxIndexerSpeed, kTimeoutMs));
    errors.add(
        m_indexerMotor.configPeakOutputReverse(
            -1.0 * IndexerSubsystem.kMaxIndexerSpeed, kTimeoutMs));

    // Configure the motor to ramp speed up to avoid current spikes
    errors.add(m_indexerMotor.configOpenloopRamp(0.1));

    // Configure motor to limit current draw after 100 ms
    // Might want to set indexer motor current limits using measured values
    // errors.add(m_indexerMotor.configPeakCurrentDuration(100));
    // errors.add(m_indexerMotor.configPeakCurrentLimit(20));
    // errors.add(m_indexerMotor.configContinuousCurrentLimit(20));

    // Handle errors encountered during configuration
    for (ErrorCode err : errors) {
      if (err != ErrorCode.OK) {
        s_indexerMotorConfigFailedAlert.set(true);
        BotLog.Errorf("Could not configure indexer. Error: " + err.toString());
      }
    }
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /** Configure the LaserCAN gamepiece sensor */
  private void configureLaserCAN() {
    // Initialise the settings of the LaserCAN
    try {
      m_gamepieceSensor.setRangingMode(IndexerSubsystem.kLaserCANRangingMode);
      m_gamepieceSensor.setRegionOfInterest(IndexerSubsystem.kLaserCANRegionOfInterest);
      m_gamepieceSensor.setTimingBudget(IndexerSubsystem.kLaserCANTimingBudget);
    } catch (ConfigurationFailedException e) {
      s_laserCANConfigFailedAlert.set(true);
      BotLog.Errorf("LaserCAN configuration failed. Error: " + e);
    }
  }
}
