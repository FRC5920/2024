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

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.lib.logging.BotLog;
import frc.lib.utility.Alert;
import java.util.ArrayList;

/** Implementation of the IntakeSubsystemIO interface using real hardware */
public class IndexerSubsystemIOReal implements IndexerSubsystemIO {

  private static final double kMotorInvert = -1.0;

  /** Motor used to drive the indexer (internal) wheels of the intake assembly */
  protected final WPI_TalonSRX m_indexerMotor;

  ////////////////////////////////////
  // Gamepiece limit switch
  ////////////////////////////////////

  /** Digital input channel connected to the gamepiece limit switch */
  private static final int kLimitSwitchDIChannel = 9;

  /** Digital input used for the intake limit switch */
  private DigitalInput m_limitSwitchInput = new DigitalInput(kLimitSwitchDIChannel);

  /** Object used to debounce the limit switch */
  private Debouncer m_limitSwitchDebouncer = new Debouncer(0.05, DebounceType.kBoth);

  /** Alert displayed on failure to configure the indexer motor controller */
  private static final Alert s_indexerMotorConfigFailedAlert =
      new Alert("Failed to configure indexer motor", Alert.AlertType.ERROR);

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /** Creates an instance of the I/O implementation */
  public IndexerSubsystemIOReal() {
    m_indexerMotor = new WPI_TalonSRX(IndexerSubsystem.kIndexerMotorDevice.id());
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /** Initializes and configures the I/O implementation */
  public void initialize() {
    configureIndexerMotor();
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
    inputs.limitSwitch = m_limitSwitchDebouncer.calculate(m_limitSwitchInput.get());
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
}
