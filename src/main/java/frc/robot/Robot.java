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
package frc.robot;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.LED.ColorConstants;
import frc.lib.utility.AdvantageKitLogInitializer;
import frc.lib.utility.Alert;
import frc.lib.utility.Alert.AlertType;
import frc.lib.utility.BatteryTracker;
import frc.lib.utility.CanBusErrorAlert;
import frc.lib.utility.RobotRunMode;
import frc.robot.commands.LEDCommands.LEDsToPattern;
import frc.robot.commands.LEDCommands.LEDsToSolidColor;
import frc.robot.subsystems.LEDs.LEDSubsystem;
import frc.robot.subsystems.swerveCTRE.CommandSwerveDrivetrain;
import java.io.File;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Paths;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

public class Robot extends LoggedRobot {
  private static final String batteryNameFile = "/home/lvuser/battery-name.txt";
  /** Alert displayed if AdvantageKit Logger can't keep up with the amount of data being logged */
  private final Alert m_logReceiverQueueAlert =
      new Alert("Logging queue exceeded capacity.  Data will NOT be logged.", AlertType.ERROR);

  /** Alert displayed if CAN errors are detected */
  private final CanBusErrorAlert m_canBusErrorAlert =
      new CanBusErrorAlert("CAN errors detected.", AlertType.ERROR);

  /** Alert displayed if a low battery is detected */
  // private final Alert lowBatteryAlert =
  //     new Alert("Low battery voltage detected.", AlertType.WARNING);

  /** Alert displayed if duplicate battery is present */
  private final Alert sameBatteryAlert =
      new Alert("The battery has not been changed since the last match.", AlertType.WARNING);

  /** Container of subsystems and other components that make up the robot */
  private final RobotContainer m_robotContainer = new RobotContainer();

  /** Mechanism2D for displaying arm components on the dashboard */
  public final Mechanisms m_botMechanisms = new Mechanisms();

  /** The present selected auto command */
  private Command m_autonomousCommand;

  // /** Time when the present auto command was started */
  // private double m_autoStartTime;

  // /** True after the autonomous command duration has been printed */
  // private boolean m_autoCompletionIsPrinted;

  // /** Alert displayed when a logging error occurs */
  // private static Alert s_logReceiverQueueAlert =
  //     new Alert("Logging queue exceeded capacity, data will NOT be logged.",
  // Alert.AlertType.ERROR);

  /** Command scheduled to change the LEDs for teleop mode */
  private Command m_teleOpLEDCommand;

  /** Command scheduled to change the LEDs for disabled mode */
  private Command m_autonomousLEDCommand;

  /** Command scheduled to change the LEDs for teleop mode */
  private Command m_testLEDCommand;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    // Initialize dashboards
    m_robotContainer.dashboardSubsystem.initialize(m_robotContainer);

    // Assign a high priority to the drive train data acquisition thread
    m_robotContainer.driveTrain.getDaqThread().setThreadPriority(99);

    // Initialize AdvantageKit logging
    AdvantageKitLogInitializer logInit =
        new AdvantageKitLogInitializer(this, Constants.getMode(), Constants.kTuningMode);

    logInit.initializeLogging(Constants.kLoggingIsEnabled, Constants.getLogDirectory());

    // Scan Battery
    System.out.println("[Init] Scanning Battery");
    Logger.recordMetadata("BatteryName", "BAT-" + BatteryTracker.scanBattery(1.5));

    // Check for battery alert
    if (Constants.getMode() == RobotRunMode.REAL
        && !BatteryTracker.getName().equals(BatteryTracker.defaultName)) {
      File file = new File(batteryNameFile);
      if (file.exists()) {
        // Read previous battery name
        String previousBatteryName = "";
        try {
          previousBatteryName =
              new String(Files.readAllBytes(Paths.get(batteryNameFile)), StandardCharsets.UTF_8);
        } catch (IOException e) {
          e.printStackTrace();
        }

        if (previousBatteryName.equals(BatteryTracker.getName())) {
          // Same battery, set alert
          sameBatteryAlert.set(true);
          // TODO: Make LEDs Angry
          // Leds.getInstance().sameBattery = true;
        } else {
          // New battery, delete file
          file.delete();
        }
      }
    }
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    Logger.recordOutput(
        CommandSwerveDrivetrain.kLogPrefix + "pose", m_robotContainer.driveTrain.getPose());
    CommandScheduler.getInstance().run();

    // Alert if a logging fault is detected
    m_logReceiverQueueAlert.set(Logger.getReceiverQueueFault());

    // Update CAN error alert
    m_canBusErrorAlert.update(RobotController.getCANStatus());

    // TODO: low battery alert

    // Update display of robot mechanisms
    m_botMechanisms.updatePivotAngle(m_robotContainer.pivotSubsystem.getAngleDeg());
    m_botMechanisms.updateClimberExtension(m_robotContainer.climberSubsystem.getExtensionPercent());
    m_botMechanisms.sendToDashboard();
  }

  //////////////////////////////////////
  // SIMULATION MODE
  //////////////////////////////////////

  /** This function is run once when the robot is first started in simulation mode */
  @Override
  public void simulationInit() {}

  /** This function is called every 20 ms when the robot is running in simulation mode */
  @Override
  public void simulationPeriodic() {}

  //////////////////////////////////////
  // DISABLED MODE
  //////////////////////////////////////

  /** This function is called once when the robot is starting Disabled mode. */
  @Override
  public void disabledInit() {}

  /** This function is called every 20 ms when the robot is disabled */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when the robot leaves disabled mode */
  @Override
  public void disabledExit() {}

  //////////////////////////////////////
  // AUTONOMOUS MODE
  //////////////////////////////////////

  /** This function runs once when the robot is starting autonomous mode */
  @Override
  public void autonomousInit() {
    // Set LEDs
    LEDSubsystem ledSubsystem = m_robotContainer.ledSubsystem;
    m_autonomousLEDCommand = LEDsToPattern.newBlasterBoltPatternCommand(ledSubsystem);
    m_autonomousLEDCommand.schedule();

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called every 20 ms when the robot is running in autonomous mode */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when the robot leaves autonomous mode */
  @Override
  public void autonomousExit() {}

  //////////////////////////////////////
  // TELEOP MODE
  //////////////////////////////////////

  /** This function runs once when the robot is starting tele-operated mode */
  @Override
  public void teleopInit() {
    // Set LEDs
    LEDSubsystem ledSubsystem = m_robotContainer.ledSubsystem;
    m_teleOpLEDCommand = new LEDsToSolidColor(ledSubsystem, ColorConstants.getAllianceColor());
    m_teleOpLEDCommand.schedule();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called every 20 ms when the robot is running in tele-operated mode */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when the robot leaves tele-operated mode */
  @Override
  public void teleopExit() {}

  //////////////////////////////////////
  // TEST MODE
  //////////////////////////////////////

  /** This function runs once when the robot is starting TEST mode */
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    LEDSubsystem ledSubsystem = m_robotContainer.ledSubsystem;
    m_testLEDCommand = LEDsToPattern.newRainbowPatternCommand(ledSubsystem);
    m_testLEDCommand.schedule();
  }

  /** This function is called every 20 ms when the robot is running in TEST mode */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot leaves TEST mode */
  @Override
  public void testExit() {}
}
