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
package frc.lib.utility;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.GenBuildInfo;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/** Provides an */
public class AdvantageKitLogInitializer {
  public final LoggedRobot m_robot;
  public final RobotRunMode m_runMode;
  public final boolean m_tuning;

  /** Alert displayed when no log path is set up */
  private final Alert m_logNoFileAlert =
      new Alert(
          "No log path set for current robot. Data will NOT be logged.", Alert.AlertType.WARNING);

  /**
   * Creates an instance of the initializer
   *
   * @param robot A reference to the LoggedRobot object declared in Robot.java
   * @param runMode The mode the bot is running in
   * @param tuning Set this to true when tuning or characterizing the robot
   */
  public AdvantageKitLogInitializer(LoggedRobot robot, RobotRunMode runMode, boolean tuning) {
    m_robot = robot;
    m_runMode = runMode;
    m_tuning = tuning;
  }

  /**
   * Called to initialize logging using AdvantageKit
   *
   * @param loggingIsEnabled true if logging is enabled; else false to disable logging
   * @param logDirectory Path of a directory where log files will be written
   */
  public void initializeLogging(boolean loggingIsEnabled, String logDirectory) {
    if (loggingIsEnabled) {

      logMetadata(); // Log metadata and build info

      // Set up logging
      configureLogging(logDirectory);

      // See http://bit.ly/3YIzFZ6 for more information on timestamps in AdvantageKit.
      // Logger.disableDeterministicTimestamps()

      // Start the AdvantageKit logger
      m_robot.setUseTiming(m_runMode != RobotRunMode.REPLAY);
      Logger.start();

      // Set up logging of active commands in the scheduler
      setupActiveCommandLogging();
    } else {
      Logger.end();
    }
  }

  /** This helper method logs build info from the generated file, GenBuildInfo.java */
  private void logMetadata() {
    // Record metadata
    Logger.recordMetadata("Robot Mode", m_runMode.toString());
    Logger.recordMetadata("TuningMode", Boolean.toString(m_tuning));
    Logger.recordMetadata("RuntimeType", LoggedRobot.getRuntimeType().toString());
    Logger.recordMetadata("ProjectName", GenBuildInfo.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", GenBuildInfo.BUILD_DATE);
    Logger.recordMetadata("GitSHA", GenBuildInfo.GIT_SHA);
    Logger.recordMetadata("GitDate", GenBuildInfo.GIT_DATE);
    Logger.recordMetadata("GitBranch", GenBuildInfo.GIT_BRANCH);
    // Scan Battery
    System.out.println("[Init] Scanning Battery");
    Logger.recordMetadata("BatteryName", "BAT-" + BatteryTracker.scanBattery(1.5));

    switch (GenBuildInfo.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }
  }

  /** Helper function that configures the AdvantageKit logger */
  private void configureLogging(String logDirectory) {
    Path logDirectoryPath = Paths.get(logDirectory);
    // Set up data receivers & replay source
    switch (m_runMode) {
      case REAL:
        if (logDirectory != null) {
          // Create the log directory if it doesn't exist
          createLogDirectory(logDirectoryPath);
          // Running on a real robot, log to the specified log directory
          Logger.addDataReceiver(new WPILOGWriter(logDirectory));
        } else {
          m_logNoFileAlert.set(true);
        }
        Logger.addDataReceiver(new NT4Publisher());
        // Log data from the power distribution hub (PDH)
        LoggedPowerDistribution.getInstance();
        break;

      case SIM:
        // Create the specified log directory if it doesn't exist
        if (logDirectory != null) {
          // Create the log directory if it doesn't exist
          createLogDirectory(logDirectoryPath);
          Logger.addDataReceiver(new WPILOGWriter(logDirectory));
        }

        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        String replayLogPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(replayLogPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(replayLogPath, "_sim")));
        break;
    }
  }

  /** Sets up logging of active Commands in the scheduler */
  private void setupActiveCommandLogging() {

    // Set up logging of active commands
    Map<String, Integer> commandCounts = new HashMap<>();
    BiConsumer<Command, Boolean> logCommandFunction =
        (Command command, Boolean active) -> {
          String name = command.getName();
          int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
          commandCounts.put(name, count);
          Logger.recordOutput(
              "CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()), active);
          Logger.recordOutput("CommandsAll/" + name, count > 0);
        };
    CommandScheduler.getInstance()
        .onCommandInitialize(
            (Command command) -> {
              logCommandFunction.accept(command, true);
            });
    CommandScheduler.getInstance()
        .onCommandFinish(
            (Command command) -> {
              logCommandFunction.accept(command, false);
            });
    CommandScheduler.getInstance()
        .onCommandInterrupt(
            (Command command) -> {
              logCommandFunction.accept(command, false);
            });
  }

  /** Creates a specified directory for log files or sets the logNoFileAlert */
  private void createLogDirectory(Path logDirectory) {
    try {
      Files.createDirectories(logDirectory);
      Logger.addDataReceiver(new WPILOGWriter(logDirectory.toString()));
    } catch (IOException e) {
      m_logNoFileAlert.set(true);
    }
  }
}
