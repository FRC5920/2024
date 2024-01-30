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

import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.utility.RobotRunMode;
import frc.lib.utility.RobotType;
import java.util.Map;
import org.littletonrobotics.junction.LoggedRobot;

public final class Constants {

  public static final double robotPeriodSec = LoggedRobot.defaultPeriodSecs;

  /** Set this to true to enable logging with AdvantageKit */
  public static final boolean kLoggingIsEnabled = true;

  /** Set the value of logPlaybackIsEnabled to true when replaying log files */
  public static final boolean kLogPlaybackIsEnabled = false;

  /** This constant indicates the type of bot the code applies to */
  public static final RobotType kRobotType = RobotType.PrototypeBot;

  /** This constant should be set to true when tuning or characterizing the robot */
  public static final boolean kTuningMode = false;

  /** A map of directories where log files should be stored */
  private static final Map<RobotRunMode, String> logDirectories =
      Map.of(RobotRunMode.REAL, "/media/sda2/", RobotRunMode.SIM, "AKitBotLogs/");

  /** Returns the current mode the robot is executing in */
  public static RobotRunMode getRobotMode() {
    if (kLogPlaybackIsEnabled) {
      return RobotRunMode.REPLAY;
    } else {
      return RobotBase.isSimulation() ? RobotRunMode.SIM : RobotRunMode.REAL;
    }
  }

  // Returns the directory used for logging
  public static String getLogDirectory() {
    String logPath = new String();
    switch (getRobotMode()) {
      case REAL:
        logPath = "/media/sda2/";
        break;
      case SIM:
        logPath = "logs/simulation";
        break;
      case REPLAY:
        logPath = "logs/replay";
        break;
    }
    return logPath;
  }

  /** Returns the configured robot run mode */
  public static RobotRunMode getMode() {
    return RobotBase.isReal()
        ? RobotRunMode.REAL
        : (kLogPlaybackIsEnabled ? RobotRunMode.REPLAY : RobotRunMode.SIM);
  }
}
