////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2025 FIRST and other WPILib contributors.
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
package frc.robot.subsystems.swerveCTRE;

import frc.lib.logging.TalonFXOutputLogger;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/** A class used to log AdvantageKit outputs from a CTRE swerve subsystem */
public class CTRESwerveOutputLogger {
  private final String m_poseLogPrefix;

  /** A handle to the swerve module whose outputs are being logged */
  private final CommandSwerveDrivetrain m_swerve;

  /** A collecton of loggers that log output values swerve module motors */
  private final List<TalonFXOutputLogger> motorLoggers;

  /**
   * Creates an instance of the class, setting up logging of each swerve module's motor outputs.
   *
   * @param logPrefix Prefix applied to all logged outputs
   * @param swerve The swerve subsystem whose outputs are to be logged
   * @param numModules The number of modules in the swerve drive
   */
  public CTRESwerveOutputLogger(String logPrefix, CommandSwerveDrivetrain swerve, int numModules) {
    m_swerve = swerve;
    motorLoggers = new ArrayList<>();

    m_poseLogPrefix = logPrefix + "/pose";

    // Set up logged outputs from each swerve module
    for (int i = 0; i < numModules; ++i) {
      String modulePrefix = String.format("%s/module%d", logPrefix, i);

      // Drive motor
      motorLoggers.add(
          new TalonFXOutputLogger(modulePrefix + "/steer", swerve.getModule(i).getSteerMotor()));
      motorLoggers.add(
          new TalonFXOutputLogger(modulePrefix + "/drive", swerve.getModule(i).getDriveMotor()));
    }
  }

  public void toLog() {
    // Log the estimated pose
    Logger.recordOutput(m_poseLogPrefix, m_swerve.getPose());

    // Log motor outputs
    for (TalonFXOutputLogger motorLogger : motorLoggers) {
      motorLogger.toLog();
    }
  }
}
