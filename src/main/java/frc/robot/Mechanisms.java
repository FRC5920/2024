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

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import java.util.Optional;

/** Class to keep all the mechanism-specific objects together and out of the main example */
public class Mechanisms {
  double HEIGHT = 1; // Controls the height of the mech2d SmartDashboard
  double WIDTH = 1; // Controls the height of the mech2d SmartDashboard

  private static final double kPivotRootX = 0.5;
  private static final double kPivotRootY = 0.25;

  private static final double kClimberBaseLength = 0.2;
  private static final double kClimberMinLength = 0.05;
  private static final double kClimberMaxLength = 0.4;
  private static final double kClimberInitialLength = kClimberMinLength;
  private static final double kInitialPivotAngleDeg = 0.0;

  /** Official FIRST alliance colors */
  private static final Color8Bit kAllianceBlue = new Color8Bit(Color.kFirstBlue);

  private static final Color8Bit kAllianceRed = new Color8Bit(Color.kFirstRed);
  private static final Color8Bit kNoAllianceGray = new Color8Bit(Color.kGray);

  /** Mechanism used to display the arm and climber assembly on the dashboard */
  private final Mechanism2d m_mechanism = new Mechanism2d(WIDTH, HEIGHT);

  /////////////////////////////
  // LEADER MECHANISIMS
  /////////////////////////////
  /** Mechanism ligament used to display the arm segment attached to the pivot motors */
  private final MechanismLigament2d m_leaderPivot =
      m_mechanism
          .getRoot("pivotRoot", kPivotRootX, kPivotRootY)
          .append(
              new MechanismLigament2d(
                  "pivotSegment",
                  kClimberBaseLength,
                  kInitialPivotAngleDeg,
                  10,
                  new Color8Bit(Color.kYellow)));

  /** Mechanism ligament used to show the climber segment of the arm */
  private final MechanismLigament2d m_leaderClimber =
      m_leaderPivot.append(
          new MechanismLigament2d(
              "climberSegment", kClimberInitialLength, 0.0, 8, new Color8Bit(Color.kGray)));

  /////////////////////////////////////////////////////
  // DECORATION MECHANISIMS
  //  These mechanisms are not manipulated directly.
  //  They serve only as 'decorations'.
  /////////////////////////////////////////////////////

  /** Mechanism used to decorate the climber segment with a 'hook' */
  private final MechanismLigament2d m_climberHook =
      m_leaderClimber.append(
          new MechanismLigament2d("climberhook", 0.075, 145.0, 6, new Color8Bit(Color.kGray)));

  /** Mechanism ligament used to display a representation of the support for the pivot */
  private final MechanismLigament2d m_pivotBase =
      m_mechanism
          .getRoot("pivotOffsetRoot", kPivotRootX, 0.1)
          .append(
              new MechanismLigament2d(
                  "pivotOffset", 0.15, 90.0, 20, new Color8Bit(Color.kGoldenrod)));

  /** Mechanism ligament used to display a representation of robot bumpers */
  private final MechanismLigament2d m_bumpers =
      m_mechanism
          .getRoot("robotBumperRoot", 0.2, 0.1)
          .append(new MechanismLigament2d("robotBumper", 0.6, 0.0, 16, new Color8Bit(Color.kPlum)));

  /**
   * Updates the angle of pivot mechanisms
   *
   * @param angleDeg Angle of the leader motor
   */
  public void updatePivotAngle(double angleDeg) {
    m_leaderPivot.setAngle(angleDeg);
  }

  /**
   * Updates the extension of climber mechanisms
   *
   * @param percent Normalized percentage (0.0 to 1.0) of full climber extension
   */
  public void updateClimberExtension(double percent) {
    m_leaderClimber.setLength(kClimberMaxLength * percent);
  }

  /** Sends the present mechanism values to the dashboard widget */
  public void sendToDashboard() {
    // Refresh the bot base with the Color of the active alliance
    Color8Bit allianceColor = kNoAllianceGray;
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      allianceColor = (alliance.get() == Alliance.Blue) ? kAllianceBlue : kAllianceRed;
    }
    m_bumpers.setColor(allianceColor);

    SmartDashboard.putData("mech2d", m_mechanism); // Creates mech2d in SmartDashboard
  }
}
