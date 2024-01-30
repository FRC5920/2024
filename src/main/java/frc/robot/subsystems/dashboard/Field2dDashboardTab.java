////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2023 FIRST and other WPILib contributors.
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
package frc.robot.subsystems.dashboard;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.lib.dashboard.IDashboardTab;
import frc.robot.RobotContainer;
import java.util.Map;
import java.util.function.Supplier;

////////////////////////////////////////////////////////////////////////////////////////////////////
/** A class supplying a Shuffleboard tab that displays the robot's pose on a 2D field */
public class Field2dDashboardTab implements IDashboardTab {
  /** Title used for a dashboard tab that displays the field */
  static final String kTabTitle = "Field";

  /** Width (in cells) of the field display */
  static final int kFieldWidthCells = 21;

  /** Height (in cells) of the field display */
  static final int kFieldHeightCells = 12;

  /** Width (in cells) of the pose value display */
  static final int kPoseWidthCells = 5;

  /** Height (in cells) of the pose value display */
  static final int kPoseHeightCells = 2;

  /** Supplier for the robot pose */
  private Supplier<Pose2d> m_poseSupplier;

  /** The Shuffleboard tab to display in */
  private ShuffleboardTab m_tab;

  /** 2d view of the field */
  private Field2d m_field2d;

  /** Creates an instance of the tab */
  public Field2dDashboardTab() {
    m_field2d = new Field2d();
  }

  /**
   * Create and initialize dashboard widgets
   *
   * @param botContainer Container that holds robot subsystems
   */
  @Override
  public void initDashboard(RobotContainer botContainer) {
    m_poseSupplier = () -> botContainer.driveTrain.getPose();

    m_tab = Shuffleboard.getTab(kTabTitle);

    // Add the 2D view of the field
    m_tab
        .add("Field", m_field2d)
        .withSize(kFieldWidthCells, kFieldHeightCells)
        .withPosition(0, 0)
        .withProperties(Map.of("Label position", "HIDDEN"));

    // // Add the pose estimator's pose
    // m_tab
    //     .addString("Pose Estimator (m)", () -> formatPose2d(poseSubsystem.getCurrentPose(),
    // false))
    //     .withSize(kPoseWidthCells, kPoseHeightCells)
    //     .withPosition(kPoseWidthCells * 0, kFieldHeightCells);
    // m_tab
    //     .addString("Pose Estimator (in)", () -> formatPose2d(poseSubsystem.getCurrentPose(),
    // true))
    //     .withSize(kPoseWidthCells, kPoseHeightCells)
    //     .withPosition(kPoseWidthCells * 0, kFieldHeightCells + kPoseHeightCells);

    // Add the pose value
    m_tab
        .addString("Swerve Pose (m)", () -> formatPose2d(m_poseSupplier.get(), false))
        .withSize(kPoseWidthCells, kPoseHeightCells)
        .withPosition(kPoseWidthCells * 1, kFieldHeightCells);
    m_tab
        .addString("Swerve Pose (in)", () -> formatPose2d(m_poseSupplier.get(), true))
        .withSize(kPoseWidthCells, kPoseHeightCells)
        .withPosition(kPoseWidthCells * 1, kFieldHeightCells + kPoseHeightCells);
  }

  /** Service dashboard tab widgets */
  @Override
  public void updateDashboard(RobotContainer botContainer) {
    // FieldObject2d estimatorPoseObject = m_field2d.getRobotObject();
    // estimatorPoseObject.setPose(m_poseSupplier.get());

    // FieldObject2d swervePoseObject = m_field2d.getObject("Odometry");
    // swervePoseObject.setPose(botContainer.swerveSubsystem.getPose());
    m_field2d.setRobotPose(m_poseSupplier.get());
  }

  private static String formatPose2d(Pose2d pose, boolean asInches) {
    if (asInches) {
      return String.format(
          "(%.2f in, %.2f in) %.2f degrees",
          Units.metersToInches(pose.getX()),
          Units.metersToInches(pose.getY()),
          pose.getRotation().getDegrees());
    } else {
      return String.format(
          "(%.2f m, %.2f m) %.2f degrees",
          pose.getX(), pose.getY(), pose.getRotation().getDegrees());
    }
  }
}
