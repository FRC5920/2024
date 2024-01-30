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
package frc.robot.autos;

import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.dashboard.ChangeDetector;
import frc.lib.dashboard.IDashboardTab;
import frc.lib.dashboard.LoggedSendableChooser;
import frc.lib.utility.TrajectoryUtil;
import frc.robot.RobotContainer;
import frc.robot.autos.AutoConstants.AutoPreset;
import frc.robot.autos.preset.PresetAutoBuilder;
import frc.robot.autos.preset.PresetAutoFactory;
import frc.robot.subsystems.swerveCTRE.CommandSwerveDrivetrain;
import java.util.*;

/**
 * A class supplying a Shuffleboard tab that provides user interface elements for selecting and/or
 * configuring the active autonomous routine.
 */
public class AutoDashboardTab implements IDashboardTab {

  /** Title displayed in the dashboard tab */
  private static final String kTabTitle = "Auto Builder";

  /** Width (in cells) of the field display */
  private static final int kFieldWidthCells = 24;

  /** Height (in cells) of the field display */
  private static final int kFieldHeightCells = 14;

  /** Width (in cells) of a swerve telemetry module on the dashboard (given a cell size of 32) */
  private static final int kChooserWidth = 13;

  /** Height (in cells) of a swerve telemetry module on the dashboard (given a cell size of 32) */
  private static final int kChooserHeight = 1;

  /** Width (in cells) of the pose value display */
  static final int kPoseWidthCells = 5;

  /** Height (in cells) of the pose value display */
  static final int kPoseHeightCells = 2;

  /** Empty PathPlannerTrajectory used to clear trajectories displayed on the field */
  private static final PathPlannerTrajectory kEmptyTrajectory =
      new PathPlannerTrajectory(new ArrayList<State>());

  /** Builder used to generate commands for preset auto routines */
  private PresetAutoFactory m_presetAutoFactory;

  /** The current auto command */
  private Command m_currentAutoCommand;

  /** The Shuffleboard tab */
  private ShuffleboardTab m_tab;

  /** 2d view of the field */
  private Field2d m_field2d;

  /* Auto type Chooser used to select between generated and preset autos */
  private final LoggedSendableChooser<AutoPreset> m_autoPresetChooser =
      new LoggedSendableChooser<AutoPreset>("presetAuto");

  /** Used to detect when the present alliance changes */
  private ChangeDetector<Optional<Alliance>> m_allianceChangeDetector;

  /** Used to maintain a set of displayed Trajectories */
  private HashMap<String, PathPlannerTrajectory> m_fieldTrajectories = new HashMap<>();

  /** Creates an instance of the tab */
  public AutoDashboardTab() {
    m_presetAutoFactory = new PresetAutoFactory();
    m_field2d = new Field2d();
    m_allianceChangeDetector =
        new ChangeDetector<Optional<Alliance>>(() -> DriverStation.getAlliance());
  }

  /**
   * Create and initialize dashboard widgets
   *
   * @param botContainer Container that holds robot subsystems
   */
  @Override
  public void initDashboard(RobotContainer botContainer) {
    // Initialize PathPlanner auto builder
    initializePPAutoBuilder(botContainer);

    m_tab = Shuffleboard.getTab(kTabTitle);

    // Set up the auto type chooser
    m_autoPresetChooser.addOptions(
        AutoPreset.getNames(), AutoPreset.values(), AutoPreset.Example.id);
    m_tab
        .add("Auto Preset", m_autoPresetChooser.getSendableChooser())
        .withSize(kChooserWidth, kChooserHeight)
        .withPosition(24, 0)
        .withProperties(Map.of("Label position", "LEFT"));

    // Add the 2D view of the field
    m_tab
        .add("Field", m_field2d)
        .withSize(kFieldWidthCells, kFieldHeightCells)
        .withPosition(0, 0)
        .withProperties(Map.of("Label position", "HIDDEN"));

    // Add display of the robot pose
    m_tab
        .addString("Swerve Pose (m)", () -> formatPose2d(botContainer.driveTrain.getPose(), false))
        .withSize(kPoseWidthCells, kPoseHeightCells)
        .withPosition(kPoseWidthCells * 1, kFieldHeightCells);
    m_tab
        .addString("Swerve Pose (in)", () -> formatPose2d(botContainer.driveTrain.getPose(), true))
        .withSize(kPoseWidthCells, kPoseHeightCells)
        .withPosition(kPoseWidthCells * 1, kFieldHeightCells + kPoseHeightCells);
  }

  /** Service dashboard tab widgets */
  @Override
  public void updateDashboard(RobotContainer botContainer) {
    CommandSwerveDrivetrain driveBase = botContainer.driveTrain;
    boolean allianceHasChanged = m_allianceChangeDetector.hasChanged();
    boolean autoPresetChanged = m_autoPresetChooser.hasChanged();

    if (allianceHasChanged || autoPresetChanged) {

      // Clear trajectories displayed on the field by giving all field objects an empty trajectory
      PathPlannerTrajectory emptyTrajectory = kEmptyTrajectory;
      for (String name : m_fieldTrajectories.keySet()) {
        m_fieldTrajectories.put(name, emptyTrajectory);
      }

      // Build an auto command for the selected auto preset.
      // Get its trajectories and initial pose
      AutoPreset selectedPreset = m_autoPresetChooser.getSelectedValue();
      PresetAutoBuilder presetBuilder = m_presetAutoFactory.getAutoBuilder(selectedPreset);
      m_currentAutoCommand = presetBuilder.getCommand(botContainer);
      Pose2d initialPose = presetBuilder.getInitialPose();
      List<PathPlannerTrajectory> trajectories = presetBuilder.getTrajectories();

      // Add trajectories to the map of field objects
      for (int idx = 0; idx < trajectories.size(); ++idx) {
        String trajectoryName = String.format("AutoTrajectory%d", idx);
        m_fieldTrajectories.put(trajectoryName, trajectories.get(idx));
      }

      // Send all trajectories to the field
      m_fieldTrajectories.forEach(
          (String name, PathPlannerTrajectory traj) ->
              m_field2d.getObject(name).setTrajectory(TrajectoryUtil.getWPITrajectory(traj)));

      // Reset the drivebase Gyro and set odometry to the initial pose
      driveBase.getPigeon2().setYaw(initialPose.getRotation().getDegrees());
      driveBase.seedFieldRelative(initialPose);
    }

    m_field2d.setRobotPose(driveBase.getPose());
  }

  /** Returns the current auto routine builder */
  public Command getCurrentAutoRoutine() {
    return m_currentAutoCommand;
  }

  /** Returns the field displayed in the dashboard tab */
  public Field2d getField2d() {
    return m_field2d;
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

  private static void initializePPAutoBuilder(RobotContainer botContainer) {
    CommandSwerveDrivetrain driveBase = botContainer.driveTrain;

    // TODO: initialize the PathPlanner auto builder using directions provided at
    // https://pathplanner.dev/pplib-build-an-auto.html#configure-autobuilder

    // // Configure AutoBuilder last
    // AutoBuilder.configureHolonomic(
    //         this::getPose, // Robot pose supplier
    //         this::resetPose, // Method to reset odometry (will be called if your auto has a
    // starting pose)
    //         this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
    //         this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE
    // ChassisSpeeds
    //         new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely
    // live in your Constants class
    //                 new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
    //                 new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
    //                 4.5, // Max module speed, in m/s
    //                 0.4, // Drive base radius in meters. Distance from robot center to furthest
    // module.
    //                 new ReplanningConfig() // Default path replanning config. See the API for the
    // options here
    //         ),
    //         () -> {
    //           // Boolean supplier that controls when the path will be mirrored for the red
    // alliance
    //           // This will flip the path being followed to the red side of the field.
    //           // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

    //           var alliance = DriverStation.getAlliance();
    //           if (alliance.isPresent()) {
    //             return alliance.get() == DriverStation.Alliance.Red;
    //           }
    //           return false;
    //         },
    //         this // Reference to this subsystem to set requirements
    // );
  }
}
