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

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.dashboard.ChangeDetector;
import frc.lib.dashboard.IDashboardTab;
import frc.lib.dashboard.LoggedSendableChooser;
import frc.robot.RobotContainer;
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

  /** Empty Trajectory used to clear trajectories displayed on the field */
  private static final Trajectory kEmptyTrajectory = new Trajectory();

  /** Builder used to generate commands for preset auto routines */
  // private PresetAutoFactory m_presetAutoFactory;

  /** The current auto command */
  // private Command m_currentAutoCommand;

  /** The Shuffleboard tab */
  private ShuffleboardTab m_tab;

  /** 2d view of the field */
  private Field2d m_field2d;

  /* Auto type Chooser used to select between generated and preset autos */
  private final LoggedSendableChooser<Command> m_autoPresetChooser;

  /** Used to detect when the present alliance changes */
  private ChangeDetector<Optional<Alliance>> m_allianceChangeDetector;

  /** Used to maintain a set of displayed Trajectories */
  private HashMap<String, Trajectory> m_fieldTrajectories = new HashMap<>();

  /** Creates an instance of the tab */
  public AutoDashboardTab() {
    // m_presetAutoFactory = new PresetAutoFactory();
    m_field2d = new Field2d();
    m_allianceChangeDetector =
        new ChangeDetector<Optional<Alliance>>(() -> DriverStation.getAlliance());

    // Load all deployed auto paths and construct an auto preset chooser with them
    final SendableChooser<Command> ppAutoChooser = AutoBuilder.buildAutoChooser();
    m_autoPresetChooser = new LoggedSendableChooser<Command>("presetAuto", ppAutoChooser);
  }

  /**
   * Create and initialize dashboard widgets
   *
   * @param botContainer Container that holds robot subsystems
   */
  @Override
  public void initDashboard(RobotContainer botContainer) {
    m_tab = Shuffleboard.getTab(kTabTitle);

    // Set up the auto type chooser
    // m_autoPresetChooser.addOptions(
    //     AutoPreset.getNames(), AutoPreset.values(), AutoPreset.Example.id);
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

    // // Add display of the robot pose
    // m_tab
    //     .addString("Swerve Pose (m)", () -> formatPose2d(botContainer.driveTrain.getPose(),
    // false))
    //     .withSize(kPoseWidthCells, kPoseHeightCells)
    //     .withPosition(kPoseWidthCells * 1, kFieldHeightCells);
    // m_tab
    //     .addString("Swerve Pose (in)", () -> formatPose2d(botContainer.driveTrain.getPose(),
    // true))
    //     .withSize(kPoseWidthCells, kPoseHeightCells)
    //     .withPosition(kPoseWidthCells * 1, kFieldHeightCells + kPoseHeightCells);

  }

  /** Service dashboard tab widgets */
  @Override
  public void updateDashboard(RobotContainer botContainer) {
    boolean allianceHasChanged = m_allianceChangeDetector.hasChanged();
    boolean autoPresetChanged = m_autoPresetChooser.hasChanged();

    if (allianceHasChanged || autoPresetChanged) {
      String autoName = m_autoPresetChooser.getChoiceName();
      if (autoName != "None") {
        boolean shouldFlipPath = false;
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          shouldFlipPath = (alliance.get() == Alliance.Red);
        }

        List<Trajectory> trajectories = getAutoTrajectoryList(autoName, shouldFlipPath);

        // Clear trajectories displayed on the field by giving all FieldObjects an empty trajectory
        for (String name : m_fieldTrajectories.keySet()) {
          m_fieldTrajectories.put(name, kEmptyTrajectory);
        }

        // Add current auto trajectories to the map of field objects
        for (int idx = 0; idx < trajectories.size(); ++idx) {
          String trajectoryName = String.format("AutoTrajectory%d", idx);
          m_fieldTrajectories.put(trajectoryName, trajectories.get(idx));
        }

        // Send all trajectories to the field
        m_fieldTrajectories.forEach(
            (String name, Trajectory traj) -> m_field2d.getObject(name).setTrajectory(traj));

        // Set the current robot pose
        Pose2d initialPose = trajectories.get(0).getInitialPose();
        botContainer.driveTrain.seedFieldRelative(initialPose);
      }

      // Reset the drivebase Gyro and set odometry to the initial pose
      // driveBase.getPigeon2().setYaw(initialPose.getRotation().getDegrees());
      // driveBase.seedFieldRelative(initialPose);
    }

    Pose2d pose = botContainer.driveTrain.getPose();
    FieldObject2d robotObject = m_field2d.getRobotObject();
    robotObject.setPose(pose);
    SmartDashboard.putString("CurrentPose", formatPose2d(pose, false));
  }

  /** Returns the current auto routine builder */
  public Command getCurrentAutoRoutine() {
    return m_autoPresetChooser.getSelectedValue();
  }

  /** Returns the field displayed in the dashboard tab */
  public Field2d getField2d() {
    return m_field2d;
  }

  private static List<Trajectory> getAutoTrajectoryList(String autoName, boolean shouldFlipPath) {
    List<Trajectory> trajectories = new ArrayList<>();
    List<PathPlannerPath> paths = PathPlannerAuto.getPathGroupFromAutoFile(autoName);
    for (PathPlannerPath path : paths) {
      PathPlannerPath allianceAdjustedPath = shouldFlipPath ? path.flipPath() : path;
      Pose2d startingPose = allianceAdjustedPath.getPreviewStartingHolonomicPose();

      ChassisSpeeds speeds = new ChassisSpeeds();
      PathPlannerTrajectory ppTrajectory =
          allianceAdjustedPath.getTrajectory(speeds, startingPose.getRotation());
      trajectories.add(ppToWPILibTrajectory(ppTrajectory));
    }
    return trajectories;
  }

  private static Trajectory ppToWPILibTrajectory(PathPlannerTrajectory ppTrajectory) {
    ArrayList<State> wpiLibStates = new ArrayList<>();

    for (PathPlannerTrajectory.State ppState : ppTrajectory.getStates()) {
      wpiLibStates.add(
          new State(
              ppState.timeSeconds,
              ppState.velocityMps,
              ppState.accelerationMpsSq,
              ppState.getTargetHolonomicPose(),
              ppState.curvatureRadPerMeter));
    }

    return new Trajectory(wpiLibStates);
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
