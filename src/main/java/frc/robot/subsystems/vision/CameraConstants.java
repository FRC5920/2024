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
package frc.robot.subsystems.vision;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.CameraID;
import java.util.Map;

/** Add your docs here. */
public class CameraConstants {
  /** FRONT-looking camera mounted on the RIGHT side of the robot */
  public static class FrontCamera {

    /** PhotonVision camera */
    public static final CameraID kCamera = CameraID.FrontCamera;

    /** Location of the camera relative to the center of the bot (meters, radians) */
    public static class Location {
      // Camera points toward the front RIGHT corner
      public static final double xOffset = 0.115378; // Taken from CAD model 2/25/2024
      public static final double yOffset = -0.282293; // Taken from CAD model 2/25/2024
      public static final double zOffset = 0.1;
      /** Rotation of the camera about its Z-axis */
      public static final double yaw = Units.degreesToRadians(-45.0);

      public static final double pitch = Units.degreesToRadians(-35);
    }
  }

  /** REAR-looking camera mounted on the LEFT side of the robot */
  public static class RearCamera {
    /** PhotonVision camera */
    public static final CameraID kCamera = CameraID.RearCamera;

    /** Location of the camera relative to the center of the bot (meters, radians) */
    public static class Location {
      // Camera points toward the rear left corner
      public static final double xOffset = 0.075053; // Taken from CAD model 2/25/2024
      public static final double yOffset = 0.280637; // Taken from CAD model 2/25/2024
      public static final double zOffset = 0.1; // Measured 2/25/2024
      /** Rotation of the camera about its Z-axis */
      public static final double yaw = Units.degreesToRadians(135.0);

      public static final double pitch = Units.degreesToRadians(-35);
    }
  }

  /** Resolution of a tag cameras in pixels */
  public static class TagCameraResolution {
    public static final int widthPx = 1280;
    public static final int heightPx = 720;
    public static final double FOVDegrees = 70;
  }

  /** Target detection pipelines used in conjunction with the Target Camera */
  public enum TargetPipeline {
    Note(0),
    AprilTag2D(1);

    /** Pipeline index used for target detection */
    public final int index;

    private TargetPipeline(int index) {
      this.index = index;
    }

    private static final Map<Integer, TargetPipeline> intToEnumMap = Map.of(0, Note, 1, AprilTag2D);

    public static TargetPipeline fromInt(int index) {
      return intToEnumMap.get(index);
    }
  }
}
