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

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;
import java.util.function.BiConsumer;

/** An object used to evaluate estimated poses produced by PhotonVision */
public class PoseEstimateProcessor {

  /** Standard deviations reported for poses estimated from a single tag */
  public static final Vector<N3> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);

  /** Standard deviations reported for poses estimated using multiple tags */
  public static final Vector<N3> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

  /** Standard deviations reported for poses that should not be trusted at all */
  public static final Vector<N3> kUntrustedPoseStdDevs =
      VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);

  private final AprilTagFieldLayout m_fieldLayout;

  /** Routine called when a new robot pose is evaluated */
  private BiConsumer<Pose2d, Vector<N3>> m_poseConsumer;

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Creates an instance of the object
   *
   * @param fieldLayout April tag field layout used to evaluate poses
   */
  PoseEstimateProcessor(AprilTagFieldLayout fieldLayout) {
    m_fieldLayout = fieldLayout;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Processes a new pose estimate produced by a PhotonPoseEstimator
   *
   * @param estimatedPose The estimated pose to process
   * @param fiducialIDs Array containing the fiducial ID's of targets (tags) used to create the pose
   *     estimate
   */
  Vector<N3> processPoseEstimate(Pose2d estimatedPose, int[] fiducialIDs) {
    Vector<N3> estStdDevs = kUntrustedPoseStdDevs;

    // Iterate over each of the tags in the field to determine which tags were
    // included in the estimate.  Calculate the average distance from the estimated
    // pose to this collection of tags.
    int numTags = 0;
    double avgDist = 0;
    for (int id : fiducialIDs) {
      var tagPose = m_fieldLayout.getTagPose(id);
      if (tagPose.isEmpty()) {
        continue;
      }

      numTags++;
      double distance =
          tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
      avgDist += distance;
    }

    // CASE: no tags were used to produce the estimate
    //  return standard deviations indicating a single tag
    if (numTags == 0) {
      estStdDevs = kSingleTagStdDevs;
    } else
    // CASE: multiple tags were used to produce the estimate, return stddevs for multiple tags
    if (numTags > 1) {
      estStdDevs = kMultiTagStdDevs;
    } else
    // CASE: only one tag was used to produce the estimate
    if (numTags == 1) {
      avgDist /= numTags;

      if (avgDist > 4) {
        // If the distance from the tag was greater than 4 meters, return standard deviations
        // indicating
        // a lack of trust
        estStdDevs = kUntrustedPoseStdDevs;
      } else {
        estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
      }
    }

    // Notify the registered consumer of the new pose and its standard deviations
    m_poseConsumer.accept(estimatedPose, estStdDevs);

    return estStdDevs;
  }
}
