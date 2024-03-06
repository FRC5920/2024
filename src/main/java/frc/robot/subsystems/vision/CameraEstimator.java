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
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

//////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * Inner class used to bundle a camera and PhotonPoseEstimator together for producing pose estimates
 */
public class CameraEstimator {

  /** Vision-based pose estimator */
  public final PhotonPoseEstimator m_estimator;

  /** Timestamp of the last pipeline result result received from the camera */
  private double m_lastTimestamp = 0.0;

  /** Processor applied to front camera pose estimates */
  private final PoseEstimateProcessor m_poseProcessor;

  /**
   * Creates an instance of the object
   *
   * @param camera PhotonCamera to use
   * @param estimator PhotonPoseEstimator to use
   */
  public CameraEstimator(PhotonPoseEstimator estimator, AprilTagFieldLayout tagLayout) {
    this.m_estimator = estimator;
    this.m_poseProcessor = new PoseEstimateProcessor(tagLayout);
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Process a new camera pipeline result by calculating a new pose estimate and using it to
   * populate outputs
   *
   * @param inputs Camera inputs to process
   * @param outputs Outputs to populate
   * @return true if a new pose estimate was calculated; else false
   */
  public boolean process(HeimdallCameraInputs inputs, HeimdallEstimatorOutputs outputs) {
    boolean newPoseCalculated = false;
    double latestTimestamp = inputs.pipelineResult.getTimestampSeconds();
    inputs.isFresh = Math.abs(latestTimestamp - m_lastTimestamp) > 1e-5;

    // Only log pipeline result if a new PipelineResult was received
    if (inputs.isFresh) {
      m_lastTimestamp = latestTimestamp; // Store the timestamp of the latest pipeline result

      // Calculate a new pose estimate and store it in outputs
      Optional<EstimatedRobotPose> estimatedPose = m_estimator.update(inputs.pipelineResult);
      outputs.noEstimate = estimatedPose.isEmpty();

      // If a new pose estimate was calculated, update outputs
      if (!outputs.noEstimate) {
        outputs.pose = estimatedPose.get().estimatedPose;
        outputs.tagIDs = getFiducialIDs(inputs.pipelineResult);

        // Calculate standard deviations for the estimate
        outputs.stdDevs =
            m_poseProcessor.processPoseEstimate(outputs.pose.toPose2d(), outputs.tagIDs);
      }
    }

    return !outputs.noEstimate;
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Utility function that returns an array containing the fiducial ID's from a PhotonPipelineResult
   *
   * @param pipelineResult Pipeline result to extract ID's from
   * @return An array containing all fiducial ID's in pipelineResult in increasing order
   */
  private static int[] getFiducialIDs(PhotonPipelineResult pipelineResult) {
    List<PhotonTrackedTarget> targets = pipelineResult.getTargets();
    int[] ids = new int[targets.size()];
    int index = 0;

    for (PhotonTrackedTarget target : targets) {
      ids[index] = target.getFiducialId();
    }

    Arrays.sort(ids); // Sort the array of fiducial ID's
    return ids;
  }
}
