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

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.vision.HeimdallSubsystemInputs.PoseEstimateInputs;
import frc.robot.subsystems.vision.HeimdallSubsystemOutputs.PoseEstimateOutputs;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** Implementation of the HeimdallSubsystemIO interface using a real PhotonVision camera */
public class HeimdallSubsystemIOReal implements HeimdallSubsystemIO {

  /** Camera and estimator for the front-looking camera */
  protected final CameraEstimator m_frontVision;

  /** Camera and estimator for the rear-looking camera */
  protected final CameraEstimator m_rearVision;

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Creates an instance of the I/O
   *
   * @param frontCamera Camera on the left side of the robot
   * @param rearCamera Camera on the left side of the robot
   */
  public HeimdallSubsystemIOReal(PhotonCamera frontCamera, PhotonCamera rearCamera) {

    // Create a pose estimator for the left camera
    PhotonPoseEstimator frontEstimator =
        new PhotonPoseEstimator(
            HeimdallSubsystem.kTagLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            frontCamera,
            HeimdallSubsystem.kFrontCameraLocationTransform);

    // Create a pose estimator for the right camera
    PhotonPoseEstimator rearEstimator =
        new PhotonPoseEstimator(
            HeimdallSubsystem.kTagLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            rearCamera,
            HeimdallSubsystem.kRearCameraLocationTransform);

    m_frontVision = new CameraEstimator(frontCamera, frontEstimator);
    m_rearVision = new CameraEstimator(rearCamera, rearEstimator);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /** Updates subsystem input and output measurements */
  @Override
  public void update(HeimdallSubsystemInputs inputs, HeimdallSubsystemOutputs outputs) {
    // Update front camera estimate, inputs, and outputs
    m_frontVision.update(inputs.frontCam, outputs.frontCam);

    // Update rear camera estimate, inputs, and outputs
    m_rearVision.update(inputs.rearCam, outputs.rearCam);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Resets the estimated pose to a given value
   *
   * @param pose Pose to set to
   */
  public void setPose(Pose2d pose) {
    // Nothing to do here for real I/O.  The PhotonPoseEstimator doesn't support setting the pose
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /** Inner class used to bundle a camera and PhotonPoseEstimator to produce pose estimates */
  protected static class CameraEstimator {

    /** PhotonCamera used to provide vision data */
    public final PhotonCamera m_camera;

    /** Vision-based pose estimator */
    public final PhotonPoseEstimator m_estimator;

    /** Timestamp of the last camera pipeline result */
    private double m_lastTimestamp = 0.0;

    public CameraEstimator(PhotonCamera camera, PhotonPoseEstimator estimator) {
      this.m_camera = camera;
      this.m_estimator = estimator;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * Gets the latest estimated pose of the robot from vision data
     *
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
     *     used for estimation; else empty if a pose is not known or can't be determined
     * @remarks This method must only be called once per robot cycle.
     */
    private boolean update(PoseEstimateInputs inputs, PoseEstimateOutputs outputs) {
      Optional<EstimatedRobotPose> estimatedPose = m_estimator.update();

      // If the timestamp from the camera has changed, this is a new result
      PhotonPipelineResult pipelineResult = m_camera.getLatestResult();
      double latestTimestamp = pipelineResult.getTimestampSeconds();
      boolean isNewEstimate =
          estimatedPose.isPresent() && Math.abs(latestTimestamp - m_lastTimestamp) > 1e-5;

      inputs.isFresh = isNewEstimate;
      outputs.noEstimate = estimatedPose.isEmpty();

      // If a new pose was calculated, update outputs and timestamp
      if (inputs.isFresh) {
        inputs.pipelineResult = pipelineResult;
        inputs.timestamp = latestTimestamp;
        outputs.pose = estimatedPose.get().estimatedPose;
        outputs.tagIDs = getFiducialIDs(pipelineResult);
        m_lastTimestamp = latestTimestamp;
      }

      return isNewEstimate;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * Utility function that returns an array containing the fiducial ID's from a
     * PhotonPipelineResult
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
}
