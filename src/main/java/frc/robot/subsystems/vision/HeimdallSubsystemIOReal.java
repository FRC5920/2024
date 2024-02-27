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

/** HeimdallSubsystem I/O implementation using real PhotonVision cameras */
public class HeimdallSubsystemIOReal implements HeimdallSubsystemIO {

  /** Camera and estimator for the front-looking camera */
  protected final CameraEstimator m_frontVision;

  /** Camera and estimator for the rear-looking camera */
  protected final CameraEstimator m_rearVision;

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Creates an instance of the I/O
   *
   * @param frontCamera Camera pointing toward the front side of the robot
   * @param rearCamera Camera pointing toward the rear side of the robot
   */
  public HeimdallSubsystemIOReal(PhotonCamera frontCamera, PhotonCamera rearCamera) {

    // Create a pose estimator for the front camera
    PhotonPoseEstimator frontEstimator =
        new PhotonPoseEstimator(
            HeimdallSubsystem.kTagLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            frontCamera,
            HeimdallSubsystem.kFrontCameraLocationTransform);
    frontEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    // Create a pose estimator for the rear camera
    PhotonPoseEstimator rearEstimator =
        new PhotonPoseEstimator(
            HeimdallSubsystem.kTagLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            rearCamera,
            HeimdallSubsystem.kRearCameraLocationTransform);
    rearEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    m_frontVision = new CameraEstimator(frontCamera, frontEstimator);
    m_rearVision = new CameraEstimator(rearCamera, rearEstimator);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Updates subsystem input and output measurements
   *
   * @param inputs Inputs to update
   * @param outputs Outputs to update
   */
  @Override
  public void update(HeimdallSubsystemInputs inputs, HeimdallSubsystemOutputs outputs) {
    // Update front camera estimate, inputs, and outputs
    m_frontVision.processInputs(inputs.frontCam);
    m_frontVision.calculatePose(inputs.frontCam, outputs.frontCam);

    // Update rear camera estimate, inputs, and outputs
    m_rearVision.processInputs(inputs.rearCam);
    m_rearVision.calculatePose(inputs.rearCam, outputs.rearCam);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Resets the estimated pose to a given value
   *
   * @param pose Pose to set to
   */
  @Override
  public void setPose(Pose2d pose) {
    // Nothing to do here for real I/O.  The PhotonPoseEstimator doesn't support setting the pose
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Inner class used to bundle a camera and PhotonPoseEstimator together for producing pose
   * estimates
   */
  protected static class CameraEstimator {

    /** PhotonCamera used to provide vision data */
    public final PhotonCamera m_camera;

    /** Vision-based pose estimator */
    public final PhotonPoseEstimator m_estimator;

    /** Timestamp of the last pipeline result result received from m_camera */
    private double m_lastTimestamp = 0.0;

    /**
     * Creates an instance of the object
     *
     * @param camera PhotonCamera to use
     * @param estimator PhotonPoseEstimator to use
     */
    public CameraEstimator(PhotonCamera camera, PhotonPoseEstimator estimator) {
      this.m_camera = camera;
      this.m_estimator = estimator;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    private void processInputs(PoseEstimateInputs inputs) {
      PhotonPipelineResult pipelineResult = m_camera.getLatestResult();
      double latestTimestamp = pipelineResult.getTimestampSeconds();
      inputs.isFresh = Math.abs(latestTimestamp - m_lastTimestamp) > 1e-5;

      // Only log pipeline result if a new PipelineResult was received
      if (inputs.isFresh) {
        inputs.pipelineResult = pipelineResult;
        inputs.timestamp = latestTimestamp;
        m_lastTimestamp = latestTimestamp; // Rotate in the latest timestamp
      }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * If a new pipeline result is available from the camera, this routine calculates a new
     * estimated pose, updating inputs and outputs.
     *
     * @param inputs Inputs to update
     * @param outputs Outputs to update
     * @remarks This function always updates inputs.isFresh to reflect whether a new estimate was
     *     calculated.
     */
    private void calculatePose(PoseEstimateInputs inputs, PoseEstimateOutputs outputs) {
      // If a new PipelineResult hasn't been received, there's nothing to do
      if (!inputs.isFresh) {
        return;
      }

      // Calculate a new estimated pose
      Optional<EstimatedRobotPose> estimatedPose = m_estimator.update(inputs.pipelineResult);
      outputs.noEstimate = estimatedPose.isEmpty();

      // If a new pose estimate was calculated, update outputs
      if (!outputs.noEstimate) {
        outputs.pose = estimatedPose.get().estimatedPose;
        outputs.tagIDs = getFiducialIDs(inputs.pipelineResult);
      }
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
