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

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

/** Implementation of the HeimdallSubsystemIO interface using a real PhotonVision camera */
public class HeimdallSubsystemIOReal implements HeimdallSubsystemIO {

  /** The camera used by vision I/O routines */
  protected final PhotonCamera m_camera;

  /** Vision-based pose estimator */
  private PhotonPoseEstimator m_photonEstimator;

  private double lastEstTimestamp = 0;

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Creates an instance of the I/O
   *
   * @param camera Camera used by the vision subsystem I/O
   */
  public HeimdallSubsystemIOReal(PhotonCamera camera) {
    m_camera = camera;
    m_photonEstimator =
        new PhotonPoseEstimator(
            HeimdallSubsystem.kTagLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            camera,
            HeimdallSubsystem.kRobotToCameraTransform);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /** Updates subsystem input measurements */
  public void updateInputs(HeimdallSubsystemInputs inputs) {
    // TODO: populate inputs
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /** Returns the latest PhotonPipelineResult from the camera */
  public PhotonPipelineResult getLatestResult() {
    return m_camera.getLatestResult();
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Gets the latest estimated pose of the robot from vision data
   *
   * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
   *     used for estimation; else empty if a pose is not known or can't be determined
   * @remarks This method must only be called once per loop.
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    Optional<EstimatedRobotPose> estimatedPose = m_photonEstimator.update();
    double latestTimestamp = m_camera.getLatestResult().getTimestampSeconds();

    boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;

    estimatedPose.ifPresentOrElse(
        est -> onUpdatedPoseEstimate(est),
        () -> {
          if (newResult) {
            onMissingUpdatedPoseEstimate();
          }
        });

    // If a result was found, record its timestamp
    if (newResult) {
      lastEstTimestamp = latestTimestamp;
    }

    return estimatedPose;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /** Called when an updated pose estimate is calculated */
  protected void onUpdatedPoseEstimate(EstimatedRobotPose updatedEstimate) {}

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /** Called when an updated pose estimate is missing */
  protected void onMissingUpdatedPoseEstimate() {}

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Returns the standard deviations of the estimated pose from {@link #getEstimatedGlobalPose()}
   * for use with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
   * SwerveDrivePoseEstimator}
   *
   * @param estimatedPose The estimated pose to guess standard deviations for.
   * @remarks This method should only be used when targets are visible.
   */
  public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
    var estStdDevs = HeimdallSubsystem.kSingleTagStdDevs;
    var targets = getLatestResult().getTargets();
    int numTags = 0;
    double avgDist = 0;
    for (var tgt : targets) {
      var tagPose = m_photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
      if (tagPose.isEmpty()) continue;
      numTags++;
      avgDist +=
          tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
    }
    if (numTags == 0) return estStdDevs;
    avgDist /= numTags;

    // Decrease std devs if multiple targets are visible
    if (numTags > 1) estStdDevs = HeimdallSubsystem.kMultiTagStdDevs;
    // Increase std devs based on (average) distance
    if (numTags == 1 && avgDist > 4)
      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

    return estStdDevs;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Resets the estimated pose to a given value
   *
   * @param pose Pose to set to
   */
  public void setPose(Pose2d pose) {
    // TODO: do we want to allow the pose to be reset when running for realz?
  }
}
