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
package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CameraID;
import frc.robot.subsystems.vision.CameraConstants.FrontCamera;
import frc.robot.subsystems.vision.CameraConstants.RearCamera;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

/** The HeimdallSubsystem provides a computer vision subsystem using the PhotonVision library */
public class HeimdallSubsystem extends SubsystemBase {

  ////////////////////////////////////
  // CONSTANTS
  ////////////////////////////////////

  /** Transformation used to convey the physical location of the front camera on the robot */
  public static final Transform3d kFrontCameraLocationTransform =
      new Transform3d(
          new Translation3d(
              FrontCamera.Location.xOffset,
              FrontCamera.Location.yOffset,
              FrontCamera.Location.zOffset),
          new Rotation3d(0, FrontCamera.Location.pitch, FrontCamera.Location.yaw));

  /** Transformation used to convey the physical location of the front camera on the robot */
  public static final Transform3d kRearCameraLocationTransform =
      new Transform3d(
          new Translation3d(
              RearCamera.Location.xOffset,
              RearCamera.Location.yOffset,
              RearCamera.Location.zOffset),
          new Rotation3d(0, RearCamera.Location.pitch, RearCamera.Location.yaw));

  // The layout of the AprilTags on the field
  public static final AprilTagFieldLayout kTagLayout =
      AprilTagFields.kDefaultField.loadAprilTagLayoutField();

  /** I/O implementation used for the front camera */
  private final HeimdallSubsystemCameraIO m_frontCameraIO;
  /** I/O implementation used for the rear camera */
  private final HeimdallSubsystemCameraIO m_rearCameraIO;

  /** Inputs for the front camera */
  private final HeimdallCameraInputs m_frontCamInputs = new HeimdallCameraInputs();
  /** Inputs for the rear camera */
  private final HeimdallCameraInputs m_rearCamInputs = new HeimdallCameraInputs();

  /** Logged outputs for the front camera estimator */
  private final HeimdallEstimatorOutputs m_frontEstimatorOutputs =
      new HeimdallEstimatorOutputs("Heimdall/frontCam");
  /** Logged outputs for the rear camera estimator */
  private final HeimdallEstimatorOutputs m_rearEstimatorOutputs =
      new HeimdallEstimatorOutputs("Heimdall/rearCam");

  /** Camera and estimator for the front-looking camera */
  protected final CameraEstimator m_frontVision;

  /** Camera and estimator for the rear-looking camera */
  protected final CameraEstimator m_rearVision;

  /** Routine called when a new robot pose is evaluated */
  private final Consumer<VisionPoseEstimate> m_poseConsumer;

  /** Supplier used to obtain the current robot pose in simulation mode */
  private final Supplier<Pose2d> m_simPoseSupplier;

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Creates an instance of the subsystem using specified I/O implementations
   *
   * @param frontCameraIO I/O implementation used for the front camera
   * @param rearCameraIO I/O implementation used for the rear camera
   * @param poseConsumer Consumer to receive estimated pose updates from the vision system
   * @param simPoseSupplier Pose supplier used in simulation mode only
   * @param io I/O implementation to be used by the subsystem
   */
  public HeimdallSubsystem(
      HeimdallSubsystemCameraIO frontCameraIO,
      HeimdallSubsystemCameraIO rearCameraIO,
      Consumer<VisionPoseEstimate> poseConsumer,
      Supplier<Pose2d> simPoseSupplier) {
    m_frontCameraIO = frontCameraIO;
    m_rearCameraIO = rearCameraIO;
    m_poseConsumer = poseConsumer;
    m_simPoseSupplier = simPoseSupplier;

    // Create a pose estimator for the front camera
    PhotonPoseEstimator frontEstimator =
        new PhotonPoseEstimator(
            HeimdallSubsystem.kTagLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            HeimdallSubsystem.kFrontCameraLocationTransform);
    frontEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    // Create a pose estimator for the rear camera
    PhotonPoseEstimator rearEstimator =
        new PhotonPoseEstimator(
            HeimdallSubsystem.kTagLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            HeimdallSubsystem.kRearCameraLocationTransform);
    rearEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    m_frontVision = new CameraEstimator(frontEstimator, kTagLayout);
    m_rearVision = new CameraEstimator(rearEstimator, kTagLayout);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /** Returns the most recent estimated pose estimate associated with a given camera */
  public Pose2d getEstimatedPose(CameraID camID) {
    Pose3d pose3d =
        (camID == CameraID.FrontCamera)
            ? m_frontEstimatorOutputs.pose
            : m_rearEstimatorOutputs.pose;
    return pose3d.toPose2d();
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /** Called periodically when the robot runs */
  @Override
  public void periodic() {
    // Update inputs using the I/O layer
    m_frontCameraIO.updateInputs(m_frontCamInputs);
    m_rearCameraIO.updateInputs(m_rearCamInputs);

    // Log/playback inputs
    Logger.processInputs("Heimdall/frontCam", m_frontCamInputs);
    Logger.processInputs("Heimdall/rearCam", m_rearCamInputs);

    // Notify our pose consumer of a new pose estimate from the front camera
    if (m_frontCamInputs.isFresh) {
      // Process inputs and generate a new pose estimate from the front camera
      m_frontVision.process(m_frontCamInputs, m_frontEstimatorOutputs);
      // Pass the new pose estimate to the registered consumer
      m_poseConsumer.accept(
          new VisionPoseEstimate(
              m_frontEstimatorOutputs.pose.toPose2d(),
              m_frontEstimatorOutputs.stdDevs,
              m_frontCamInputs.timestamp));
    }

    // Notify our pose consumer of a new pose estimate from the rear camera
    if (m_rearCamInputs.isFresh) {
      // Process inputs and generate a new pose estimate from the rear camera
      m_rearVision.process(m_rearCamInputs, m_rearEstimatorOutputs);
      // Pass the new pose estimate to the registered consumer
      m_poseConsumer.accept(
          new VisionPoseEstimate(
              m_rearEstimatorOutputs.pose.toPose2d(),
              m_rearEstimatorOutputs.stdDevs,
              m_rearCamInputs.timestamp));
    }

    // Log outputs
    m_frontEstimatorOutputs.toLog();
    m_rearEstimatorOutputs.toLog();
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /** Called periodically when the robot runs in simulation mode */
  @Override
  public void simulationPeriodic() {
    if (m_frontCamInputs.isFresh) {
      m_frontCameraIO.updateSimPose(
          m_frontEstimatorOutputs.noEstimate ? null : m_frontEstimatorOutputs.pose.toPose2d());
    }

    if (m_rearCamInputs.isFresh) {
      m_rearCameraIO.updateSimPose(
          m_rearEstimatorOutputs.noEstimate ? null : m_rearEstimatorOutputs.pose.toPose2d());
    }
  }

  /** Class used to communicate a vision-based pose estimate */
  public static class VisionPoseEstimate {
    public final Pose2d pose;
    public final Matrix<N3, N1> stddevs;
    public final double timestamp;

    public VisionPoseEstimate(Pose2d pose, Matrix<N3, N1> stddevs, double timestamp) {
      this.pose = pose;
      this.stddevs = stddevs;
      this.timestamp = timestamp;
    }
  }
}
