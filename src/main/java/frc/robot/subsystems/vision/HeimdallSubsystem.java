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
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CameraInfo.FrontCameraLocation;
import frc.robot.Constants.CameraInfo.RearCameraLocation;
import frc.robot.subsystems.vision.HeimdallSubsystemOutputs.PoseEstimateOutputs;
import java.util.function.BiConsumer;
import org.littletonrobotics.junction.Logger;

/** The HeimdallSubsystem provides a computer vision subsystem using the PhotonVision library */
public class HeimdallSubsystem extends SubsystemBase {

  ////////////////////////////////////
  // CONSTANTS
  ////////////////////////////////////

  /** Transformation used to convey the physical location of the front camera on the robot */
  public static final Transform3d kFrontCameraLocationTransform =
      new Transform3d(
          new Translation3d(
              FrontCameraLocation.xOffset,
              FrontCameraLocation.yOffset,
              FrontCameraLocation.zOffset),
          new Rotation3d(0, 0, FrontCameraLocation.yaw));

  /** Transformation used to convey the physical location of the front camera on the robot */
  public static final Transform3d kRearCameraLocationTransform =
      new Transform3d(
          new Translation3d(
              RearCameraLocation.xOffset, RearCameraLocation.yOffset, RearCameraLocation.zOffset),
          new Rotation3d(0, 0, RearCameraLocation.yaw));

  // The layout of the AprilTags on the field
  public static final AprilTagFieldLayout kTagLayout =
      AprilTagFields.kDefaultField.loadAprilTagLayoutField();

  /** I/O implementation used by the subsystem */
  private final HeimdallSubsystemIO m_io;

  /** Inputs updated by the subsystem */
  private final HeimdallSubsystemInputs m_inputs = new HeimdallSubsystemInputs();

  private final HeimdallSubsystemOutputs m_outputs = new HeimdallSubsystemOutputs("Heimdall");

  /** Processor applied to front camera pose estimates */
  private final PoseEstimateProcessor m_frontPoseProcessor;

  /** Processor applied to rear camera pose estimates */
  private final PoseEstimateProcessor m_rearPoseProcessor;

  /** Consumer notified when a new estimate is produced by the front camera */
  private BiConsumer<Pose2d, Vector<N3>> m_frontPoseConsumer = null;

  /** Consumer notified when a new estimate is produced by the rear camera */
  private BiConsumer<Pose2d, Vector<N3>> m_rearPoseConsumer = null;

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Creates an instance of the subsystem using specified I/O implementations
   *
   * @param io I/O implementation to be used by the subsystem
   */
  public HeimdallSubsystem(
      HeimdallSubsystemIO io,
      PoseEstimateProcessor frontPoseProcessor,
      PoseEstimateProcessor rearPoseProcessor) {
    m_io = io;
    m_frontPoseProcessor = frontPoseProcessor;
    m_rearPoseProcessor = rearPoseProcessor;
  }

  /** ID's used to specify a camera/estimator in the subsystem */
  public enum CameraID {
    FrontCamera,
    RearCamera;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Assigns a routine to be called when a new robot pose is evaluated
   *
   * @param camID ID of the camera to assign a consumer for
   * @param poseConsumer Consumer to be notified of a new estimated pose for the camera with the
   *     pose estimate and associated standard deviations
   */
  void setPoseConsumer(CameraID camID, BiConsumer<Pose2d, Vector<N3>> poseConsumer) {
    switch (camID) {
      case FrontCamera:
        m_frontPoseConsumer = poseConsumer;
        break;
      case RearCamera:
        m_rearPoseConsumer = poseConsumer;
        break;
    }
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /** Returns the most recent estimated pose estimate associated with a given camera */
  public PoseEstimateOutputs getEstimatedPose(CameraID camID) {
    return (camID == CameraID.FrontCamera) ? m_outputs.frontCam : m_outputs.rearCam;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Resets the current pose of the vision estimators to a specified pose.
   *
   * @remarks This should ONLY be called when the robot's position on the field is known, (e.g. the
   *     beginning of a match).
   * @param newPose Pose to set the robot to
   */
  public void setPose(Pose2d newPose) {
    m_io.setPose(newPose);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  @Override
  public void periodic() {
    m_io.update(m_inputs, m_outputs);

    // Handle a new pose estimate from the front camera
    if (m_inputs.frontCam.isFresh) {
      // Calculate standard deviations using the front estimate processor
      m_outputs.frontCam.stdDevs =
          m_frontPoseProcessor.processPoseEstimate(
              m_outputs.frontCam.pose.toPose2d(), m_outputs.frontCam.tagIDs);
      // Notify the front pose consumer
      m_frontPoseConsumer.accept(m_outputs.frontCam.pose.toPose2d(), m_outputs.frontCam.stdDevs);
    }

    // Handle a new pose estimate from the rear camera
    if (m_inputs.rearCam.isFresh) {
      // Calculate standard deviations using the rear estimate processor
      m_outputs.rearCam.stdDevs =
          m_rearPoseProcessor.processPoseEstimate(
              m_outputs.rearCam.pose.toPose2d(), m_outputs.rearCam.tagIDs);
      // Notify the rear pose consumer
      m_rearPoseConsumer.accept(m_outputs.rearCam.pose.toPose2d(), m_outputs.rearCam.stdDevs);
    }

    // Log inputs
    Logger.processInputs("Heimdall", m_inputs);

    // Log outputs
    m_outputs.toLog();
  }
}
