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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.subsystems.vision.CameraConstants.TagCameraResolution;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/** Add your docs here. */
public class HeimdallSubsystemIOSim extends HeimdallSubsystemIOReal {

  /** Enable a raw camera stream for the simulated cameras */
  private static final boolean kEnableRawSimCameraStream = true;

  /** Enable a processed camera stream for the simulated cameras */
  private static final boolean kEnableProcessedSimCameraStream = true;

  /** Draw a wire frame to the simulated camera stream (dramatically increases loop time) */
  private static final boolean kDrawWireFrameToVideoStream = false;

  /** Average camera latency to simulate */
  private static final double kLatencyAverageMs = 31.5;

  /** Average camera latency to simulate */
  private static final double kLatencyStdDevMs = 13.5;

  /** Average frame rate to simulate */
  private static final double kAverageFramesPerSecond = 25.0;

  /** Average calibration error (pixels) */
  private static final double kAverageCalibrationErrorPx = 0.35;

  /** Calibration error standard deviation */
  private static final double kCalibrationErrorStdDev = 0.1;

  /** Simulated PhotonVision system */
  private final VisionSystemSim m_visionSim;

  /** Front tag camera simulation */
  private final PhotonCameraSim m_frontCameraSim;

  /** Rear tag camera simulation */
  private final PhotonCameraSim m_rearCameraSim;

  /** Supplier from which the simulated robot pose is obtained */
  private final Supplier<Pose2d> m_poseSupplier;

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Creates an instance of the I/O
   *
   * @param frontCamera Camera pointing toward the front side of the robot
   * @param rearCamera Camera pointing toward the rear side of the robot
   * @param poseSupplier Routine that will be called to obtain the present robot pose for updating
   *     the vision simulation
   */
  public HeimdallSubsystemIOSim(
      PhotonCamera leftCamera, PhotonCamera rightCamera, Supplier<Pose2d> poseSupplier) {
    super(leftCamera, rightCamera);

    m_poseSupplier = poseSupplier;

    // Create the vision system simulation which handles cameras and targets on the field.
    m_visionSim = new VisionSystemSim("TagCameras");
    // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
    m_visionSim.addAprilTags(HeimdallSubsystem.kTagLayout);

    // Create simulated camera properties. These can be set to mimic your actual camera.
    var camProps = new SimCameraProperties();
    camProps.setCalibration(
        TagCameraResolution.widthPx, TagCameraResolution.heightPx, Rotation2d.fromDegrees(0));
    camProps.setCalibError(kAverageCalibrationErrorPx, kCalibrationErrorStdDev);
    camProps.setFPS(kAverageFramesPerSecond);
    camProps.setAvgLatencyMs(kLatencyAverageMs);
    camProps.setLatencyStdDevMs(kLatencyStdDevMs);

    // Set up a simulated front camera
    m_frontCameraSim = new PhotonCameraSim(m_frontVision.m_camera, camProps);
    // Add the simulated camera to view the targets on this simulated field.
    m_visionSim.addCamera(m_frontCameraSim, HeimdallSubsystem.kFrontCameraLocationTransform);

    // Set up a simulated rear camera
    m_rearCameraSim = new PhotonCameraSim(m_rearVision.m_camera, camProps);
    // Add the simulated camera to view the targets on this simulated field.
    m_visionSim.addCamera(m_rearCameraSim, HeimdallSubsystem.kRearCameraLocationTransform);

    m_frontCameraSim.enableRawStream(kEnableRawSimCameraStream);
    m_frontCameraSim.enableProcessedStream(kEnableProcessedSimCameraStream);
    m_frontCameraSim.enableDrawWireframe(kDrawWireFrameToVideoStream);

    m_rearCameraSim.enableRawStream(kEnableRawSimCameraStream);
    m_rearCameraSim.enableProcessedStream(kEnableProcessedSimCameraStream);
    m_rearCameraSim.enableDrawWireframe(kDrawWireFrameToVideoStream);
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

    // Update the vision simulation with the current simulated pose
    m_visionSim.update(m_poseSupplier.get());

    super.update(inputs, outputs);

    // Update the estimated poses on the dashboard field when new estimates are calculated
    Field2d debugField = m_visionSim.getDebugField();

    // If the front camera produced a new pose, display it on the field
    if (inputs.frontCam.isFresh) {
      debugField.getObject("FrontCam").setPose(outputs.frontCam.pose.toPose2d());
    }
    // Otherwise, if the estimator produced no output, remove the pose from the field
    if (outputs.frontCam.noEstimate) {
      debugField.getObject("FrontCam").setPoses();
    }

    // If the rear camera produced a new pose, display it on the field
    if (inputs.rearCam.isFresh) {
      debugField.getObject("RearCam").setPose(outputs.rearCam.pose.toPose2d());
    }
    // Otherwise, if the estimator produced no output, remove the pose from the field
    if (outputs.rearCam.noEstimate) {
      debugField.getObject("RearCam").setPoses();
    }
  }
}
