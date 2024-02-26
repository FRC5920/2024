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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants.CameraInfo.TagCameraResolution;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/** Add your docs here. */
public class HeimdallSubsystemIOSim extends HeimdallSubsystemIOReal {

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
  private VisionSystemSim m_visionSim;

  /** Simulated left camera */
  private PhotonCameraSim m_frontCameraSim;

  /** Simulated right camera */
  private PhotonCameraSim m_rearCameraSim;

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Creates an instance of the I/O
   *
   * @param camera Camera used by the vision subsystem I/O
   */
  public HeimdallSubsystemIOSim(PhotonCamera leftCamera, PhotonCamera rightCamera) {
    super(leftCamera, rightCamera);

    // Create the vision system simulation which handles cameras and targets on the field.
    m_visionSim = new VisionSystemSim("main");
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

    m_frontCameraSim.enableDrawWireframe(kDrawWireFrameToVideoStream);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /** Updates subsystem input and output measurements */
  @Override
  public void update(HeimdallSubsystemInputs inputs, HeimdallSubsystemOutputs outputs) {
    super.update(inputs, outputs);

    // Update the estimated poses on the dashboard field when new estimates are calculated
    Field2d debugField = m_visionSim.getDebugField();

    // If the front camera produced a new pose, display it on the field
    if (inputs.frontCam.isFresh) {
      debugField.getObject("FrontCamPose").setPose(outputs.frontCam.pose.toPose2d());
    }
    // Otherwise, if the estimator produced no output, remove the pose from the field
    if (outputs.frontCam.noEstimate) {
      debugField.getObject("FrontCamPose").setPoses();
    }

    // If the rear camera produced a new pose, display it on the field
    if (inputs.rearCam.isFresh) {
      debugField.getObject("RearCamPose").setPose(outputs.rearCam.pose.toPose2d());
    }
    // Otherwise, if the estimator produced no output, remove the pose from the field
    if (outputs.rearCam.noEstimate) {
      debugField.getObject("RearCamPose").setPoses();
    }
  }
}
