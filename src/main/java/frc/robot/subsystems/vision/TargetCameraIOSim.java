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
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants.CameraID;
import frc.robot.subsystems.vision.CameraConstants.TagCameraResolution;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

/** Target Camera I/O implemented using PhotonVision simulation */
public class TargetCameraIOSim implements TargetCameraIO {

  /** Enable a raw camera stream for the simulated cameras */
  private static final boolean kEnableRawSimCameraStream = true;

  /** Enable a processed camera stream for the simulated cameras */
  private static final boolean kEnableProcessedSimCameraStream = true;

  /** Draw a wire frame to the simulated camera stream (dramatically increases loop time) */
  private static final boolean kDrawWireFrameToVideoStream = true;

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

  /** Photon camera used for the simulation */
  private final PhotonCamera m_camera;

  /** Simulated PhotonVision system */
  private static VisionSystemSim m_visionSim;

  /** Front tag camera simulation */
  private final PhotonCameraSim m_cameraSim;

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Creates an instance of the I/O
   *
   * @param cameraID Camera to use
   * @param robot2CameraTransform Transformation used to describe the camera's location relative to
   *     the robot
   * @param visionSystemSim Simulated vision system to add the object's camera to
   */
  public TargetCameraIOSim(
      CameraID cameraID, Transform3d robot2CameraTransform, VisionSystemSim visionSystemSim) {
    m_camera = new PhotonCamera(cameraID.name);
    m_visionSim = visionSystemSim;

    // Create simulated camera properties. These can be set to mimic your actual camera.
    var camProps = new SimCameraProperties();
    camProps.setCalibration(
        TagCameraResolution.widthPx,
        TagCameraResolution.heightPx,
        Rotation2d.fromDegrees(TagCameraResolution.FOVDegrees));
    camProps.setCalibError(kAverageCalibrationErrorPx, kCalibrationErrorStdDev);
    camProps.setFPS(kAverageFramesPerSecond);
    camProps.setAvgLatencyMs(kLatencyAverageMs);
    camProps.setLatencyStdDevMs(kLatencyStdDevMs);

    // Set up a simulated front camera
    m_cameraSim = new PhotonCameraSim(m_camera, camProps);
    // Add the simulated camera to view the targets on this simulated field.
    m_visionSim.addCamera(m_cameraSim, robot2CameraTransform);

    m_cameraSim.enableRawStream(kEnableRawSimCameraStream);
    m_cameraSim.enableProcessedStream(kEnableProcessedSimCameraStream);
    m_cameraSim.enableDrawWireframe(kDrawWireFrameToVideoStream);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Updates camera input values
   *
   * @param inputs Inputs to update
   * @param io I/O implementation used to update the inputs
   */
  @Override
  public void updateInputs(TargetCameraInputs inputs) {

    inputs.cameraIsConnected = true;

    // Get the latest result from the tag camera
    PhotonPipelineResult pipelineResult = m_camera.getLatestResult();
    inputs.targetIsDetected = pipelineResult.hasTargets();
    inputs.pipelineResult = pipelineResult;
  }
}
