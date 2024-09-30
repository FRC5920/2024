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

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.utility.Alert;
import frc.robot.subsystems.vision.CameraConstants.TargetPipeline;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;

////////////////////////////////////////////////////////////////////////////////////////////////////
/** A subsystem for the camera used to target gamepieces */
public class TargetCameraSubsystem extends SubsystemBase {

  ////////////////////////////////////
  // CONSTANTS
  ////////////////////////////////////
  /** Transformation used to convey the physical location of the front camera on the robot */
  public static final Transform3d kCameraLocationTransform =
      new Transform3d(
          new Translation3d(0.0, 0.0, 0.5), new Rotation3d(0, Units.degreesToRadians(20), 0.0));

  /** I/O implementation used by the subsystem */
  private final TargetCameraIO m_io;

  private final TargetCameraInputsAutoLogged m_inputs = new TargetCameraInputsAutoLogged();

  /** Alert displayed on failure to configure pivot motors */
  private static final Alert m_cameraNotPresentAlert =
      new Alert("Target camera is not connected!", Alert.AlertType.ERROR);

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Creates a new instance of the subsystem
   *
   * @param io I/O implementation for the subsystem to use
   */
  public TargetCameraSubsystem(TargetCameraIO io) {
    m_io = io;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Sets the pipeline to be used for target detection
   *
   * @param pipeline Pipeline to be used by the camera
   */
  public void setPipeline(TargetPipeline pipeline) {
    m_io.setPipeline(pipeline);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /** Returns the pipeline used for target detection */
  public TargetPipeline getPipeline() {
    return m_io.getPipeline();
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /** Returns the current camera Pipeline result */
  public PhotonPipelineResult getLatestResult() {
    return m_inputs.pipelineResult;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  @Override
  public void periodic() {
    updateInputs();
    Logger.processInputs("TargetCamera", m_inputs);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /** Updates subsystem inputs */
  private void updateInputs() {

    m_inputs.cameraIsConnected = m_io.cameraIsConnected();
    // Alert if the camera is not connected
    m_cameraNotPresentAlert.set(!m_inputs.cameraIsConnected);

    // Get the latest result from the tag camera
    PhotonPipelineResult pipelineResult = m_io.getLatestResult();
    m_inputs.pipelineResult = pipelineResult;
    m_inputs.targetIsDetected = pipelineResult.hasTargets();
  }
}
