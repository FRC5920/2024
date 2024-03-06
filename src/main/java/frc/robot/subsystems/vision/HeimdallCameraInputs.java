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

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.targeting.PhotonPipelineResult;

/** Measured inputs of the Heimdall vision subsystem */
public class HeimdallCameraInputs implements LoggableInputs {
  private static final String kKeyCameraIsConnected = "/cameraIsConnected";
  private static final String kKeyIsFresh = "/isFresh";
  private static final String kKeyTimestamp = "/timestamp";
  private static final String kKeyPipelineResult = "/pipelineResult";

  /** true if the camera is connected; else false */
  public boolean cameraIsConnected = false;
  /** true if the estimate was created in the present robot cycle; else false if it is stale */
  public boolean isFresh = false;
  /** Timestamp of the estimate */
  public double timestamp = 0.0;
  /** PhotonPipelineResult used to create the pose estimate */
  PhotonPipelineResult pipelineResult = new PhotonPipelineResult();

  /** Creates an instance of the object */
  public HeimdallCameraInputs() {}

  /** Copy constructor */
  private HeimdallCameraInputs(HeimdallCameraInputs other) {
    cameraIsConnected = other.cameraIsConnected;
    isFresh = other.isFresh;
    timestamp = other.timestamp;
    pipelineResult = other.pipelineResult;
  }

  /** Write input values to log */
  public void toLog(LogTable table) {
    table.put(kKeyCameraIsConnected, cameraIsConnected);
    table.put(kKeyIsFresh, isFresh);

    // Only log remaining data if the estimate is fresh
    if (isFresh) {
      table.put(kKeyTimestamp, timestamp);
      table.put(kKeyPipelineResult, pipelineResult);
    }
  }

  /** Read input values from log */
  public void fromLog(LogTable table) {
    cameraIsConnected = table.get(kKeyCameraIsConnected, cameraIsConnected);
    isFresh = table.get(kKeyIsFresh, isFresh);

    // Only load remaining data if it is flagged as fresh
    if (isFresh) {
      timestamp = table.get(kKeyTimestamp, timestamp);
      pipelineResult = table.get(kKeyPipelineResult, pipelineResult);
    }
  }

  /** Create a clone of input values */
  public HeimdallCameraInputs clone() {
    return new HeimdallCameraInputs(this);
  }
}
