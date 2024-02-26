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
public class HeimdallSubsystemInputs implements LoggableInputs {

  /** Pose estimate data from the front camera */
  public PoseEstimateInputs frontCam = new PoseEstimateInputs("frontCam");

  /** Pose estimate data from the rear camera */
  public PoseEstimateInputs rearCam = new PoseEstimateInputs("rearCam");

  /** Write input values to log */
  public void toLog(LogTable table) {
    frontCam.toLog(table);
    rearCam.toLog(table);
  }

  /** Read input values from log */
  public void fromLog(LogTable table) {
    frontCam.fromLog(table);
  }

  /** Create a clone of input values */
  public HeimdallSubsystemInputs clone() {
    HeimdallSubsystemInputs copy = new HeimdallSubsystemInputs();
    copy.frontCam = frontCam.clone();
    copy.rearCam = rearCam.clone();
    return copy;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /** An object containing a PhotonVision-based pose estimate and associated data */
  public static class PoseEstimateInputs {

    private final String m_keyIsFresh;
    private final String m_keyTimestamp;
    private final String m_keyPipelineResult;

    /** true if the estimate was created in the present robot cycle; else false if it is stale */
    public boolean isFresh = false;
    /** Timestamp of the estimate */
    public double timestamp = 0.0;
    /** PhotonPipelineResult used to create the pose estimate */
    PhotonPipelineResult pipelineResult = new PhotonPipelineResult();

    /**
     * Creates a new instance of the inputs with a given log prefix
     *
     * @param prefix Prefix to use when logging data fields
     */
    public PoseEstimateInputs(String prefix) {
      m_keyIsFresh = prefix + "/isFresh";
      m_keyTimestamp = prefix + "/timestamp";
      m_keyPipelineResult = prefix + "/pipelineResult";
    }

    /** Copy constructor */
    private PoseEstimateInputs(PoseEstimateInputs other) {
      m_keyIsFresh = other.m_keyIsFresh;
      m_keyTimestamp = other.m_keyTimestamp;
      m_keyPipelineResult = other.m_keyPipelineResult;
    }

    /** Write input values to log */
    public void toLog(LogTable table) {
      table.put(m_keyIsFresh, isFresh);

      // Only log remaining data if the estimate is fresh
      if (isFresh) {
        table.put(m_keyTimestamp, timestamp);
        table.put(m_keyPipelineResult, pipelineResult);
      }
    }

    /** Read input values from log */
    public void fromLog(LogTable table) {
      // Read the fresh flag
      isFresh = table.get(m_keyIsFresh, isFresh);

      // Only load remaining data if it is flagged as fresh
      if (isFresh) {
        timestamp = table.get(m_keyTimestamp, timestamp);
        pipelineResult = table.get(m_keyPipelineResult, pipelineResult);
      }
    }

    /** Create a clone of input values */
    public PoseEstimateInputs clone() {
      PoseEstimateInputs copy = new PoseEstimateInputs(this);
      copy.isFresh = this.isFresh;
      copy.timestamp = this.timestamp;
      copy.pipelineResult = this.pipelineResult;
      return copy;
    }
  }
}
