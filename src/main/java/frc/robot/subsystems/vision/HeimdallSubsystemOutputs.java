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

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N3;
import org.littletonrobotics.junction.Logger;

/** Logged outputs produced by the Heimdall vision subsystem */
public class HeimdallSubsystemOutputs {

  public final PoseEstimateOutputs frontCam;
  public final PoseEstimateOutputs rearCam;

  /**
   * Creates an instance of the outputs
   *
   * @param prefix Prefix to apply to logged data
   */
  public HeimdallSubsystemOutputs(String prefix) {
    frontCam = new PoseEstimateOutputs(prefix + "frontCam");
    rearCam = new PoseEstimateOutputs(prefix + "rearCam");
  }

  /** Write output values to log */
  public void toLog() {
    frontCam.toLog();
    rearCam.toLog();
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /** An object containing a PhotonVision-based pose estimate and associated output data */
  public static class PoseEstimateOutputs {

    private final String m_keyNoEstimate;
    private final String m_keyPose;
    private final String m_keyStdDevs;
    private final String m_keyTagIDs;

    /** true when an empty estimate was produced by the pose estimator; else false */
    public boolean noEstimate = false;
    /** Estimated robot pose produced from vision data */
    public Pose3d pose = new Pose3d();
    /** Standard deviations associated with the pose estimate */
    public Vector<N3> stdDevs = PoseEstimateProcessor.kUntrustedPoseStdDevs;
    /** Sorted array of tag ID's indicating the tags used to calculate the estimate */
    public int[] tagIDs = new int[] {};

    /**
     * Creates a new instance of the inputs with a given log prefix
     *
     * @param prefix Prefix to use when logging data fields
     */
    public PoseEstimateOutputs(String prefix) {
      m_keyNoEstimate = prefix + "noEstimate";
      m_keyPose = prefix + "pose";
      m_keyStdDevs = prefix + "stdDevs";
      m_keyTagIDs = prefix + "tagIDs";
    }

    /** Writes outputs to the log */
    public void toLog() {
      Logger.recordOutput(m_keyNoEstimate, noEstimate);
      Logger.recordOutput(m_keyPose, pose);
      Logger.recordOutput(m_keyStdDevs, stdDevs.getData());
      Logger.recordOutput(m_keyTagIDs, tagIDs);
    }
  }
}
