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
package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.joystick.ProcessedXboxController;
import frc.robot.Constants.CANDevice;
import frc.robot.Constants.RobotCANBus;
import org.littletonrobotics.junction.Logger;

// Reference Phoenix6 example:

/**
 * Subsystem for controlling the extension/retraction of climber mechanisms
 *
 * @remarks Motor config is based on the CTRE Phoenix 5 library PositionClosedLoop example
 *     https://github.com/CrossTheRoadElec/Phoenix5-Examples/blob/master/Java%20General/PositionClosedLoop/src/main/java/frc/robot/Robot.java
 */
public class ClimberSubsystem extends SubsystemBase {

  ////////////////////////////////////
  // CONSTANTS
  ////////////////////////////////////

  /** CAN bus used to communicate with the subsystem */
  public static final RobotCANBus kCANBus = RobotCANBus.Rio;

  ////////////////////////////////////
  // Climber Motor Configuration
  ////////////////////////////////////

  /** CAN device ID of the climber leader motor */
  public static final CANDevice kLeaderMotorDevice = CANDevice.ClimberLeaderMotor;

  /** CAN device ID of the climber follower motor */
  public static final CANDevice kFollowerMotorDevice = CANDevice.ClimberFollowerMotor;

  /** Gear ratio between climber motors and the climber mechanism */
  public static final double kClimberMotorGearRatio = 100.0 / 1.0;

  /** Set to true to reverse the direction of climber motors */
  public static final boolean kInvertMotors = false;

  /** Max number of rotations to achieve full extension */
  public static final double kMaxRotations = 6.3;

  /** Maximum motor output allowed (0.0 to 1.0) */
  public static final double kMaxMotorOutputPercent = 1.0;

  ////////////////////////////////////
  // Attributes
  ////////////////////////////////////

  /** I/O implementation used by the subsystem */
  private final ClimberSubsystemIO m_io;

  /** Measured input values for the subsystem */
  private final ClimberSubsystemInputs m_inputs = new ClimberSubsystemInputs();

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /** Creates a new climber subsystem */
  public ClimberSubsystem(ClimberSubsystemIO io) {
    m_io = io;
    m_io.initialize();
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Sets the desired climber position as a normalized percentage of maximum extension
   *
   * @param degrees Normalized percentage of full climber extension (0.0 to 1.0)
   */
  public void setExtensionPercent(double percent) {
    m_inputs.leader.targetPosition = percent;
    m_inputs.follower.targetPosition = percent;
    m_io.setExtensionPercent(percent);
  }

  public enum ClimberMotorID {
    Leader,
    Follower
  };

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Runs the climber motors directly
   *
   * @param percent Percent of motor output (-1.0 to 1.0)
   */
  public void setMotorOutputPercent(double percent) {
    m_io.setMotorOutputPercent(percent);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Returns the current extension of a climber motor as a normalized percentage of maximum
   *
   * @motorID Climber motor whose extension is to be returned
   * @return normalized percentage of maximum climber extension (0.0 to 1.0)
   */
  public double getExtensionPercent(ClimberMotorID motorID) {
    return m_io.getExtensionPercent(motorID);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Returns the current extension of the climber as a normalized percentage of maximum
   *
   * @return normalized percentage of maximum climber extension (0.0 to 1.0)
   */
  public double getExtensionPercent() {
    return m_io.getExtensionPercent(ClimberMotorID.Leader);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  @Override
  public void periodic() {
    // Update measurements
    m_io.processInputs(m_inputs);

    // Send input data to the logging framework (or update from the log during replay)
    Logger.processInputs("Climber", m_inputs);

    // Display velocities on dashboard
    SmartDashboard.putNumber("climber/targetPosition", m_inputs.leader.targetPosition);
    SmartDashboard.putNumber("climber/position", m_inputs.leader.position);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  public static class ClimberJoystickTeleOp extends Command {
    private final ClimberSubsystem m_climberSubsystem;
    private final ProcessedXboxController m_controller;

    /** Creates a new ClimberJoystickTeleOp. */
    public ClimberJoystickTeleOp(
        ClimberSubsystem climberSubsystem, ProcessedXboxController controller) {
      m_climberSubsystem = climberSubsystem;
      m_controller = controller;
      addRequirements(m_climberSubsystem);
      // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      double percent = m_controller.getLeftY() * -1.0;
      m_climberSubsystem.setMotorOutputPercent(percent);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
  }
}
