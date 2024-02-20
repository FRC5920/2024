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
package frc.robot.subsystems.intake;

import frc.robot.Constants.CANDevice;
import frc.robot.Constants.RobotCANBus;

/** I/O abstraction for the IntakeSubsystem */
public interface IntakeSubsystemIO {

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /** Initializes and configures the I/O implementation */
  default void initialize() {}

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /** Update logged input quantities */
  default void processInputs(IntakeSubsystemInputs inputs) {}

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Sets the desired speed of the indexer mechanism as a normalized percentage of full scale
   *
   * @param percent Normalized percentage of full speed (0.0 to 1.0)
   */
  default void setIndexerSpeed(double percent) {}

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Sets the desired velocity of the flywheel mechanism
   *
   * @param rotPerSec Desired velocity in rotations per second
   */
  default void setFlywheelVelocity(double rotPerSec) {}

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Returns the current speed of the indexer mechanism as a percentage of full speed
   *
   * @return Normalized percentage of full speed (0.0 to 1.0)
   */
  default double getIndexerSpeed() {
    return 0.0;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Returns the current velocity of the flywheel mechanism
   *
   * @return The velocity of the flywheel mechanism in rotations per second
   */
  default double getFlywheelVelocity() {
    return 0.0;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Returns the distance measured by the gamepiece sensor
   *
   * @return The distance measured by the gamepiece sensor in meters
   */
  default double getGamepieceDistance() {
    return 0.0;
  }

  /** Parameters used to configure the subsystem I/O */
  public static class Config {
    public final RobotCANBus canBus;
    public final CANDevice flywheelMotorDevice;
    public final double flywheelGearRatio;
    public final boolean invertFlywheelMotor;
    public final double maxFlywheelVelocity;

    public final CANDevice indexerMotorDevice;
    public final double indexerGearRatio;
    public final boolean invertIndexerMotor;
    public final double maxIndexerSpeed;

    public final CANDevice gamepieceSensorDevice;

    public Config() {
      this.canBus = IntakeSubsystem.kCANBus;
      this.flywheelMotorDevice = IntakeSubsystem.kFlywheelMotorCANDevice;
      this.flywheelGearRatio = IntakeSubsystem.kFlywheelMotorGearRatio;
      this.invertFlywheelMotor = IntakeSubsystem.kFlywheelMotorInverted;
      this.maxFlywheelVelocity = IntakeSubsystem.kMaxFlywheelMotorVelocity;
      this.indexerMotorDevice = IntakeSubsystem.kIndexerMotorCANDevice;
      this.indexerGearRatio = IntakeSubsystem.kIndexerMotorGearRatio;
      this.invertIndexerMotor = IntakeSubsystem.kIndexerMotorInverted;
      this.maxIndexerSpeed = IntakeSubsystem.kMaxIndexerSpeed;
      this.gamepieceSensorDevice = IntakeSubsystem.kGamepieceSensorCANDevice;
    }
  }
}
