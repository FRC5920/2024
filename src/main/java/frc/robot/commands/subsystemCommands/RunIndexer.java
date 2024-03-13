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
package frc.robot.commands.subsystemCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.logging.BotLog.DebugPrintCommand;
import frc.lib.thirdparty.LoggedTunableNumber;
import frc.robot.subsystems.indexer.IndexerSubsystem;

//////////////////////////////////////////////////////////////////////////////////////////////////
/** A command to run the indexer at a specified speed */
public class RunIndexer extends Command {

  /** Speed presets for the indexer */
  public enum IndexerPreset {
    IntakeRing(-0.75),
    ShootRing(1.0),
    Stopped(0.0);

    /** Tunable value for the preset */
    private final LoggedTunableNumber tunableValue;

    /**
     * Creates the element
     *
     * @param defaultSpeed Default for the element's tunable value
     */
    private IndexerPreset(double defaultSpeed) {
      tunableValue = new LoggedTunableNumber("IndexerPreset/" + this.name(), defaultSpeed);
    }

    /** Returns the element's speed value */
    public double getSpeed() {
      return tunableValue.get();
    }
  }

  /** The indexer subsystem to operate on */
  private final IndexerSubsystem m_indexer;
  /** The indexer speed to apply */
  private final double m_targetSpeed;
  /** Maximum time to run the indexer (negative value indicates no timeout) */
  private final double m_timeoutSec;
  /** Timer used to implement timeout */
  private final Timer m_timer;

  /**
   * Creates a command that will run the indexer at a given speed perpetually
   *
   * @param indexer Indexer subsystem to operate on
   * @param preset Speed preset to run the indexer at
   */
  public RunIndexer(IndexerSubsystem indexer, IndexerPreset preset) {
    this(indexer, preset, -1.0);
  }

  /**
   * Creates a command that will run the indexer at a given speed preset for a maximum amount of
   * time
   *
   * @param indexer Indexer subsystem to operate on
   * @param preset Speed preset to run the indexer at
   * @param timeoutSec Maximum time in seconds to run the indexer motor
   */
  public RunIndexer(IndexerSubsystem indexer, IndexerPreset preset, double timeoutSec) {
    m_indexer = indexer;
    m_targetSpeed = preset.getSpeed();
    m_timeoutSec = timeoutSec;
    m_timer = new Timer();
    addRequirements(m_indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_indexer.setIndexerSpeed(m_targetSpeed);
    if (m_timeoutSec > 0.0) {
      m_timer.reset();
      m_timer.start();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_indexer.setIndexerSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_timeoutSec > 0.0) ? m_timer.hasElapsed(m_timeoutSec) : false;
  }

  public static Command stop(IndexerSubsystem indexer) {
    return new SequentialCommandGroup(
        new DebugPrintCommand("Stopping the indexer"),
        new InstantCommand(() -> indexer.setIndexerSpeed(0.0)));
  }
}
