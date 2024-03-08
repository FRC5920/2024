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
package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.logging.BotLog;
import frc.lib.logging.BotLog.DebugPrintCommand;
import frc.lib.logging.BotLog.InfoPrintCommand;
import frc.robot.Constants.ScoringTarget;
import frc.robot.commands.ArmCommands.PivotCommand.AnglePreset;
import frc.robot.commands.subsystemCommands.RunFlywheel;
import frc.robot.commands.subsystemCommands.RunFlywheel.FlywheelPreset;
import frc.robot.commands.subsystemCommands.RunIndexer;
import frc.robot.commands.subsystemCommands.RunIndexer.IndexerPreset;
import frc.robot.subsystems.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.pivot.PivotSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootNote extends SequentialCommandGroup {
  /** Maximum time to run the indexer */
  private static final double kIndexerTimeoutSec = 2.0;

  /** Creates a new ShootNote. */
  public ShootNote(
      ScoringTarget target,
      PivotSubsystem pivot,
      FlywheelSubsystem flywheel,
      IndexerSubsystem indexer) {
    switch (target) {
      case Amp:
        addCommands(
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new InfoPrintCommand("ShootAtAmp"),
                    new PivotCommand(pivot, AnglePreset.ShootAmp)),
                new SequentialCommandGroup(
                    new DebugPrintCommand("Spin up the Flywheel"),
                    new RunFlywheel(flywheel, FlywheelPreset.ShootNoteAmp))),
            new DebugPrintCommand("Run the indexer"),
            new RunIndexer(indexer, IndexerPreset.ShootRing, kIndexerTimeoutSec),
            new DebugPrintCommand("Stop the flywheel and indexer"),
            RunFlywheel.stop(flywheel),
            RunIndexer.stop(indexer),
            new PivotCommand(pivot, AnglePreset.Park));
        break;
      case Speaker:
        addCommands(
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new InfoPrintCommand("ShootAtSpeaker"),
                    new PivotCommand(pivot, AnglePreset.ShootSpeaker)),
                new SequentialCommandGroup(
                    new DebugPrintCommand("Spin up the Flywheel"),
                    new RunFlywheel(flywheel, FlywheelPreset.ShootNoteSpeaker))),
            new DebugPrintCommand("Run the indexer"),
            new RunIndexer(indexer, IndexerPreset.ShootRing, kIndexerTimeoutSec),
            new DebugPrintCommand("Stop the flywheel"),
            RunFlywheel.stop(flywheel),
            new DebugPrintCommand("Stop the indexer"),
            RunIndexer.stop(indexer),
            new PivotCommand(pivot, AnglePreset.Park));
        break;
      case Trap:
        addCommands(new BotLog.InfoPrintCommand("ShootAtTrap"));
        break;
    }
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

  }
}
