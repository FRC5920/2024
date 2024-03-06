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

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.logging.BotLog;
import frc.lib.logging.BotLog.DebugPrintCommand;
import frc.lib.logging.BotLog.InfoPrintCommand;
import frc.robot.Constants.ScoringTarget;
import frc.robot.commands.ArmCommands.PivotCommand.AnglePreset;
import frc.robot.subsystems.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.flywheel.RunFlywheelAtSpeed;
import frc.robot.subsystems.flywheel.RunFlywheelAtSpeed.FlywheelPreset;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem.IndexerPreset;
import frc.robot.subsystems.indexer.IndexerSubsystem.RunIndexerAtSpeed;
import frc.robot.subsystems.pivot.PivotSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootNote extends SequentialCommandGroup {
  /** Creates a new ShootNote. */
  public ShootNote(
      ScoringTarget target,
      PivotSubsystem pivot,
      FlywheelSubsystem flywheel,
      IndexerSubsystem indexer) {
    switch (target) {
      case Amp:
        addCommands(
            new InfoPrintCommand("ShootAtAmp"),
            new PivotCommand(pivot, AnglePreset.ShootAmp),
            new DebugPrintCommand("Spin up the Flywheel"),
            new RunFlywheelAtSpeed(flywheel, FlywheelPreset.ShootNoteAmp),
            new DebugPrintCommand("Run the indexer"),
            new RunIndexerAtSpeed(indexer, IndexerPreset.ShootRing),
            new DebugPrintCommand("Stop the flywheel"),
            new InstantCommand(() -> flywheel.setFlywheelVelocity(0.0)));
        break;
      case Speaker:
        addCommands(
            new BotLog.InfoPrintCommand("ShootAtSpeaker"),
            new BotLog.DebugPrintCommand("Pivot to shooting position"),
            new PivotCommand(pivot, AnglePreset.ShootSpeaker),
            new DebugPrintCommand("Spin up the Flywheel"),
            new RunFlywheelAtSpeed(flywheel, FlywheelPreset.ShootNoteSpeaker),
            new DebugPrintCommand("Run the indexer"),
            new RunIndexerAtSpeed(indexer, IndexerPreset.ShootRing),
            new DebugPrintCommand("Stop the flywheel"),
            new InstantCommand(() -> flywheel.setFlywheelVelocity(0.0)));
        break;
      case Trap:
        addCommands(new BotLog.InfoPrintCommand("ShootAtTrap"));
        break;
    }
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

  }
}
