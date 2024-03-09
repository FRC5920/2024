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
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
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
  private static final double kIndexerTimeoutSec = 1.0;

  private final FlywheelPreset flywheelPreset;
  private final AnglePreset pivotAngle;

  /** Creates a new ShootNote. */
  public ShootNote(
      ScoringTarget target,
      PivotSubsystem pivot,
      FlywheelSubsystem flywheel,
      IndexerSubsystem indexer) {

    switch (target) {
      case Amp:
        flywheelPreset = FlywheelPreset.ShootNoteAmp;
        pivotAngle = AnglePreset.ShootAmp;
        break;
      case Speaker:
        flywheelPreset = FlywheelPreset.ShootNoteSpeaker;
        pivotAngle = AnglePreset.ShootSpeaker;
        break;
      case Trap:
      default:
        flywheelPreset = FlywheelPreset.Stop;
        pivotAngle = AnglePreset.Park;
    }

    // Register subsystem requirements
    addRequirements(pivot, flywheel, indexer);

    addCommands(
        new InfoPrintCommand("ShootNote at " + target.name()),
        new ParallelRaceGroup(
            new RunFlywheel(flywheel, flywheelPreset),
            new SequentialCommandGroup(
                new PivotCommand(pivot, pivotAngle),
                new WaitUntilCommand(() -> flywheelReachedSpeed(flywheel, flywheelPreset))
                    .withTimeout(5.0),
                new DebugPrintCommand("Run the indexer"),
                new RunIndexer(indexer, IndexerPreset.ShootRing, kIndexerTimeoutSec),
                new DebugPrintCommand("Indexer finished"))),
        new InstantCommand(() -> flywheel.setFlywheelVelocity(0.0)),
        new InfoPrintCommand("Pivot back to park"),
        new PivotCommand(pivot, AnglePreset.Park));

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

  }

  private static boolean flywheelReachedSpeed(FlywheelSubsystem flywheel, FlywheelPreset preset) {
    return Math.abs(flywheel.getFlywheelVelocity()) >= (0.9 * Math.abs(preset.flywheelRPM));
  }
}
