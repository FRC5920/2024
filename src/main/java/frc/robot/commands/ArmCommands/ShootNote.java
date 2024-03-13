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

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.lib.logging.BotLog.DebugPrintCommand;
import frc.lib.logging.BotLog.InfoPrintCommand;
import frc.robot.Constants.ScoringTarget;
import frc.robot.commands.ArmCommands.PivotCommand.PivotPreset;
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
  private final double targetFlywheelRPM;
  private final PivotPreset pivotAngle;

  /** Creates a new ShootNote. */
  public ShootNote(
      ScoringTarget target,
      PivotSubsystem pivot,
      FlywheelSubsystem flywheel,
      IndexerSubsystem indexer) {

    switch (target) {
      case Amp:
        flywheelPreset = FlywheelPreset.ShootNoteAmp;
        pivotAngle = PivotPreset.ShootAmp;
        break;
      case Speaker:
        flywheelPreset = FlywheelPreset.ShootNoteSpeaker;
        pivotAngle = PivotPreset.ShootSpeaker;
        break;
      case Trap:
      default:
        flywheelPreset = FlywheelPreset.Stop;
        pivotAngle = PivotPreset.Park;
    }

    targetFlywheelRPM = flywheelPreset.getRPM();

    // Register subsystem requirements
    addRequirements(pivot, flywheel, indexer);

    addCommands(
        new ConditionalCommand(
            new SequentialCommandGroup(
                new InfoPrintCommand("ShootNote at " + target.name()),
                new ParallelRaceGroup(
                    new RunFlywheel(flywheel, flywheelPreset),
                    new SequentialCommandGroup(
                        new PivotCommand(pivot, pivotAngle),
                        new WaitUntilCommand(
                                () -> flywheelReachedSpeed(flywheel, targetFlywheelRPM))
                            .withTimeout(5.0),
                        new DebugPrintCommand("Run the indexer"),
                        new RunIndexer(indexer, IndexerPreset.ShootRing, kIndexerTimeoutSec),
                        new DebugPrintCommand("Indexer finished"))),
                new InstantCommand(() -> flywheel.setFlywheelVelocity(0.0)),
                new InfoPrintCommand("Pivot back to park"),
                new PivotCommand(pivot, PivotPreset.Park)),
            // If no gamepiece is present, don't do anything
            new InfoPrintCommand("ShootNote aborted because intake is empty"),
            indexer::gamepieceIsDetected));

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

  }

  private boolean flywheelReachedSpeed(FlywheelSubsystem flywheel, double targetSpeed) {
    return Math.abs(flywheel.getFlywheelVelocity()) >= (0.9 * Math.abs(targetFlywheelRPM));
  }
}
