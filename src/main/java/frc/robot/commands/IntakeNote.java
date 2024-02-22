// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.pivot.PivotSubsystem;

/** Command composition used to pick up and intake a note */
public class IntakeNote extends SequentialCommandGroup {

  /** Creates an instance of the command
   * @param intake  IntakeSubsystem to operate on
   * @param pivot  PivotSubsystem to operate on
   */
  public IntakeNote(IntakeSubsystem intake, PivotSubsystem pivot) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands();
  }
}
