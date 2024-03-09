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
import frc.lib.logging.BotLog;
import frc.robot.commands.subsystemCommands.RunFlywheel.FlywheelPreset;
import frc.robot.subsystems.flywheel.FlywheelSubsystem;

//////////////////////////////////////////////////////////////////////////////////////////////////
public class RunFlywheel extends Command {

  public enum FlywheelPreset {
    Stop(0.0),
    IntakeRing(-2000.0),
    ShootNoteAmp(1000.0),
    ShootNoteSpeaker(4500.0);

    public final double flywheelRPM;

    private FlywheelPreset(double flywheelRPS) {
      this.flywheelRPM = flywheelRPS;
    }
  }

  /** The Flywheel subsystem to operate on */
  private final FlywheelSubsystem m_flywheel;

  /** Flywheel speed preset to be run */
  private final FlywheelPreset m_preset;

  private final double m_timeoutSec;
  private final Timer m_timer = new Timer();

  /** Creates a command that will run the Flywheel for a specified max number of seconds */
  public RunFlywheel(FlywheelSubsystem flywheel, FlywheelPreset preset, double timeoutSec) {
    m_flywheel = flywheel;
    m_preset = preset;
    m_timeoutSec = timeoutSec;
    addRequirements(flywheel);
  }

  /** Creates a command that will run the Flywheel perpetually */
  public RunFlywheel(FlywheelSubsystem flywheel, FlywheelPreset preset) {
    this(flywheel, preset, -1.0);
  }

  @Override
  public void initialize() {
    BotLog.Debugf(
        "Run Flywheel at %f RPM%s",
        m_preset.flywheelRPM,
        (m_timeoutSec > 0.0) ? String.format(" for %f seconds", m_timeoutSec) : " perpetually");

    // Set the flywheel speed
    m_flywheel.setFlywheelVelocity(m_preset.flywheelRPM);

    m_timer.reset();
    m_timer.start();
  }

  @Override
  public boolean isFinished() {
    boolean finished = false;
    if (m_timeoutSec > 0.0) {
      finished = m_timer.hasElapsed(m_timeoutSec);
      if (finished) {
        BotLog.Debugf("RunFlywheel timed out after %f sec", m_timer.get());
      }
    }
    return finished;
  }

  @Override
  public void end(boolean interrupted) {
    BotLog.Debug("RunFlywheel " + (interrupted ? "interrupted" : "finished"));
  }
}
