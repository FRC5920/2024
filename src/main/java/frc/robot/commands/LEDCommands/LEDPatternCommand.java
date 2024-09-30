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
package frc.robot.commands.LEDCommands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.lib.LED.ColorConstants;
import frc.lib.LED.LEDLayer;
import frc.lib.LED.LEDPattern;
import frc.lib.LED.Patterns.BlasterBoltPattern;
import frc.lib.LED.Patterns.CandyCanePattern;
import frc.lib.LED.Patterns.RainbowPattern;
import frc.robot.subsystems.LEDs.LEDSubsystem;
import frc.robot.subsystems.LEDs.LEDSubsystem.LayerID;
import frc.robot.subsystems.LEDs.LEDSubsystem.StripID;

/** A command that implements display of a pattern on an LED layer */
public class LEDPatternCommand extends Command {

  // Pattern applied to the LEDs layer
  private LEDPattern m_pattern;

  /**
   * Creates a new Command that implements the specified LED pattern
   *
   * @param layer LED layer the pattern will be written to
   * @param pattern LED pattern to display on the layer
   */
  private LEDPatternCommand(LEDPattern pattern) {
    m_pattern = pattern;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_pattern.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_pattern.process();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  // Overload that allows the command to display on LEDs when the robot is disabled
  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  /** LED patterns available as commands */
  public enum Pattern {
    BlasterBolt,
    CandyCane,
    Rainbow
  }

  /** Creates a command to display a blaster bolt pattern on the LEDs in a subsystem */
  public static class BlasterBoltPatternCommand extends ParallelCommandGroup {
    /** Constructs a BlasterBoltPatternCommand with default colors and speed */
    public BlasterBoltPatternCommand(LEDSubsystem subsystem, LayerID layerID) {
      this(subsystem, layerID, ColorConstants.kVikoticsYellow, 4);
    }

    /**
     * Constructs a BlasterBoltPatternCommand with default colors and speed
     *
     * @param subsystem LED subsystem used to display the pattern
     * @param layerID LED layer to display the pattern on
     * @param color Color of the blaster bolt
     * @param cyclesPerPixel Update rate of the pattern (higher values make a slower pattern)
     */
    public BlasterBoltPatternCommand(
        LEDSubsystem subsystem, LayerID layerID, Color color, int cyclesPerPixel) {
      LEDLayer leftLayer = subsystem.getLayer(StripID.Left, layerID);
      LEDLayer rightLayer = subsystem.getLayer(StripID.Right, layerID);

      addRequirements(subsystem);

      this.addCommands(
          new LEDPatternCommand(new BlasterBoltPattern(leftLayer, color, cyclesPerPixel))
              .ignoringDisable(true),
          new LEDPatternCommand(new BlasterBoltPattern(rightLayer, color, cyclesPerPixel))
              .ignoringDisable(true));
    }
  }

  /** Creates a command to display a candy cane pattern on the LEDs in a subsystem */
  public static class CandyCanePatternCommand extends ParallelCommandGroup {
    /** Constructs a CandyCanePatternCommand with default colors */
    public CandyCanePatternCommand(LEDSubsystem subsystem, LayerID layerID) {
      this(subsystem, layerID, ColorConstants.kOff, Color.kYellow);
    }

    /**
     * Constructs a BlasterBoltPatternCommand with default colors and speed
     *
     * @param subsystem LED subsystem used to display the pattern
     * @param layerID LED layer to display the pattern on
     * @param primaryColor Primary color in the candy cane pattern
     * @param secondaryColor Secondary color in the candy cane pattern
     */
    public CandyCanePatternCommand(
        LEDSubsystem subsystem, LayerID layerID, Color primaryColor, Color secondaryColor) {
      LEDLayer leftLayer = subsystem.getLayer(StripID.Left, layerID);
      LEDLayer rightLayer = subsystem.getLayer(StripID.Right, layerID);

      addRequirements(subsystem);

      this.addCommands(
          new LEDPatternCommand(new CandyCanePattern(leftLayer, primaryColor, secondaryColor))
              .ignoringDisable(true),
          new LEDPatternCommand(new CandyCanePattern(rightLayer, primaryColor, secondaryColor))
              .ignoringDisable(true));
    }
  }

  /** Creates a command to display a rainbow pattern on the LEDs in a subsystem */
  public static class RainbowPatternCommand extends ParallelCommandGroup {
    /** Constructs a RainbowPatternCommand */
    public RainbowPatternCommand(LEDSubsystem subsystem, LayerID layerID) {
      LEDLayer leftLayer = subsystem.getLayer(StripID.Left, layerID);
      LEDLayer rightLayer = subsystem.getLayer(StripID.Right, layerID);

      addRequirements(subsystem);

      this.addCommands(
          new LEDPatternCommand(new RainbowPattern(leftLayer)).ignoringDisable(true),
          new LEDPatternCommand(new RainbowPattern(rightLayer)).ignoringDisable(true));
    }
  }
}
