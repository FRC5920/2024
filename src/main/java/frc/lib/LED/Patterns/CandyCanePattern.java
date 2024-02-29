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
package frc.lib.LED.Patterns;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.lib.LED.LEDPattern;
import frc.lib.LED.LEDStrip;

/** An object that displays a red and yellow "Hazard" pattern on an LED strip */
public class CandyCanePattern extends LEDPattern {
  /** Width of the primary segment of the pattern */
  private static final int kNumPrimarySegmentLEDs = 3;
  /** Width of the secondary segment of the pattern */
  private static final int kNumSecondarySegmentLEDs = 2;
  /** Total number of LEDs in the pattern */
  private static final int kNumPatternLEDs = kNumPrimarySegmentLEDs + kNumSecondarySegmentLEDs;
  /** Period in seconds that the pattern is updated */
  private static final double kAnimationPeriodSec = 0.2;

  /** Primary segment color in the pattern */
  private final Color m_primaryColor;
  /** Secondary segment color in the pattern */
  private final Color m_secondaryColor;

  /** Timer to determine the speed of candy cane animation */
  private final Timer m_animationTimer = new Timer();

  /** Current offset of the candy cane segments */
  private int m_pixelOffset = 0;

  /** Creates an instance of the pattern */
  public CandyCanePattern(LEDStrip ledStrip, Color primaryColor, Color secondaryColor) {
    super(ledStrip);
    m_primaryColor = primaryColor;
    m_secondaryColor = secondaryColor;
    m_animationTimer.start();
  }

  /** Resets the LED pattern to its initial position in the target LED strip */
  public void reset() {
    m_pixelOffset = 0;
    m_animationTimer.restart();
  }

  /** Display's this object's pattern */
  public void process() {
    LEDStrip strip = getLEDStrip();

    // Periodically increment the pixel offset to implement animation of the pattern
    if (m_animationTimer.hasElapsed(kAnimationPeriodSec)) {
      m_pixelOffset = (m_pixelOffset + 1) % kNumPatternLEDs;
      m_animationTimer.restart();
    }

    int counter = m_pixelOffset;
    for (int i = 0; i < strip.getNumLEDs(); ++i) {
      Color pixelColor = (counter < kNumPrimarySegmentLEDs) ? m_primaryColor : m_secondaryColor;
      strip.setLED(i, pixelColor);
      https: // github.com/Mechanical-Advantage/RobotCode2023/blob/9884d13b2220b76d430e82248fd837adbc4a10bc/src/main/java/org/littletonrobotics/frc2023/Robot.java#L37
      counter = (counter + 1) % kNumPatternLEDs;
    }
  }
}
