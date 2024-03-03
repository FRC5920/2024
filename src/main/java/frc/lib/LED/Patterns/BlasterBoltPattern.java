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
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.lib.LED.LEDPattern;
import frc.lib.LED.LEDStrip;
import frc.robot.Constants;

/** Add your docs here. */
public class BlasterBoltPattern extends LEDPattern {
  private final int kPatternLength = 6;
  private Color8Bit m_pattern[];
  private int m_arraySize;
  /** length of LED strip */
  private int m_startIndex;
  /** starting index in the LED strip */
  private int m_iteration;
  /** number of times array has been processed */
  private int m_arrayPos;
  /** current position to start writing into the LED strip array */
  private int m_patternStart;
  /** where in the pattern to begin to copy from */
  private int m_patternEnd;

  /**
   * Controls the apparent speed of the LED animation by setting the number of robot cycles that
   * must elapse between each pattern update
   */
  private double m_secondsPerPixel;

  /** Timer used to control the speed of the LED animation */
  private Timer m_animationTimer = new Timer();

  /** Direction the LED will appear to move */
  public enum Direction {
    kForward,
    /** pattern moves from lowest LED address to highest */
    kReverse
    /** pattern moves from highest LED address to lowest */
  };

  private Direction m_direction;

  /**
   * Creates an LEDPattern object that will display a pattern in an LED strip
   *
   * @param ledStrip The LED strip (or sub-strip) where the pattern will be applied
   * @param color Color to use for the pattern
   * @param cyclesPerPixel Number of robot cycles that must elapse between pixel animations (higher
   *     values create slower animation)
   */
  public BlasterBoltPattern(LEDStrip ledStrip, Color color, int cyclesPerPixel) {
    super(ledStrip);
    m_direction = Direction.kForward;
    m_startIndex = 0;
    m_arraySize = ledStrip.getNumLEDs();

    m_secondsPerPixel = Constants.robotPeriodSec * ((cyclesPerPixel < 1) ? 1 : cyclesPerPixel);
    reset();

    // Initialize the LED pattern array
    m_pattern = new Color8Bit[kPatternLength];
    setColor(new Color8Bit(color));

    m_animationTimer.start();
  }

  /**
   * Sets the color displayed by the pattern
   *
   * @param color The color the pattern should display
   */
  public void setColor(Color8Bit color) {
    m_pattern[0] = scaleIntensity(color, 1.0);
    m_pattern[1] = scaleIntensity(color, 0.75);
    m_pattern[2] = scaleIntensity(color, 0.5);
    m_pattern[3] = scaleIntensity(color, 0.25);
    m_pattern[4] = scaleIntensity(color, 0.125);
    m_pattern[5] = scaleIntensity(color, 0.06);
  }

  /** Resets the LED pattern to its initial position in the target LED strip */
  public void reset() {
    m_iteration = 0;
    m_arrayPos = 0;
    m_patternStart = 0;
    m_patternEnd = 0;
    m_animationTimer.reset();
  }

  /**
   * Translate an index in the LED array according to whether the pattern is being moved forward or
   * reverse in the array
   */
  private int translateIndex(int index) {
    int result = index;
    if (m_direction == Direction.kReverse) {
      result = (m_arraySize - 1) - index;
    }

    return result;
  }

  /** Display's this object's pattern */
  public void process() {
    // Don't do anything until the minimum time has elapsed
    if (!m_animationTimer.hasElapsed(m_secondsPerPixel)) {
      return;
    }

    Color8Bit off = new Color8Bit(0, 0, 0);
    LEDStrip strip = getLEDStrip();

    // Default all LEDs in the strip to an OFF state
    for (int i = m_startIndex; i < m_startIndex + m_arraySize; ++i) {
      strip.setLED8Bit(i, off);
    }

    // Copy the pattern into ledArray starting at the current
    // array index
    int a = m_startIndex + translateIndex(m_arrayPos);
    for (int p = m_patternStart; p <= m_patternEnd; ++p) {
      Color8Bit c = m_pattern[p];
      strip.setLED8Bit(a, c);
      a += (m_direction == Direction.kForward) ? -1 : 1;
    }

    m_iteration += 1;
    if (m_iteration > (m_arraySize + kPatternLength - 2)) {
      reset();
    } else {
      if (m_arrayPos < (m_arraySize - 1)) {
        m_arrayPos += 1;
      }

      if (m_iteration >= m_arraySize) {
        m_patternStart += 1;
      }

      if (m_patternEnd < (kPatternLength - 1)) {
        m_patternEnd += 1;
      }
    }

    m_animationTimer.restart();
  }
}
