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
package frc.lib.LED;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

/**
 * LEDLayer allows LEDs in a virtual LEDStrip to be written to in a layered fashion.
 *
 * @details Non-zero LED values in an LEDLayer will be applied to their target LED strip. LED's set
 *     to an 'unlit' state (i.e. red, green, and blue elements all equal to zero) will not be
 *     applied when the layer is rendered to the target strip.
 */
public class LEDLayer {

  /** kTransparent gives the value of an LED element that should be treated as transparent */
  public static final Color kTransparent = new Color(0, 0, 0);

  /** The LEDStrip the layer is targeting */
  private final LEDStrip m_strip;

  /** Buffer of LED states addressed by the layer */
  private AddressableLEDBuffer m_ledBuffer;

  /** Priority of the layer in its parent strip (zero is lowest priority) */
  public final int priority;

  /**
   * Creates an instance of the object
   *
   * @param strip LEDStrip the object is targeting
   * @param priority Priority of the layer
   */
  public LEDLayer(LEDStrip strip, int priority) {
    m_strip = strip;
    this.priority = priority;
    m_ledBuffer = new AddressableLEDBuffer(strip.getNumLEDs());
  }

  /**
   * Returns a Color representing the LED at a given offset inside the LEDStrip
   *
   * @param offset Offset of the target LED in the strip (0 ... (numLEDs - 1))
   */
  public Color getLED(int offset) {
    return m_ledBuffer.getLED(offset);
  }

  /**
   * Sets the LED at a given offset inside the LEDStrip to a specified Color
   *
   * @param offset Offset of the target LED in the strip (0 ... (numLEDs - 1))
   * @param color Color to set the LED to
   */
  public void setLED(int offset, Color color) {
    m_ledBuffer.setRGB(
        offset, (int) (color.red * 255), (int) (color.green * 255), (int) (color.blue * 255));
  }

  /**
   * Sets all LEDs in the strip to a given color
   *
   * @param color Color to set the LEDs to
   */
  public void fillColor(Color color) {
    for (int i = 0; i < m_ledBuffer.getLength(); ++i) {
      this.setLED(i, color);
    }
  }

  /** Returns the number of LEDs targeted by the object */
  public int getNumLEDs() {
    return m_ledBuffer.getLength();
  }
}
