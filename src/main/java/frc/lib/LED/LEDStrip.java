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
import edu.wpi.first.wpilibj.util.Color8Bit;

/** An object used to represent a portion of an addressable LED strip */
public class LEDStrip {
  /** Address of the first LED in the object's portion of the overall LED strip */
  private final int m_firstAddress;
  /** Number of contiguous LEDs in the object's portion of the overall LED strip */
  private final int m_numLEDs;

  /** Buffer of LED states addressed by this object */
  private AddressableLEDBuffer m_ledBuffer;

  /**
   * Creates an instance of the object
   *
   * @param start Start address (index) of the strip
   * @param count Number of LED's in the strip
   */
  public LEDStrip(int start, int count, AddressableLEDBuffer ledBuffer) {
    m_firstAddress = start;
    m_numLEDs = count;
    m_ledBuffer = ledBuffer;

    if ((start + count) > ledBuffer.getLength()) {
      throw new IndexOutOfBoundsException(
          String.format("LEDStrip would exceed the bounds of its AddressableLEDBuffer"));
    }
  }

  /**
   * Returns a Color representing the LED at a given offset inside the LEDStrip
   *
   * @param offset Offset of the target LED in the strip (0 ... (numLEDs - 1))
   */
  public Color getLED(int offset) {
    return m_ledBuffer.getLED(translateAddress(offset));
  }

  /**
   * Returns a Color8Bit representing the LED at a given offset inside the LEDStrip
   *
   * @param offset Offset of the target LED in the strip (0 ... (numLEDs - 1))
   */
  public Color8Bit getLED8Bit(int offset) {
    return m_ledBuffer.getLED8Bit(translateAddress(offset));
  }

  /**
   * Sets the LED at a given offset inside the LEDStrip to a specified Color
   *
   * @param offset Offset of the target LED in the strip (0 ... (numLEDs - 1))
   * @param color Color to set the LED to
   */
  public void setLED(int offset, Color color) {
    m_ledBuffer.setRGB(
        translateAddress(offset),
        (int) (color.red * 255),
        (int) (color.green * 255),
        (int) (color.blue * 255));
  }

  /**
   * Sets all LEDs in the strip to a given color
   *
   * @param color Color to set the LEDs to
   */
  public void fillColor(Color color) {
    for (int i = 0; i < m_numLEDs; ++i) {
      this.setLED(i, color);
    }
  }

  /** Returns the absolute address of the first LED targeted by the object */
  public int getFirstAddress() {
    return m_firstAddress;
  }

  /** Returns the number of LEDs targeted by the object */
  public int getNumLEDs() {
    return m_numLEDs;
  }

  /**
   * Translates a zero-based address within the strip to an absolute address in the
   * AddressableLEDBuffer targeted by the object
   */
  public int translateAddress(int address) {
    // Catch invalid addresses
    if ((address < 0) || (address >= m_numLEDs)) {
      throw new IndexOutOfBoundsException(address);
    }

    return m_firstAddress + address;
  }
}
