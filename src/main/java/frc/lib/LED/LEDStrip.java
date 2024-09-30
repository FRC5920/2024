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
import java.util.Comparator;
import java.util.PriorityQueue;

/**
 * An LEDStrip represents a range of contiguous addressable LEDs and a set of layers used to set the
 * colors of those LEDs
 */
public class LEDStrip {
  /** Address of the first LED in the strip */
  private final int m_firstAddress;

  /** Number of LEDs in the strip */
  private final int m_numLEDs;

  /** LED 'layers' rendered onto the strip */
  PriorityQueue<LEDLayer> m_layers;

  /**
   * Creates an instance of the object
   *
   * @param start Start address (index) of the strip
   * @param count Number of LED's in the strip
   */
  public LEDStrip(int start, int count) {
    m_firstAddress = start;
    m_numLEDs = count;
    m_layers = new PriorityQueue<LEDLayer>(5, new LayerComparator());
  }

  /**
   * Adds an LED layer that can be used to set LED states in the strip
   *
   * @param priority Priority of the layer (zero is lowest priority)
   */
  public LEDLayer addLayer(int priority) {
    LEDLayer layer = new LEDLayer(this, priority);
    m_layers.add(layer);
    return layer;
  }

  /**
   * Removes LED layer(s) with a given priority
   *
   * @param targetPriority Priority of the LED layer to remove
   */
  public void removeLayer(int targetPriority) {
    m_layers.removeIf(
        (layer) -> {
          return layer.priority == targetPriority;
        });
  }

  /** Turns all the LEDs off in all LED layers */
  public void allLayersOff() {
    // Set all LEDs in the strip to unlit
    for (LEDLayer layer : m_layers) {
      layer.fillColor(ColorConstants.kOff);
    }
  }

  /** Returns the number of LEDs targeted by the object */
  public int getNumLEDs() {
    return m_numLEDs;
  }

  /**
   * Renders the object's LED layers into a target Addressable LED buffer.
   *
   * @param buffer Addressable LED buffer to render into
   * @remarks Higher-priority layers overlay lower priority layers. In layers with a priority of
   *     zero, LEDs given a value of ColorConstants.kOff will be unlit. In all other layers, this
   *     LED value will act as 'transparent', allowing the color in lower layers to come through.
   */
  public void renderLayers(AddressableLEDBuffer buffer) {
    m_layers.forEach(
        (layer) -> {
          for (int i = 0; i < m_numLEDs; ++i) {
            // In layers with a non-zero priority, treat LEDs set to 'off' as transparent
            if ((layer.priority != 0) && (layer.getLED(i) == LEDLayer.kTransparent)) {
              continue;
            }

            buffer.setLED(translateAddress(i), layer.getLED(i));
          }
        });
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

  /** LayerComparator implements Comparator<> to provide sorting of LEDLayers by priority */
  private class LayerComparator implements Comparator<LEDLayer> {

    // Implement compare() method of Comparator for ascending order of LED layers
    public int compare(LEDLayer l1, LEDLayer l2) {
      if (l1.priority == l2.priority) {
        return 0;
      }

      return (l1.priority > l2.priority) ? 1 : -1;
    }
  }
}
