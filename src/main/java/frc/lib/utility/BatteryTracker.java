////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2023-6328 FIRST and other WPILib contributors.
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
package frc.lib.utility;

import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.Constants;
import java.util.Arrays;

public class BatteryTracker {
  public static final String defaultName = "0000-000";

  private static final int nameLength = 8;
  private static final byte[] scanCommand =
      new byte[] {0x7e, 0x00, 0x08, 0x01, 0x00, 0x02, 0x01, (byte) 0xab, (byte) 0xcd};
  private static final byte[] responsePrefix =
      new byte[] {0x02, 0x00, 0x00, 0x01, 0x00, 0x33, 0x31};
  private static final int fullResponseLength = responsePrefix.length + nameLength;

  private static String name = defaultName;

  /**
   * Scans the battery. This should be called before the first loop cycle
   *
   * @param timeout The time to wait before giving up
   */
  public static String scanBattery(double timeout) {
    if (Constants.getMode() == RobotRunMode.REAL) {
      // Only scan in real mode

      try (SerialPort port = new SerialPort(9600, SerialPort.Port.kUSB)) {
        port.setTimeout(timeout);
        port.setWriteBufferSize(scanCommand.length);
        port.setReadBufferSize(fullResponseLength);

        port.write(scanCommand, scanCommand.length);
        byte[] response = port.read(fullResponseLength);

        // Ensure response is correct length
        if (response.length != fullResponseLength) {
          System.out.println(
              "[BatteryTracker] Expected "
                  + fullResponseLength
                  + " bytes from scanner, got "
                  + response.length);
          return name;
        }

        // Ensure response starts with prefix
        for (int i = 0; i < responsePrefix.length; i++) {
          if (response[i] != responsePrefix[i]) {
            System.out.println("[BatteryTracker] Invalid prefix from scanner.  Got data:");
            System.out.println("[BatteryTracker] " + Arrays.toString(response));
            return name;
          }
        }

        // Read name from data
        byte[] batteryNameBytes = new byte[nameLength];
        System.arraycopy(response, responsePrefix.length, batteryNameBytes, 0, nameLength);
        name = new String(batteryNameBytes);
        System.out.println("[BatteryTracker] Scanned battery " + name);

      } catch (Exception e) {
        System.out.println("[BatteryTracker] Exception while trying to scan battery");
        e.printStackTrace();
      }
    }

    return name;
  }

  /** Returns the name of the last scanned battery. */
  public static String getName() {
    return name;
  }
}
