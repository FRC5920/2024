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
package frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.LED.LEDConstants;
import frc.lib.LED.LEDPattern;
import frc.lib.LED.LEDStrip;
import frc.lib.LED.Patterns.CandyCanePattern;
import frc.robot.commands.LEDCommands.LEDsToSolidColor;

public class LEDSubsystem extends SubsystemBase {
  //////////////////////////////////
  /// *** CONSTANTS ***
  //////////////////////////////////
  public static int kLEDPort = 9; // Port connected to the addressable LED strip

  /** Number of contiguous LEDs in the addressable LED strip */
  private static final int kNumAddressableLEDs = 20;

  /** Address of the first LED in the example LED strip */
  private static final int kFirstLEDAddress = 0;

  /** The strip of Addressable LEDs driven by this subsystem */
  private static AddressableLED m_ledStrip;

  /** Buffer used to operate on LEDs in memory before sending them to the hardware */
  private static AddressableLEDBuffer m_ledStripBuffer;

  // Start address and count for the left and right LED strips
  private static final int kFirstLeftLED = kFirstLEDAddress;
  private static final int kNumLeftLEDs = kNumAddressableLEDs / 2;
  private static final int kFirstRightLED = kFirstLEDAddress + kNumLeftLEDs;
  private static final int kNumRightLEDs = kNumAddressableLEDs - kNumLeftLEDs;

  // The LEDSubStrip objects are used to address subsets of the overall addressable LED strip
  private final LEDStrip m_leftSubStrip;
  private final LEDStrip m_rightSubStrip;

  /** Pattern to display on the LEDs when the robot is disconnected */
  private final LEDPattern m_disconnectedPattern;

  /** Pattern to display on the LEDs when the robot is disabled */
  private final LEDPattern m_disabledPattern;

  /** Creates an instance of the LEDSubsystem */
  public LEDSubsystem(Color defaultLEDColor) {
    m_ledStrip = new AddressableLED(kLEDPort);
    m_ledStripBuffer = new AddressableLEDBuffer(kNumAddressableLEDs);

    m_leftSubStrip = new LEDStrip(kFirstLeftLED, kNumLeftLEDs, m_ledStripBuffer);
    m_rightSubStrip = new LEDStrip(kFirstRightLED, kNumRightLEDs, m_ledStripBuffer);

    // Set up a White and yellow candy cane pattern when disconnected
    m_disconnectedPattern =
        new CandyCanePattern(
            new LEDStrip(0, kNumAddressableLEDs, m_ledStripBuffer),
            LEDConstants.kOff,
            Color.kYellow);

    // Set up a "Hazard" pattern in disabled mode
    m_disabledPattern =
        new CandyCanePattern(
            new LEDStrip(0, kNumAddressableLEDs, m_ledStripBuffer), Color.kYellow, Color.kRed);

    // Set up a command to set the default LED color
    setDefaultCommand(new LEDsToSolidColor(this, defaultLEDColor));

    m_ledStrip.setLength(m_ledStripBuffer.getLength());
    m_ledStrip.setData(m_ledStripBuffer);
    m_ledStrip.start();
  }

  /** This method gets called once each time the scheduler runs */
  @Override
  public void periodic() {
    if (!DriverStation.isDSAttached()) {
      m_disconnectedPattern.process();
    } else if (RobotState.isDisabled()) {
      m_disabledPattern.process();
    }

    // Apply the LED states in the addressable LED buffer to the LED hardware
    m_ledStrip.setData(m_ledStripBuffer);
  }

  /** Returns a reference to the left LED strip */
  public LEDStrip getLeftStrip() {
    return m_leftSubStrip;
  }

  /** Returns a reference to the left LED strip */
  public LEDStrip getRightStrip() {
    return m_rightSubStrip;
  }
}
