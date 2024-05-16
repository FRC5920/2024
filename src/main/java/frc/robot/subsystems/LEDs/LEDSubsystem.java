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
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.LED.ColorConstants;
import frc.lib.LED.LEDLayer;
import frc.lib.LED.LEDStrip;
import frc.robot.commands.LEDCommands.LEDPatternCommand.CandyCanePatternCommand;

public class LEDSubsystem extends SubsystemBase {
  //////////////////////////////////
  /// *** CONSTANTS ***
  //////////////////////////////////
  public static int kLEDPort = 9; // Port connected to the addressable LED strip

  /** Number of contiguous LEDs in the addressable LED strip */
  private static final int kNumAddressableLEDs = 32;

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

  /** Command to execute on the subsystem when the robot is disconnected */
  private Command m_disconnectedCommand;

  /** Command to execute on the subsystem when the robot is disabled */
  private Command m_disabledCommand;

  /** Creates an instance of the LEDSubsystem */
  public LEDSubsystem(Color defaultLEDColor) {
    m_ledStrip = new AddressableLED(kLEDPort);
    m_ledStripBuffer =
        RobotBase.isReal()
            ? new AddressableLEDBufferGRB(kNumAddressableLEDs)
            : new AddressableLEDBuffer(kNumAddressableLEDs);

    m_ledStrip.setLength(m_ledStripBuffer.getLength());
    m_ledStrip.setData(m_ledStripBuffer);
    m_ledStrip.start();

    m_leftSubStrip = new LEDStrip(kFirstLeftLED, kNumLeftLEDs);
    m_rightSubStrip = new LEDStrip(kFirstRightLED, kNumRightLEDs);

    // Set up patterns displayed when disconnected or disabled
    m_disconnectedCommand =
        new CandyCanePatternCommand(this, LayerID.Bottom, ColorConstants.kOff, Color.kYellow);
    m_disabledCommand =
        new CandyCanePatternCommand(this, LayerID.Bottom, Color.kYellow, Color.kRed);
  }

  /** This method gets called once each time the scheduler runs */
  @Override
  public void periodic() {
    if (!DriverStation.isDSAttached()) {
      m_disconnectedCommand.execute();
    } else if (RobotState.isDisabled()) {
      m_disabledCommand.execute();
    }

    m_leftSubStrip.renderLayers(m_ledStripBuffer);
    m_rightSubStrip.renderLayers(m_ledStripBuffer);

    // Apply the LED states in the addressable LED buffer to the LED hardware
    m_ledStrip.setData(m_ledStripBuffer);
  }

  /** Turns all the LEDs off on all strips */
  public void allOff() {
    m_leftSubStrip.allLayersOff();
    m_rightSubStrip.allLayersOff();
  }

  /** Identifiers for individual LED strips on the robot */
  public enum StripID {
    Left,
    Right
  }

  /** Identifiers for individual LED strips on the robot */
  public enum LayerID {
    Bottom(0),
    Middle(1),
    Top(2);

    public final int priority;

    private LayerID(int priority) {
      this.priority = priority;
    }
  }

  /** Returns a reference to a given LED strip */
  public LEDLayer getLayer(StripID stripID, LayerID layerID) {
    LEDStrip strip = (stripID == StripID.Left) ? m_leftSubStrip : m_rightSubStrip;
    return strip.addLayer(layerID.priority);
  }

  /** Sets the command executed when the Driver Station is disconnected */
  void setDisconnectedCommand(Command command) {
    m_disconnectedCommand = command;
  }

  /** Returns the command executed when the Driver Station is disconnected */
  Command getDisconnectedCommand() {
    return m_disconnectedCommand;
  }

  /** Sets the command executed when the robot is disabled */
  void setDisabledCommand(Command command) {
    m_disabledCommand = command;
  }

  /** Returns the command executed when the robot is disabled */
  Command getDisabledCommand() {
    return m_disabledCommand;
  }

  /** Customized version of an AddressableLEDBuffer for LED strips that use GRB data format */
  private static class AddressableLEDBufferGRB extends AddressableLEDBuffer {

    public AddressableLEDBufferGRB(int length) {
      super(length);
    }

    @Override
    public void setRGB(int index, int r, int g, int b) {
      super.setRGB(index, g, r, b);
    }
  }
}
