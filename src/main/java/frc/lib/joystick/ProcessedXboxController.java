////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2023 FIRST and other WPILib contributors.
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
package frc.lib.joystick;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.HashMap;

/////////////////////////////////////////////////////////////////////////////
/** This utility class wraps the functionality of a joystick controller for driving the robot */
public class ProcessedXboxController extends XboxController {

  /** Stick processing defaults */
  private static final double kDefaultStickSensitivity = 1.0;

  private static final double kDefaultStickDeadbandLower = 0.05;
  private static final double kDefaultStickDeadbandUpper = 0.95;

  /** Trigger processing defaults */
  private static final double kDefaultTriggerSensitivity = 1.0;

  private static final double kDefaultTriggerDeadbandLower = 0.05;
  private static final double kDefaultTriggerDeadbandUpper = 0.95;

  /** Processors applied to axis values from joysticks and triggers */
  private HashMap<XboxController.Axis, AxisProcChain> m_axisProcessorMap;

  /** Enumeration of buttons on the POV pad */
  public enum POVAngle {
    Up(0),
    UpRight(45),
    Right(90),
    DownRight(135),
    Down(180),
    DownLeft(225),
    Left(270),
    UpLeft(315),
    CenterPress(-1);

    private final int angleDeg;

    private POVAngle(int angle) {
      angleDeg = angle;
    }
  }

  /** Xbox controller 'A' button */
  public final JoystickButton A;
  /** Xbox controller 'B' button */
  public final JoystickButton B;
  /** Xbox controller 'X' button */
  public final JoystickButton X;
  /** Xbox controller 'Y' button */
  public final JoystickButton Y;
  /** Xbox controller left bumper button */
  public final JoystickButton leftBumper;
  /** Xbox controller right bumper button */
  public final JoystickButton rightBumper;
  /** Xbox controller left bumper button */
  public final JoystickButton leftStickPress;
  /** Xbox controller right bumper button */
  public final JoystickButton rightStickPress;
  /** Xbox controller 'Back' button */
  public final JoystickButton back;
  /** Xbox controller 'Start' button */
  public final JoystickButton start;

  /** Left trigger as a button */
  public Trigger leftTriggerAsButton;

  /** Right trigger as a button */
  public Trigger rightTriggerAsButton;

  /**
   * Creates a JoystickController instance that communicates with an Xbox controller on a specified
   * port and uses default processing for joystick axes
   *
   * @param port Port number of the Xbox controller the object will use
   */
  public ProcessedXboxController(int port) {
    super(port);

    m_axisProcessorMap = new HashMap<XboxController.Axis, AxisProcChain>();

    // Configure default stick processing
    AxisProcChain.Config defaultStickConfig =
        new AxisProcChain.Config(
            kDefaultStickSensitivity,
            new double[] {kDefaultStickDeadbandLower, kDefaultStickDeadbandUpper});
    m_axisProcessorMap.put(XboxController.Axis.kLeftX, new AxisProcChain(defaultStickConfig));
    m_axisProcessorMap.put(XboxController.Axis.kLeftY, new AxisProcChain(defaultStickConfig));
    m_axisProcessorMap.put(XboxController.Axis.kRightX, new AxisProcChain(defaultStickConfig));
    m_axisProcessorMap.put(XboxController.Axis.kRightY, new AxisProcChain(defaultStickConfig));

    // Set up default trigger processing
    AxisProcChain.Config defaultTriggerConfig =
        new AxisProcChain.Config(
            kDefaultTriggerSensitivity,
            new double[] {kDefaultTriggerDeadbandLower, kDefaultTriggerDeadbandUpper});
    m_axisProcessorMap.put(
        XboxController.Axis.kLeftTrigger, new AxisProcChain(defaultTriggerConfig));
    m_axisProcessorMap.put(
        XboxController.Axis.kRightTrigger, new AxisProcChain(defaultTriggerConfig));

    // Create buttons
    A = new JoystickButton(this, XboxController.Button.kA.value);
    B = new JoystickButton(this, XboxController.Button.kB.value);
    X = new JoystickButton(this, XboxController.Button.kX.value);
    Y = new JoystickButton(this, XboxController.Button.kY.value);
    leftBumper = new JoystickButton(this, XboxController.Button.kLeftBumper.value);
    rightBumper = new JoystickButton(this, XboxController.Button.kRightBumper.value);
    leftStickPress = new JoystickButton(this, XboxController.Button.kLeftStick.value);
    rightStickPress = new JoystickButton(this, XboxController.Button.kRightStick.value);
    back = new JoystickButton(this, XboxController.Button.kBack.value);
    start = new JoystickButton(this, XboxController.Button.kStart.value);
    leftTriggerAsButton = new Trigger(() -> this.getLeftTriggerAxis() > 0.5);
    rightTriggerAsButton = new Trigger(() -> this.getRightTriggerAxis() > 0.5);
  }

  /** Return a mutable reference to the processing chain applied to a specified stick */
  public AxisProcChain getStickProcessing(XboxController.Axis axisId) {
    if ((XboxController.Axis.kLeftTrigger == axisId)
        || (XboxController.Axis.kRightTrigger == axisId)) {
      throw new IndexOutOfBoundsException("axis ID specifies a trigger, not a stick");
    }

    return m_axisProcessorMap.get(axisId);
  }

  /**
   * Return a mutable reference to the processing chain applied to a specified controller trigger
   */
  public AxisProcChain getTriggerProcessing(XboxController.Axis axisId) {
    if (!((XboxController.Axis.kLeftTrigger == axisId)
        || (XboxController.Axis.kRightTrigger == axisId))) {
      throw new IndexOutOfBoundsException("axis ID specifies a stick, not a trigger");
    }

    return m_axisProcessorMap.get(axisId);
  }

  /**
   * Get the X axis value of the controller's left joystick.
   *
   * @return A value in the range [-1.0 to 1.0]
   */
  @Override
  public double getLeftX() {
    return processAxis(Axis.kLeftX);
  }

  /**
   * Get the Y axis value of the controller's left joystick.
   *
   * @return A value in the range [-1.0 to 1.0]
   */
  @Override
  public double getLeftY() {
    return processAxis(Axis.kLeftY);
  }

  /**
   * Get the X axis value of the controller's right joystick.
   *
   * @return A value in the range [-1.0 to 1.0]
   */
  @Override
  public double getRightX() {
    return processAxis(Axis.kRightX);
  }

  /**
   * Get the Y axis value of the controller's right joystick.
   *
   * @return A value in the range [-1.0 to 1.0]
   */
  @Override
  public double getRightY() {
    return processAxis(Axis.kRightY);
  }

  /**
   * Returns a value reflecting the pull on the controller's left trigger.
   *
   * @return A value in the range [0.0 to 1.0]
   */
  @Override
  public double getLeftTriggerAxis() {
    return processAxis(Axis.kLeftTrigger);
  }

  /**
   * Returns a value reflecting the pull on the controller's right trigger.
   *
   * @return A value in the range [0.0 to 1.0]
   */
  @Override
  public double getRightTriggerAxis() {
    return processAxis(Axis.kRightTrigger);
  }

  /** Returns the Trigger for a specified button on the POV pad */
  public Trigger povTrigger(POVAngle povAngle) {
    EventLoop eventLoop = CommandScheduler.getInstance().getDefaultButtonLoop();
    return new Trigger(eventLoop, () -> this.getPOV() == povAngle.angleDeg);
  }

  /** Return the value of a given axis after processing it through its chain */
  private double processAxis(XboxController.Axis axisId) {
    double axisValue = getRawAxis(axisId.value);
    return m_axisProcessorMap.get(axisId).process(axisValue);
  }
}
