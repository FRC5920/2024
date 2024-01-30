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
package frc.lib.dashboard;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.networktables.LoggedDashboardInput;

////////////////////////////////////////////////////////////////////////////////////////////////////
/** LoggedSendableChooser provides a chooser whose value is logged */
public class LoggedSendableChooser<V> implements LoggedDashboardInput {
  /**
   * The m_key of the chooser for dashboard and logs: "/SmartDashboard/{m_key}" for NetworkTables
   * and "/DashboardInputs/{m_key}" when logged.
   */
  private final String m_key;

  /** Holds the name displayed for the selected value in the chooser */
  /** Map of item names in the chooser to associated values */
  private Map<String, V> m_choiceMap = new HashMap<>();

  /** The name of the selected item in the chooser */
  private String m_selectedChoiceName = null;

  /**
   * Chooser object displayed on the dashboard. Values in this chooser are the same as the option
   * names. The 'value' in the chooser sent to the Dashboard is be mapped through to the 'real'
   * registered value using the map m_choices.
   */
  private SendableChooser<String> m_chooser = new SendableChooser<>();

  /** Used to detect when the selected choice has been changed */
  private ChangeDetector<String> m_changeDetector;

  /** Internal class used to log which item is selected in the chooser */
  private final LoggableInputs m_inputs =
      new LoggableInputs() {
        /** Called to write the title of choosers selected value to the log */
        public void toLog(LogTable table) {
          table.put(m_key, m_selectedChoiceName);
        }

        /** Called during playback to read the choosers selected value from the log */
        public void fromLog(LogTable table) {
          m_selectedChoiceName = table.get(m_key, m_selectedChoiceName);
        }
      };

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Creates an empty chooser that will be logged with a given key
   *
   * @param key The key for the chooser, published to "/SmartDashboard/{key}" for NetworkTables or
   *     "/DashboardInputs/{key}" when logged
   */
  public LoggedSendableChooser(String key) {
    m_key = key;
    SmartDashboard.putData(key, m_chooser);
    m_changeDetector = new ChangeDetector<String>(() -> m_chooser.getSelected());
    periodic();
    Logger.registerDashboardInput(this);

    // When the selected item in the chooser changes, store it to m_selectedChoiceName
    m_chooser.onChange(newValue -> m_selectedChoiceName = newValue);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Creates a LoggedSendableChooser with options copied from a given SendableChooser object
   *
   * @details This constructor copies the options from a given SendableChooser. Updates to the
   *     original SendableChooser will not affect the constructed object.
   * @param key The key for the chooser, published to "/SmartDashboard/{key}" for NetworkTables or
   *     "/DashboardInputs/{key}" when logged
   */
  @SuppressWarnings("unchecked")
  public LoggedSendableChooser(String key, SendableChooser<V> chooser) {
    this(key);
    m_changeDetector = new ChangeDetector<String>(() -> m_chooser.getSelected());

    // Get access to the options in the given chooser
    Map<String, V> options = new HashMap<>();
    try {
      Field mapField = SendableChooser.class.getDeclaredField("m_map");
      mapField.setAccessible(true);
      options = (Map<String, V>) mapField.get(chooser);
    } catch (NoSuchFieldException
        | SecurityException
        | IllegalArgumentException
        | IllegalAccessException e) {
      e.printStackTrace();
    }

    // Get default option
    String defaultString = "";
    try {
      Field defaultField = SendableChooser.class.getDeclaredField("m_defaultChoice");
      defaultField.setAccessible(true);
      defaultString = (String) defaultField.get(chooser);
    } catch (NoSuchFieldException
        | SecurityException
        | IllegalArgumentException
        | IllegalAccessException e) {
      e.printStackTrace();
    }

    // Add options
    for (String optionKey : options.keySet()) {
      if (optionKey.equals(defaultString)) {
        addDefaultOption(optionKey, options.get(optionKey));
      } else {
        addOption(optionKey, options.get(optionKey));
      }
    }

    // When the selected item in the chooser changes, store it to m_selectedChoiceName
    m_chooser.onChange(newValue -> m_selectedChoiceName = newValue);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Adds a new option to the chooser
   *
   * @param name Name displayed in the chooser for the new option
   * @param value Value associated with the choice
   */
  public void addOption(String name, V value) {
    m_chooser.addOption(name, name);
    m_choiceMap.put(name, value);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Adds a new option to the chooser
   *
   * @param names Names of the options to load into the chooser
   * @param values Values corresponding to the elements in names
   * @param defaultIndex Index in names representing the default choice
   */
  public void addOptions(String[] names, V[] values, int defaultIndex) {
    for (int idx = 0; idx < names.length; ++idx) {
      String name = names[idx];
      V value = (idx < values.length) ? values[idx] : null;
      if (name == null) {
        throw new IllegalArgumentException("choice name cannot be null");
      } else if (value == null) {
        throw new IllegalArgumentException("choice value cannot be null");
      } else {
        this.addOption(name, value);
      }
    }

    // Set the initial option
    m_chooser.setDefaultOption(names[defaultIndex], names[defaultIndex]);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Adds an option to the chooser and makes it the default choice
   *
   * @param name Name displayed in the chooser for the new option
   * @param value Value associated with the choice
   */
  public void addDefaultOption(String name, V value) {
    m_chooser.setDefaultOption(name, name);
    m_choiceMap.put(name, value);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Returns the value of the selected option. If no option has been selected, the default selection
   * is returned. If no option is selected and no default has been assigned, {@code null} is
   * returned.
   */
  public V getSelectedValue() {
    return m_choiceMap.get(m_selectedChoiceName);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Returns the name of the selected option. If no option has been selected, the default selection
   * is returned. If no option is selected and no default has been assigned, {@code null} is
   * returned.
   */
  public String getChoiceName() {
    return m_selectedChoiceName;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Returns the internal sendable chooser object, for use when setting up dashboard layouts. Do not
   * read data directly from the returned object.
   */
  public SendableChooser<String> getSendableChooser() {
    return m_chooser;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  public boolean hasChanged() {
    return m_changeDetector.hasChanged();
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /** This method must be called periodically to log the selected item in the chooser */
  public void periodic() {
    if (!Logger.hasReplaySource()) {
      // If we're logging
      m_selectedChoiceName = m_chooser.getSelected();
    }
    Logger.processInputs(prefix, m_inputs);
  }
}
