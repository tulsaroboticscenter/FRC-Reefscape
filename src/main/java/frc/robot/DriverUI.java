package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.networktables.GenericEntry;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class DriverUI extends Command {

  // Make sure there is only one instance of this.
  private static DriverUI m_instance;
  public static synchronized void init() {
    if (m_instance == null) {
      m_instance = new DriverUI();
    }
  }
  public static DriverUI instance() { 
    init();
    return m_instance; 
  }

  private ShuffleboardTab m_driverTab;
  private SendableChooser<Command> m_autoChooser;
  boolean m_isCompetition;

  // For testing
  // private SendableChooser<Command> testingChooser;

  private DriverUI(){

    m_isCompetition = false;

    m_driverTab = Shuffleboard.getTab("Driver");
    Shuffleboard.selectTab("Driver");

    m_autoChooser = new SendableChooser<>();

    CameraInit();
    AutosInit();

    // Testing only
    // testingChooser.addDefaultOption("Defualt Option", new Command());
    // testingChooser.addOption("First Option", new InstantCommand());

    // m_driverTab.add("Testing Chooser", testingChooser)
    //            .withWidget(BuiltInWidgets.kComboBoxChooser)
    //            .withSize(1, 1)
    //            .withPosition(5, 3);
  };

  public void setCompetitionMode(boolean isCompetition) { m_isCompetition = isCompetition; }

  private void CameraInit() {
    CameraServer.startAutomaticCapture();
    m_driverTab.addCamera("Robot View", "front", "http://roboRIO-TEAM-frc.local:1181")
               .withPosition(0, 0)
               .withSize(5, 5);
  }

  private void AutosInit() {

    //FIXME This does not work with Shuffleboard.
    // Build an auto chooser. This will use Commands.none() as the default option.
    // As an example, this will only show autos that start with "comp" while at
    // competition as defined by the programmer
    m_autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
        (stream) -> m_isCompetition
        ? stream.filter(auto -> auto.getName().startsWith("Comp"))
        : stream
    );

    // m_driverTab.putData("Auto Chooser", m_autoChooser);
    m_driverTab.add("Autonomous Chooser", m_autoChooser)
               .withWidget(BuiltInWidgets.kComboBoxChooser)
               .withPosition(5, 0)
               .withSize(2, 1);
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
}