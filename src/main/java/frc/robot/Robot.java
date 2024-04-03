package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.core.util.CTREConfigs;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.auto.AutoCommands;

import edu.wpi.first.wpilibj.DataLogManager;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  public static CTREConfigs ctreConfigs;
  private final Field2d m_field = new Field2d();
  private SendableChooser<Command> autoChooser;
  private ShuffleboardTab tab = Shuffleboard.getTab("Odometry Data");
  private GenericEntry visionFilterEntry = tab.add("Vision Filter Overide", false).getEntry();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // TODO put auto chooser here. make sure to use the one from
    // robot/auto/selector/AutoModeSelector.java
    
    //Initialize AdvantageKit
    AdvantageKitInit();
    //Initialize logging  
    DataLogManager.start();


    //Autocommands
    OI.getInstance().registerCommands();
    AutoCommands.registerAutoCommands();
    ctreConfigs = new CTREConfigs();

    if(RobotMap.PrototypeMap.LIVE_WINDOW_ENABLED)
      enableLiveWindowInTest(true);

    SmartDashboard.putData("field", m_field);
    
    if(Config.Subsystems.DRIVETRAIN_ENABLED){
       //Zero Gyro
      Drivetrain.getInstance().zeroGyroYaw();
      PoseEstimator.getInstance().resetPoseEstimate(Drivetrain.getInstance().getPose()); 
    }

    //TODO: for now I'm using a try/catch as this will fail with the WPILib Simulation mode
    //TODO: Eventually we need to modify auto-chooser so it behaves gracefully in simulation mode
    try{
      autoChooser = AutoBuilder.buildAutoChooser();
      SmartDashboard.putData("Auto Chooser", autoChooser);
    } catch(Exception e) {
      DataLogManager.log("Autobuilder failed: " + e.getMessage());
    }
    //PoseEstimator.getInstance().setEstimatedPose(Drivetrain.getInstance().getPose());
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    if(Config.Subsystems.DRIVETRAIN_ENABLED) {
      m_field.getObject("Odometry Pose").setPose(Drivetrain.getInstance().getPose());
      m_field.getObject("Vision Pose").setPose(Vision.getInstance().visionBotPose());
      m_field.getObject("PoseEstimate Pose").setPose(PoseEstimator.getInstance().getPosition());
    }  
    if (Vision.getInstance().getNotePose2d() != null){
      m_field.getObject("Note Pose").setPose(Vision.getInstance().getNotePose2d());
      
    }

    if (visionFilterEntry.getBoolean(false)){
      m_field.getObject("Filtered Vision Pose").setPose(Vision.getInstance().getFilterVisionPose());
    }
    
    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
      m_field.getObject("target pose").setPose(pose);
    });

    PathPlannerLogging.setLogActivePathCallback((poses) -> {
      m_field.getObject("path").setPoses(poses);
    });
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    CommandScheduler.getInstance().cancelAll();

    var autonomousCommand = getAutonomousCommand();

      // Supplier<Pose2d> getTarget = () -> DriverStation.getAlliance().isPresent()
      // && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue
      // ? RobotMap.Coordinates.BLUE_SPEAKER
      // : RobotMap.Coordinates.RED_SPEAKER;

      if(autonomousCommand != null) {
        autonomousCommand.schedule();
        // if(Config.Subsystems.DRIVETRAIN_ENABLED && Config.Subsystems.PIVOT_ENABLED) {
        //   Pivot pivot = Pivot.getInstance();
        //   pivot.setDefaultCommand(pivot.updatePosition(
        //     () -> RobotMap.ShooterMap.SPEAKER_LOOKUP_TABLE.get(PoseEstimator.getInstance().getDistanceToPose(getTarget.get().getTranslation()))
        //       .getAngle()));
        // }
        // if(Config.Subsystems.DRIVETRAIN_ENABLED && Config.Subsystems.SHOOTER_ENABLED) {
        //   Shooter shooter = Shooter.getInstance();
        //   shooter.setDefaultCommand(shooter.setFlywheelVelocityCommand(
        //     () -> RobotMap.ShooterMap.SPEAKER_LOOKUP_TABLE.get(PoseEstimator.getInstance().getDistanceToPose(getTarget.get().getTranslation()))
        //       .getRPM()));
        // }
      }
    //Drivetrain.getInstance().setGyroYaw(180);
  }
    

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  public void AdvantageKitInit(){
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }
   
    // Set up data receivers & replay source
    switch (Config.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // See http://bit.ly/3YIzFZ6 for more information on timestamps in AdvantageKit.
    // Logger.disableDeterministicTimestamps()

    // Start AdvantageKit logger
    Logger.start();
  }
}

