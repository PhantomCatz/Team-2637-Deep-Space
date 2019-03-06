package frc.robot;

import java.util.Enumeration;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

import frc.Autonomous.CatzDriveStraight;
import frc.Autonomous.CatzTurn;
import frc.Vision.UDPServerThread;
import frc.Vision.SensorObjContainer;
import frc.Vision.SensorObject;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Mechanisms.CatzArm;
import frc.Mechanisms.CatzDriveTrain;
import frc.Mechanisms.CatzIntake;
import frc.Mechanisms.CatzLift;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot 
{
  private CatzArm         arm;
  private CatzDriveTrain  driveTrain;
  private CatzIntake      intake;
  private CatzLift        lift;
  private UDPServerThread server;

  //private UsbCamera drvCamera;

  public static AHRS navx;

  private static XboxController xboxDrv;
  private static XboxController xboxAux;

  private static final int XBOX_DRV_PORT = 0;
  private static final int XBOX_AUX_PORT = 1;

  private static final double MAX_POWER  = .75;

  private double heading;
  private double distance;

  private static DigitalInput testLimit;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() 
  {
    navx       = new AHRS(SPI.Port.kMXP,(byte)200);
    
    server     = new UDPServerThread();

    //drvCamera = CameraServer.getInstance().startAutomaticCapture();
    //drvCamera = new UsbCamera("Driver Camera", "/dev/video1");
    //drvCamera.setFPS(24);

    arm        = new CatzArm();
    driveTrain = new CatzDriveTrain();
    intake     = new CatzIntake();
    lift       = new CatzLift();
    
    xboxDrv    = new XboxController(XBOX_DRV_PORT);
    xboxAux    = new XboxController(XBOX_AUX_PORT);

    server.start();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() 
  {
    SmartDashboard.putNumber("PIVOT: Encoder Value",  arm.armPivotEnc.getVoltage() * 72); //arm.getPivotAngle());
    SmartDashboard.putNumber("PIVOT: Encoder Voltage", arm.armPivotEnc.getVoltage());

    SmartDashboard.putNumber("WRIST: Encoder Value",  intake.intakeWristEnc.getVoltage() * 72); // intake.getWristAngle());
    SmartDashboard.putNumber("WRIST: Encoder Voltage", intake.intakeWristEnc.getVoltage());
    //SmartDashboard.putNumber("WRIST: Encoder Angle ", intake.getWristAngle());

    SmartDashboard.putNumber("ARM : Encoder Value", arm.getArmExtensionEncoderCounts());
    SmartDashboard.putNumber("ARM : Encoder Distance", arm.getArmExtensionDistance());
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() 
  {

    //runs drivetrain
    CatzDriveTrain.arcadeDrive(xboxDrv.getY(Hand.kLeft), xboxDrv.getX(Hand.kRight));

    //runs lift
    if(xboxDrv.getBumper(Hand.kLeft))
    {
      lift.lift(CatzLift.LIFT_UP_MAX_POWER);
    }
    else if(xboxDrv.getBumper(Hand.kRight))
    {
      lift.lift(CatzLift.LIFT_DN_MAX_POWER);
    }
    else
    {
      lift.lift(0);
    }

    //moves arm pivot
    // 
    double pivotPower = xboxAux.getY(Hand.kLeft);  

    if (xboxAux.getTriggerAxis(Hand.kLeft) >= 0.25)
    {
      if (!arm.isLocked())
      {
        arm.lockPivot(pivotPower);
      }
    }
    else
    {
      arm.unlockPivot();
    }

    arm.turnPivot(pivotPower);

    //extends retracts arm
    arm.extendArm(xboxDrv.getTriggerAxis(Hand.kRight) - xboxDrv.getTriggerAxis(Hand.kLeft));

    // Runs intake wheels
    if(xboxAux.getAButton())
    {
      intake.getCargo(MAX_POWER);
    }
    else if(xboxAux.getYButton())
    {
      intake.releaseCargo(MAX_POWER);
    }
    else
    {
      intake.getCargo(0);
    }

    // Rotating the intake wrist
    intake.rotateWrist(xboxAux.getY(Hand.kRight));

    // eject hatch
    if(xboxAux.getBumper(Hand.kRight))
    {
      intake.hatchEject();
    }
    else if(xboxAux.getBumper(Hand.kLeft))
    {
      intake.hatchDeployed();
    }

    // open/close the intake
    if(xboxAux.getXButton())
    {
      intake.openCargoClamp();
      //arm.turnPivotToAngle(4, 90);

    }
    else if(xboxAux.getBButton())
    {
      intake.closeCargoClamp();
    }

    if(xboxAux.getStartButton())
    {
    }
   
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() 
  {
  
  }
}
