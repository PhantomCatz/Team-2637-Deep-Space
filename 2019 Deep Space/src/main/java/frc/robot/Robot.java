package frc.robot;

import java.util.Enumeration;

import javax.sound.midi.SysexMessage;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

import frc.Autonomous.CatzDriveStraight;
import frc.Autonomous.CatzTurn;

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
  //private UDPServerThread server;

  private UsbCamera drvCamera;

  public static AHRS navx;

  private static XboxController xboxDrv;
  //Changed to public so we can see it in the thread KH
  public static XboxController xboxAux;

  private static final int XBOX_DRV_PORT = 0;
  private static final int XBOX_AUX_PORT = 1;

  private static final double MAX_POWER  = .75;

  private double heading;
  private double distance;

  private static final boolean INTAKE_ARM_OPEN   = true;
  private static final boolean INTAKE_ARM_CLOSED = false;

  private static final double INTAKE_WHEEL_SPEED  = 0.5;
  private static final double OUTTAKE_WHEEL_SPEED = 0.8;


  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() 
  {
    //vision packet recieving thread
    //server     = new UDPServerThread();

    drvCamera = CameraServer.getInstance().startAutomaticCapture();
    //drvCamera = new UsbCamera("Driver Camera", "/dev/video1");
    drvCamera.setFPS(24);

    arm        = new CatzArm();
    driveTrain = new CatzDriveTrain();
    intake     = new CatzIntake();
    lift       = new CatzLift();
    
    xboxDrv    = new XboxController(XBOX_DRV_PORT);
    xboxAux    = new XboxController(XBOX_AUX_PORT);
    

    //server.start();

    //grabbing PID values from SmartDashboard (testing purposes)
    SmartDashboard.putNumber("PIVOT_DEBUG_KP", CatzIntake.WRIST_DEBUG_KP);
    SmartDashboard.putNumber("PIVOT_DEBUG_KD", CatzIntake.WRIST_DEBUG_KD);
    SmartDashboard.putNumber("PIVOT_DEBUG_KA", CatzIntake.WRIST_DEBUG_KA);


    //arm.pivotPID();
    if(CatzConstants.USING_SOFT_LIMITS)
    {
      intake.wristPID();
    }
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
    CatzIntake.WRIST_DEBUG_KA = SmartDashboard.getNumber("PIVOT_DEBUG_KA", 0);
    CatzIntake.WRIST_DEBUG_KS = SmartDashboard.getNumber("PIVOT_DEBUG_KS", 0);
    CatzIntake.WRIST_DEBUG_KD = SmartDashboard.getNumber("PIVOT_DEBUG_KD", 0);
    CatzIntake.WRIST_DEBUG_KP = SmartDashboard.getNumber("PIVOT_DEBUG_KP", 0);

    SmartDashboard.putNumber("test",  CatzIntake.WRIST_DEBUG_KA +
                                      CatzIntake.WRIST_DEBUG_KS +
                                      CatzIntake.WRIST_DEBUG_KD +
                                      CatzIntake.WRIST_DEBUG_KP);
    /*

    SmartDashboard.putNumber("PIVOT: Encoder Value",  arm.armPivotEnc.getVoltage() * 72); //arm.getPivotAngle());
    SmartDashboard.putNumber("PIVOT: Encoder Voltage", arm.armPivotEnc.getVoltage());
    SmartDashboard.putNumber("PIVOT: Encoder Angle", arm.getPivotAngle());

    //SmartDashboard.putNumber("WRIST: Encoder Value",  intake.intakeWristEnc.getVoltage() * 72); // intake.getWristAngle());*/
    SmartDashboard.putNumber("WRIST: Encoder Voltage", intake.intakeWristEnc.getVoltage());
    SmartDashboard.putNumber("WRIST: Encoder Angle ", intake.getWristAngle());
    SmartDashboard.putNumber("Wrist Power", intake.getWristPower());


    /*
    SmartDashboard.putNumber("ARM : Encoder Value", arm.getArmExtensionEncoderCounts());
    SmartDashboard.putNumber("ARM : Encoder Distance", arm.getArmExtensionDistance());

    */SmartDashboard.putBoolean("Lift Bot Limit", lift.isLiftAtBottom());
    SmartDashboard.putNumber("Lift Power", lift.getLiftPower());
    SmartDashboard.putNumber("Wrist Target Angle",intake.getTargetAngle());
    /*SmartDashboard.putBoolean("Lift Top Limit", lift.isLiftAtTop());

    SmartDashboard.putNumber("Lift Counts", lift.getLiftCounts());
    SmartDashboard.putNumber("Lift Height", lift.getLiftHeight());

    SmartDashboard.putBoolean("ARM Retracted Limit", arm.isArmLimitRetractedActivated());

    Timer.delay(0.33);*/
  }

  @Override
  public void autonomousInit() 
  {
    CatzDriveTrain.arcadeDrive(0, 0);
    arm.turnPivot(0);  
    intake.getCargo(0);
    lift.lift(0);
  }
  /**
   * This function is called periodically during operator control.
   */

  @Override
  public void autonomousPeriodic()
  {
    if(xboxDrv.getBumper(Hand.kRight) == false)
    {
      CatzDriveTrain.arcadeDrive(xboxDrv.getY(Hand.kLeft), -xboxDrv.getX(Hand.kRight));
      lift.lift(0);
    }
    else
    {
      lift.lift(-xboxDrv.getY(Hand.kLeft));
      CatzDriveTrain.arcadeDrive(0, 0);
    }

    //moves arm pivot
    if(Math.abs(xboxAux.getY(Hand.kLeft)) < 0.1)
    {
      arm.turnPivot(0);
    }
    else
    {
      arm.turnPivot(xboxAux.getY(Hand.kLeft));
    }

    //extends retracts arm
    arm.extendArm(-xboxDrv.getTriggerAxis(Hand.kRight) + xboxDrv.getTriggerAxis(Hand.kLeft));

    //sets wrist to loading station position
    if(xboxAux.getXButton())
    {
      intake.setWristTargetAngle(-25);
    }
    /*
    //sets wrist to ground pickup position
    if(xboxAux.getAButton())
    {
      intake.setWristTargetAngle(0);
    }

    //sets pivot to loading station position
    if(xboxAux.getYButton())
    {
      arm.setPivotTargetAngle(52.2);
    }
    //sets pivot to ground pickup position
    if(xboxAux.getBButton())
    {
      arm.setPivotTargetAngle(45);
    }*/

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
       
    intake.getCargo(xboxAux.getTriggerAxis(Hand.kLeft) - xboxAux.getTriggerAxis(Hand.kRight));
  }
  @Override
  public void teleopInit() 
  {
    CatzDriveTrain.arcadeDrive(0, 0);
    arm.turnPivot(0);  
    intake.getCargo(0);
    lift.lift(0);  
  }

  @Override
  public void teleopPeriodic() 
  {
    //CatzDriveTrain.arcadeDrive(xboxDrv.getY(Hand.kLeft), -xboxDrv.getX(Hand.kRight));

    //runs drivetrain
    if(xboxDrv.getBumper(Hand.kRight) == false)
    {
      CatzDriveTrain.arcadeDrive(xboxDrv.getY(Hand.kLeft), -xboxDrv.getX(Hand.kRight));
      lift.lift(0);
    }
    else
    {
      lift.lift(-xboxDrv.getY(Hand.kLeft));
      CatzDriveTrain.arcadeDrive(0, 0);
    }

    //moves arm pivot
    if(Math.abs(xboxAux.getY(Hand.kLeft)) < 0.1)
    {
      arm.turnPivot(0);
    }
    else
    {
      arm.turnPivot(xboxAux.getY(Hand.kLeft));
    }

    //extends retracts arm
    arm.extendArm(-xboxDrv.getTriggerAxis(Hand.kRight) + xboxDrv.getTriggerAxis(Hand.kLeft));

    //sets wrist to loading station position
    if(xboxAux.getXButton())
    {
      intake.setWristTargetAngle(-25);
    }
    /*
    //sets wrist to ground pickup position
    if(xboxAux.getAButton())
    {
      intake.setWristTargetAngle(0);
    }

    //sets pivot to loading station position
    if(xboxAux.getYButton())
    {
      arm.setPivotTargetAngle(52.2);
    }
    //sets pivot to ground pickup position
    if(xboxAux.getBButton())
    {
      arm.setPivotTargetAngle(45);
    }*/

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
       
    intake.getCargo(xboxAux.getTriggerAxis(Hand.kLeft) - xboxAux.getTriggerAxis(Hand.kRight));
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() 
  {
  
  }
}
