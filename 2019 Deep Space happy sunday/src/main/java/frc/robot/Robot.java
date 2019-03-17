package frc.robot;

import java.util.Enumeration;

import javax.sound.midi.SysexMessage;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
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

  private static final double INTAKE_WHEEL_SPEED  = 0.45;
  private static final double OUTTAKE_WHEEL_SPEED = 1.0;

  private static double cameraResolutionWidth = 320;
  private static double cameraResolutionHeight = 240;
  private static double cameraFPS = 15;

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
    drvCamera.setFPS(15);
    drvCamera.setResolution(320, 240);
    //drvCamera.setPixelFormat(PixelFormat.kYUYV);
    //drvCamera.setPixelFormat(PixelFormat.kGray);
    drvCamera.setPixelFormat(PixelFormat.kMJPEG);

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

    SmartDashboard.putNumber("Camera_Resolution_Width", cameraResolutionWidth);
    SmartDashboard.putNumber("Camera_Resolution_Height", cameraResolutionHeight);
    SmartDashboard.putNumber("Camera_FPS", cameraFPS);

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

    

    cameraResolutionWidth = SmartDashboard.getNumber("Camera_Resolution_Width", 1);
    cameraResolutionHeight = SmartDashboard.getNumber("Camera_Resolution_Height", 1);
    cameraFPS = SmartDashboard.getNumber("Camera_FPS", 1);

    drvCamera.setResolution((int)cameraResolutionWidth, (int)cameraResolutionHeight);
    drvCamera.setFPS((int)cameraFPS);


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
    driveTrain.arcadeDrive(0, 0);
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

    //runs lift
    /*if(xboxDrv.getBumper(Hand.kLeft))
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
    }*/

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
    //sets wrist to ground pickup position
    /*if(xboxAux.getAButton())
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
    System.out.println(xboxAux.getY(Hand.kRight));
    intake.rotateWrist(xboxAux.getY(Hand.kRight));

    // eject hatch
    if(xboxAux.getBumper(Hand.kRight))
    {
      //intake.hatchEject();
      intake.closeCargoClamp();
    }
    else if(xboxAux.getBumper(Hand.kLeft))
    {
     // intake.hatchDeployed();
     intake.releaseCargo(0.0);
    }

    if(xboxAux.getTriggerAxis(Hand.kLeft) == 1)
    {
      intake.getCargo(INTAKE_WHEEL_SPEED);
    }

    if(xboxAux.getTriggerAxis(Hand.kLeft)==0 && xboxAux.getTriggerAxis(Hand.kRight) == 0)
    {
      intake.getCargo(0.12);
    }

    

    if(xboxAux.getTriggerAxis(Hand.kRight) == 1)
    {
      intake.releaseCargo(INTAKE_WHEEL_SPEED);
    }
    
    if(xboxAux.getBackButton())
    {
      intake.getCargo(0.0);
    }

    /***** 
    // open/close the intake & runs intake wheels
    if(xboxAux.getTriggerAxis(Hand.kLeft) == 1)
    {
      if(intake.isIntakeOpen() == INTAKE_ARM_CLOSED)
      {
        intake.openCargoClamp();
      }
      else //intake is open
      {
        if(intake.getIntakePower() <= 0.0)
        {      
          intake.getCargo(INTAKE_WHEEL_SPEED);
        }
      }
    }
    else if(xboxAux.getTriggerAxis(Hand.kRight) == 1)
    {
      if(intake.isIntakeOpen() == INTAKE_ARM_OPEN)
      {
        intake.closeCargoClamp();

        Timer.delay(0.2);

        intake.getCargo(0.1);
      }
      else //intake is closed
      {
        if(intake.getIntakePower() >= 0.0)
        {
          intake.releaseCargo(INTAKE_WHEEL_SPEED);
        }
        else
        {
          intake.releaseCargo(0);
        }
      }
    }

    ***/
   
    if(xboxAux.getYButton())
    {
     intake.releaseCargo(INTAKE_WHEEL_SPEED);
    }
    else if(xboxAux.getBButton())
    {
      intake.getCargo(INTAKE_WHEEL_SPEED);
    }

    if(xboxAux.getBackButton())
    {
  
    }
  }
  @Override
  public void teleopInit() 
  {
    driveTrain.arcadeDrive(0, 0);
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

    //runs lift
    /*if(xboxDrv.getBumper(Hand.kLeft))
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
    }*/

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
    }/*
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

    // open/close the intake & runs intake wheels
    if(xboxAux.getTriggerAxis(Hand.kLeft) == 1)
    {
      if(intake.isIntakeOpen() == INTAKE_ARM_CLOSED)
      {
        intake.openCargoClamp();
      }
      else //intake is open
      {
        if(intake.getIntakePower() <= 0.0)
        {      
          intake.getCargo(INTAKE_WHEEL_SPEED);
        }
      }
    }
    else if(xboxAux.getTriggerAxis(Hand.kRight) == 1)
    {
      if(intake.isIntakeOpen() == INTAKE_ARM_OPEN)
      {
        intake.closeCargoClamp();

        Timer.delay(0.2);

        intake.getCargo(0.1);
      }
      else //intake is closed
      {
        if(intake.getIntakePower() >= 0.0)
        {
          intake.releaseCargo(INTAKE_WHEEL_SPEED);
        }
        else
        {
          intake.releaseCargo(0);
        }
      }
    }
   
    if(xboxAux.getYButton())
    {
      intake.releaseCargo(INTAKE_WHEEL_SPEED);
    }
    else if(xboxAux.getBButton())
    {
      intake.getCargo(INTAKE_WHEEL_SPEED);
    }

    else if(xboxAux.getBumper(Hand.kLeft))
    {
     // intake.hatchDeployed();
     intake.releaseCargo(0.0);
    }

    if(xboxAux.getTriggerAxis(Hand.kLeft) == 1)
    {
      intake.getCargo(INTAKE_WHEEL_SPEED);
    }

    if(xboxAux.getTriggerAxis(Hand.kLeft)==0 && xboxAux.getTriggerAxis(Hand.kRight) == 0)
    {
      intake.getCargo(0.12);
    }

    

    if(xboxAux.getTriggerAxis(Hand.kRight) == 1)
    {
      intake.releaseCargo(INTAKE_WHEEL_SPEED);
    }
    
    if(xboxAux.getBackButton())
    {
      intake.getCargo(0.0);
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
