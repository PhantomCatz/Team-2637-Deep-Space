package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SerialPort;
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
  public static CatzArm         arm;
  public static CatzDriveTrain  driveTrain;
  public static CatzIntake      intake;
  public static CatzLift        lift;
  //private UDPServerThread server;

  private UsbCamera drvCameraFrt;
  private UsbCamera drvCameraBck;

  public static AHRS navx;

  private static XboxController xboxDrv;
  private static XboxController xboxAux;

  private static final int XBOX_DRV_PORT = 0;
  private static final int XBOX_AUX_PORT = 1;

  private static final double MAX_POWER  = .75;

  private double heading;
  private double distance;

  private static final boolean INTAKE_ARM_OPEN   = true;
  private static final boolean INTAKE_ARM_CLOSED = false;

  double intakePow;
  final double INTAKE_POWER = 0.7;
  final double HOLD_BALL_POWER = 0.2;

  private boolean wristPIDState = false;
  private boolean armPIDState = false;
  public static boolean armExtPIDState = false;

  private static final double AUX_JOYSTICK_LT_DEADBAND_THRESHOLD = 0.15;
  private static final double AUX_JOYSTICK_RT_DEADBAND_THRESHOLD = 0.1;
  private static final double DRV_JOYSTICK_LT_DEADBAND_THRESHOLD = 0.1;
  private static final double DRV_JOYSTICK_RT_DEADBAND_THRESHOLD = 0.1;


  private static final double INTAKE_WHEEL_SPEED  = 0.45;
  private static final double OUTTAKE_WHEEL_SPEED = 1.0;

  private static double cameraResolutionWidth = 320;
  private static double cameraResolutionHeight = 240;
  private static double cameraFPS = 15;

  double pThr = 0;

  private boolean ejected = false;
  private int ejectcount = 0;
  
  private boolean iGrabbedABall = false; 

  private static final double DPAD_UP = 0;
  private static final double DPAD_DN = 180;
  private static final double DPAD_LT = 270;
  private static final double DPAD_RT = 90;

  @Override
  public void robotInit() 
  {

    /*--------------------------------------------------------------------------
    *  Initialize Drive Cameras
    *-------------------------------------------------------------------------*/
    drvCameraFrt = CameraServer.getInstance().startAutomaticCapture();
    drvCameraFrt.setFPS(15);
    drvCameraFrt.setResolution(320, 240);
    drvCameraFrt.setPixelFormat(PixelFormat.kMJPEG);

    drvCameraBck = CameraServer.getInstance().startAutomaticCapture();
    drvCameraBck.setFPS(15);
    drvCameraBck.setResolution(320, 240);
    drvCameraBck.setPixelFormat(PixelFormat.kMJPEG);

    /*--------------------------------------------------------------------------
    *  Initialize Mechanisms & Drive Controllers
    *-------------------------------------------------------------------------*/
    arm        = new CatzArm();
    driveTrain = new CatzDriveTrain();
    intake     = new CatzIntake();
    lift       = new CatzLift();
  
    xboxDrv    = new XboxController(XBOX_DRV_PORT);
    xboxAux    = new XboxController(XBOX_AUX_PORT);
    
    /*--------------------------------------------------------------------------
    *  Initialize Vision
    *-------------------------------------------------------------------------*/
    //server     = new UDPServerThread();       //vision packet recieving thread

    //navx = new AHRS(Port.kMXP, (byte)200);

    //server.start();

    /*--------------------------------------------------------------------------
    *  Engage Mechanical Brakes, Set Target Angles to Current Angles & Start
    *  PID Threads
    *-------------------------------------------------------------------------*/
    arm.engageArmExtensionBrake();
    //arm.engageArmPivotBrake();
    
    intake.setWristTargetAngle(intake.getWristAngle());
    arm.setArmTargetExt(CatzConstants.INVALID_EXT);
    arm.setPivotTargetAngle(arm.getPivotAngle());

    intake.hatchMechClosed();

    intake.wristPID();
    arm.ArmPID();
    arm.pivotPID();
  }


  @Override
  public void robotPeriodic() 
  {

    drvCameraFrt.setResolution((int)cameraResolutionWidth, (int)cameraResolutionHeight);
    drvCameraFrt.setFPS((int)cameraFPS);

    drvCameraBck.setResolution((int)cameraResolutionWidth, (int)cameraResolutionHeight);
    drvCameraBck.setFPS((int)cameraFPS);

    SmartDashboard.putNumber("pivot power", arm.getPivotPower());
    SmartDashboard.putNumber("Pivot target angle", arm.getPivotTargetAngle());
    SmartDashboard.putNumber("PIVOT: Encoder Voltage", arm.armPivotEnc.getVoltage());
    SmartDashboard.putNumber("PIVOT: Encoder Angle", arm.getPivotAngle());
    //SmartDashboard.putNumber("PIVOT: Encoder Value", arm.armPivotEnc.getAverageVoltage()*72);

    SmartDashboard.putNumber("WRIST: Encoder Voltage", intake.intakeWristEnc.getVoltage());
    SmartDashboard.putNumber("WRIST: Encoder Angle ",  intake.getWristAngle());
    SmartDashboard.putNumber("Wrist Power", intake.getWristPower()); 

    SmartDashboard.putNumber("ARM Encoder Counts", arm.getArmExtensionEncoderCounts());
    SmartDashboard.putNumber("Target Ext", arm.getArmTargetExt());
    SmartDashboard.putNumber("ARM Ext distance", arm.getArmExtensionDistance());
    SmartDashboard.putNumber("Arm power", arm.getExtensionPower());
    SmartDashboard.putBoolean("arm ext limit switch", arm.isArmLimitRetractedActivated());

    SmartDashboard.putBoolean("Lift Bot Limit", lift.isLiftAtBottom());
    SmartDashboard.putNumber("Lift Power", lift.getLiftPower());
    SmartDashboard.putNumber("lift encoder counts", lift.getLiftCounts());
    SmartDashboard.putNumber("Lift Height", lift.getLiftHeight());

    SmartDashboard.putNumber("Wrist Target Angle", intake.getTargetAngle());
    SmartDashboard.putBoolean("wrist pid state", wristPIDState);

    SmartDashboard.putBoolean("target hit", arm.getArmTargetHit());

    //SmartDashboard.putNumber("arm current", arm.getCurrent());

    SmartDashboard.putBoolean("arm ext pid state", armExtPIDState);
    // SmartDashboard.putBoolean("Controls Arm ext break", false);
    /*if(SmartDashboard.getBoolean("Controls Arm ext break", true))
    {
      arm.engageArmExtensionBrake();
    } */

    
    SmartDashboard.putNumber("Aux Left  Stick Y", xboxAux.getY(Hand.kLeft));
    SmartDashboard.putNumber("Aux Right Stick Y", xboxAux.getY(Hand.kRight));
    SmartDashboard.putNumber("Drv Left  Stick Y", xboxDrv.getY(Hand.kLeft));
    SmartDashboard.putNumber("Drv Right Stick X", xboxDrv.getX(Hand.kRight));

    if(arm.isArmLimitRetractedActivated())
    {
      arm.resetArmExtensionEncoderCounts();
    }

    if(lift.isLiftAtBottom())
    {
      lift.resetLiftEnc();
    }
  }

  @Override
  public void autonomousInit() 
  {
    initializeRobotPositions();
  }

  @Override
  public void autonomousPeriodic()
  {
    robotControls();
  }

  @Override
  public void teleopInit() 
  {
    initializeRobotPositions();
  
  }

  @Override
  public void teleopPeriodic() 
  {
    robotControls();
    
  }


  private void initializeRobotPositions()
  {
    arm.setPivotTargetAngle(arm.getPivotAngle());
    arm.setArmTargetExt(arm.getArmExtensionDistance());
    intake.setWristTargetAngle(intake.getWristAngle());

    driveTrain.shiftToDriveTrain();
    //intake.hatchDeployed();
  }

  private void robotControls()
  { 
    
    /**
     *  Drivetrain and lift controls
     * 
     */
    if(xboxDrv.getBumper(Hand.kRight) == false && driveTrain.getDriveTrainSolenoidState() == DoubleSolenoid.Value.kForward)
    {
      CatzDriveTrain.arcadeDrive(xboxDrv.getY(Hand.kLeft), -xboxDrv.getX(Hand.kRight));
      lift.lift(0);
    }
    else if(xboxDrv.getBumper(Hand.kRight) == true)
    {
      lift.lift(xboxDrv.getY(Hand.kLeft));
      CatzDriveTrain.arcadeDrive(0, 0);
    }
    else
    {
      CatzDriveTrain.arcadeDrive(xboxDrv.getY(Hand.kLeft), 0.0);
    }

    /**
     *  Drivetrain shifter
     */
    if(xboxDrv.getBackButton())
    {
      driveTrain.shiftToClimber();
    }
    else if(xboxDrv.getStartButton())
    {
      driveTrain.shiftToDriveTrain();
    }

    /**
     * 
     *  Pivot Movement
     * 
     */
    pThr = xboxAux.getY(Hand.kLeft);
    if(Math.abs(pThr) > AUX_JOYSTICK_LT_DEADBAND_THRESHOLD)
    {
      //pause pivot thread and rotate pivot
      armPIDState = false;
      arm.setPivotTargetAngle(CatzConstants.INVALID_ANGLE);
      arm.turnPivot(pThr);
    }
    else 
    {
      if(armPIDState == false)
      {
        arm.setPivotTargetAngle(arm.getPivotAngle());
        armPIDState = true;
      } 
    }


     /**
      * 
      * Arm Movement - MANUAL CONTROL
      *
      */
     double armExtPower =  xboxAux.getTriggerAxis(Hand.kRight) - xboxAux.getTriggerAxis(Hand.kLeft);
     if(Math.abs(armExtPower) > 0.1)
     {
       /*-----------------------------------------------------------------------
       *  Out of Deadband - Manual Control
       *----------------------------------------------------------------------*/
       arm.setArmTargetExt(CatzConstants.INVALID_EXT);
       arm.extendArm(armExtPower);
       armExtPIDState = false;  
     }
     else
     {
       /*-----------------------------------------------------------------------
       *  In Deadband - Hold Position
       *----------------------------------------------------------------------*/
       if(armExtPIDState == false)
       {
         // only stop arm if pid thread is not running 
          arm.holdArm();
       }
     }

    /**
     *  Drv Controller - Presets
     */
    if(xboxDrv.getPOV() == DPAD_DN)
    {
      CatzPreSets.cargoBayReversed();
    }
     
    if(xboxDrv.getBumper(Hand.kLeft))
    {
      CatzPreSets.transport();
    }

    /**
     * 
     *  Aux Controller - Hatch Positions Presets
     * 
     */
    if(xboxAux.getStartButton())
    {
      CatzPreSets.hatchPickUp();
    }

    if(xboxAux.getPOV() == DPAD_UP)
    {
      CatzPreSets.lvl3RocketHatch();
    }
    else if(xboxAux.getPOV() == DPAD_LT)
    {
      CatzPreSets.lvl2RocketHatch();
    }
    else if(xboxAux.getPOV() == DPAD_DN)
    {
      CatzPreSets.lvl1RocketHatch();
    }

    /**
     * 
     * Aux Controller - Cargo Positions Presets
     * 
     */
    if(xboxAux.getYButton())
    {
      CatzPreSets.lvl3RocketBall();
    }
    else if(xboxAux.getAButton())
    {
      CatzPreSets.cargoBayBall();
    }
    else if(xboxAux.getBButton())
    {
      CatzPreSets.lvl2RocketBall();
    }

    if(xboxAux.getXButton())
    {
      CatzPreSets.cargoPickUp();
    }

    if(xboxAux.getPOV() == DPAD_RT)
    {
      CatzPreSets.cargoBayReversed();
    }


    /**
     *  Wrist Movement
     */
    if(Math.abs(xboxAux.getY(Hand.kRight)) > AUX_JOYSTICK_RT_DEADBAND_THRESHOLD)
    {
      //pause wrist thread and rotate wrist
      intake.setWristTargetAngle(CatzConstants.INVALID_ANGLE);
      wristPIDState = false;
      intake.rotateWrist(xboxAux.getY(Hand.kRight));
      
    }
    else if(wristPIDState == false && Math.abs(xboxAux.getY(Hand.kRight)) <= AUX_JOYSTICK_RT_DEADBAND_THRESHOLD)
    {
      intake.setWristTargetAngle(intake.getWristAngle());
      wristPIDState = true;
    }


    /**
     * 
     *  Intake Wheels
     * 
     */
    if(intake.isBumpSwitchPressed() == true) 
    {
      intake.getCargo(0.2);
      iGrabbedABall = true;
    }
    
    if(xboxAux.getBumper(Hand.kLeft))
    {
      intake.getCargo(INTAKE_POWER);
    }
    else if(xboxAux.getBumper(Hand.kRight))
    {
      intake.releaseCargo(INTAKE_POWER); 
      iGrabbedABall = false;
    }
    else
    {
      if(iGrabbedABall == true)
      {
        intake.getCargo(0.3);
      }
      else
      {
        intake.getCargo(0.0);
      }
    }

    /**
     *  Hatch Mech Control
     * 
     */
    if(xboxAux.getBackButtonPressed())
    {
      //Works as a toggle
      if(intake.getHatchState() == DoubleSolenoid.Value.kForward)
      {
       intake.hatchMechClosed();
      }
      else
      {
        intake.hatchMechOpen();
      }
    }
  }
}

