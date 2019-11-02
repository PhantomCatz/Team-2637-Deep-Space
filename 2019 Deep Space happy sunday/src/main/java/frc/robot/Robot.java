package frc.robot;

import java.io.FileOutputStream;
import java.io.OutputStream;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Autonomous.CatzDriveStraight;
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

  double intakePow;
  final double INTAKE_POWER = 0.7;
  final double HOLD_BALL_POWER = 0.3;

  private boolean wristPIDState = false;
  private boolean armPIDState = false;
  public static boolean armExtPIDState = false;

  private static final double AUX_JOYSTICK_LT_DEADBAND_THRESHOLD = 0.2;
  private static final double AUX_JOYSTICK_RT_DEADBAND_THRESHOLD = 0.1;
  private static final double DRV_JOYSTICK_LT_DEADBAND_THRESHOLD = 0.1;
  private static final double DRV_JOYSTICK_RT_DEADBAND_THRESHOLD = 0.1;


  private static final double INTAKE_WHEEL_SPEED  = 0.45;
  private static final double OUTTAKE_WHEEL_SPEED = 1.0;

  private static int cameraResolutionWidth = 320;
  private static int cameraResolutionHeight = 240;
  private static int cameraFPS = 15;

  double pivotPower = 0;
  
  private boolean iGrabbedABall = false; 

  private static final double DPAD_UP = 0;
  private static final double DPAD_DN = 180;
  private static final double DPAD_LT = 270;
  private static final double DPAD_RT = 90;

  private boolean PIDStopped = false;

  private static double scalar = 1.0;

  private static PowerDistributionPanel pdp;

  private static String[] canBusWarningMessages = new String[14];
  private static boolean[] PDPCurrent = new boolean[14];
  private static String[] motorWarningMessages = new String[14];

  private static Timer warningMessageTimer;

  private static Timer velocityTimer;

  private static Timer pdpCurrentTimer;

  private OutputStream output;



  @Override
  public void robotInit() 
  {
    pdpCurrentTimer = new Timer();
    /*--------------------------------------------------------------------------
    *  Initialize Drive Cameras
    *-------------------------------------------------------------------------*/
    drvCameraFrt = CameraServer.getInstance().startAutomaticCapture();
    drvCameraFrt.setFPS(cameraFPS);
    drvCameraFrt.setResolution(cameraResolutionWidth, cameraResolutionHeight);
    drvCameraFrt.setPixelFormat(PixelFormat.kMJPEG);

    /*drvCameraBck = CameraServer.getInstance().startAutomaticCapture();
    drvCameraBck.setFPS(cameraFPS);
    drvCameraBck.setResolution(cameraResolutionWidth, cameraResolutionHeight);
    drvCameraBck.setPixelFormat(PixelFormat.kMJPEG);*/

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
    *  Initialize Robot Config
    *-------------------------------------------------------------------------*/
   

    /*--------------------------------------------------------------------------
    *  Engage Mechanical Brakes, Set Target Angles to Current Angles & Start
    *  PID Threads
    *-------------------------------------------------------------------------*/
    arm.engageArmExtensionBrake();
    //arm.engageArmPivotBrake();
    
    intake.setWristTargetAngle(intake.getWristAngle());
    arm.setArmTargetExt(arm.getArmExtensionDistance());
    arm.setPivotTargetAngle(arm.getPivotAngle());
    
    arm.pivotPID();
    Timer.delay(0.1);

    intake.wristPID();
    Timer.delay(0.1);

    arm.ArmPID();
    
    //pdp = new PowerDistributionPanel(0);

    warningMessageTimer = new Timer();
    

  }



  @Override
  public void robotPeriodic() 
  {
    //System.out.println(driveTrain.getDriveTrainEncoderDistance() + ", " + warningMessageTimer.get());

    SmartDashboard.putBoolean("pivot pid state", armPIDState);

    SmartDashboard.putNumber("pivot power", arm.getPivotPower());
    SmartDashboard.putNumber("Pivot target angle", arm.getPivotTargetAngle());
    SmartDashboard.putNumber("PIVOT: Encoder Voltage", arm.getPivotEncVoltage());
    SmartDashboard.putNumber("PIVOT: Encoder Angle", arm.getPivotAngle());

    SmartDashboard.putNumber("WRIST: Encoder Voltage", intake.intakeWristEnc.getVoltage());
    SmartDashboard.putNumber("WRIST: Encoder Angle ",  intake.getWristAngle());
    /*SmartDashboard.putNumber("Wrist kPPow", intake.kPPow); 
    SmartDashboard.putNumber("Wrist kIPoww", intake.kIPow); 
    SmartDashboard.putNumber("Wrist kDPow", intake.kDPow); */
    SmartDashboard.putNumber("Wrist Power", intake.getWristPower()); 
    
    SmartDashboard.putBoolean("bump switch", intake.isBumpSwitchPressed());

    SmartDashboard.putNumber("ARM Encoder Counts", arm.getArmExtensionEncoderCounts());
    SmartDashboard.putNumber("Target Ext", arm.getArmTargetExt());
    SmartDashboard.putNumber("ARM Ext distance", arm.getArmExtensionDistance());
    /*SmartDashboard.putNumber("Arm P power", arm.armPPow);
    SmartDashboard.putNumber("Arm I power", arm.armIPow);
    SmartDashboard.putNumber("Arm D power", arm.armDPow);
    SmartDashboard.putNumber("Arm A power", arm.armAPow);*/

    SmartDashboard.putNumber("Arm power", arm.getExtensionPower());
    SmartDashboard.putBoolean("arm ext limit switch", arm.isArmLimitRetractedActivated());

    SmartDashboard.putBoolean("Lift Bot Limit", lift.isLiftAtBottom());
    SmartDashboard.putNumber("Lift Power", lift.getLiftPower());
    SmartDashboard.putNumber("lift encoder counts", lift.getLiftCounts());
    SmartDashboard.putNumber("Lift Height", lift.getLiftHeight());

    SmartDashboard.putNumber("Wrist Target Angle", intake.getTargetAngle());
    SmartDashboard.putBoolean("wrist pid state", wristPIDState);

    SmartDashboard.putBoolean("target hit", arm.getArmTargetHit());

    SmartDashboard.putNumber("arm current", arm.getArmMtrCtrlACurrent());

    SmartDashboard.putBoolean("arm ext pid state", armExtPIDState);

    SmartDashboard.putNumber("wrist power", intake.getWristPower());
    SmartDashboard.putNumber("roller power", intake.getIntakePower());
    //if(SmartDashboard.getBoolean("Controls Arm ext break", true))
    //{
    //  arm.engageArmExtensionBrake();
    //} 

    //Use to calibrate deadband value
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

    //System.out.println(arm.getPivotPower() + ", " + arm.getArmExtPower() + ", " + intake.getWristPower());
    
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
    warningMessageTimer.start();

    driveTrain.retractClimberBack();
    driveTrain.retractClimberFront();

    intake.hatchMechClosed();

    arm.releaseArmPivotBrake();

    arm.setPivotTargetAngle(arm.getPivotAngle());
    arm.setArmTargetExt(arm.getArmExtensionDistance());
    intake.setWristTargetAngle(intake.getWristAngle());

    //intake.getCargo(HOLD_BALL_POWER);
  }

  private void robotControls()
  { 
    //
    // System.out.println(xboxAux.getPOV());
    /**
     *  Drivetrain and lift controls
     * 
     */

    CatzDriveTrain.arcadeDrive(xboxDrv.getY(Hand.kLeft), -xboxDrv.getX(Hand.kRight));

    if(xboxDrv.getBumper(Hand.kRight) == true) 
    {
      //lift enabled     
      lift.lift(xboxDrv.getX(Hand.kLeft));
    }
    else
    {
      //lift disabled
      lift.lift(0);
    }
    

    
    /**
     * 
     *  Pivot Movement
     * 
     * 
     */
    
    pivotPower = xboxAux.getY(Hand.kLeft);
   
    if(Math.abs(pivotPower) > AUX_JOYSTICK_LT_DEADBAND_THRESHOLD)
    {
      //pause pivot thread and rotate pivot
      armPIDState = false;
      arm.setPivotTargetAngle(CatzConstants.INVALID_ANGLE);
      arm.turnPivot(pivotPower);

     /* if(!PDPCurrent[0] && !PDPCurrent[15])
      {
        arm.turnPivot(pivotPower * scalar);
      }
      else
      {
        arm.turnPivot(0.0);
      } */
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
     //arm.extendArm(armExtPower);
     if(Math.abs(armExtPower) > 0.1)
     {
       /*-----------------------------------------------------------------------
       *  Out of Deadband - Manual Control
       * 
       *----------------------------------------------------------------------*/
       arm.setArmTargetExt(CatzConstants.INVALID_EXT);
       armExtPIDState = false;  

       //if(!PDPCurrent[8] && !PDPCurrent[9])
       //{

        arm.extendArm(armExtPower);

       //} else 
       //{
        //arm.extendArm(0.0);
       //}
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
     *  Aux Controller - Hatch Position Presets
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
     * Aux Controller - Cargo Position Presets
     * 
    */ 
    if(xboxAux.getYButton())
    {
      CatzPreSets.lvl3RocketBall();
    }
    else if(xboxAux.getBButton())
    {
      //CatzPreSets.lvl2RocketBall();
      CatzPreSets.prestowed();
      Timer.delay(0.2);

      CatzPreSets.prestowed2();
      Timer.delay(0.2);

      CatzPreSets.stowed();
    }
    else if(xboxAux.getAButton())
    {
      CatzPreSets.cargoBayBall();
    }
    else if(xboxAux.getXButton())
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

      //if(!PDPCurrent[10])
      //{
       intake.rotateWrist(xboxAux.getY(Hand.kRight));
      //} else 
      //{
        //intake.rotateWrist(0.0);
      //}

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
      intake.getCargo(HOLD_BALL_POWER);
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
        intake.getCargo(HOLD_BALL_POWER);
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
      if(intake.getHatchState() == DoubleSolenoid.Value.kReverse)
      {
       intake.hatchMechClosed();
      }
      else
      {
        intake.hatchMechOpen();
      }
    }

    if(xboxAux.getStickButtonPressed(Hand.kLeft) && xboxAux.getStickButtonPressed(Hand.kRight))
    {
      arm.stopPivotPID();
      intake.stopWristPID();
    }


    if(xboxDrv.getXButton())
    {
      intake.hatchMechClosed();
    }
     if(xboxDrv.getBButton())
    {
      intake.hatchMechOpen();
    }
  }

    
  

  private static void canBusCheck()
  {
    /*if(driveTrain.getMtrCtrlLTBackFirmware() != CatzConstants.SPARK_MAX_FIRMWARE)
    {
      canBusWarningMessages[CatzConstants.DRV_TRAIN_LT_BACK] = "Drive Train LT BACK CAN Issues";
    }
    if(driveTrain.getMtrCtrlLTMidlFirmware() != CatzConstants.SPARK_MAX_FIRMWARE)
    {
      canBusWarningMessages[CatzConstants.DRV_TRAIN_LT_MIDL] = "Drive Train LT Midle CAN Issues";
    }
    if(driveTrain.getMtrCtrlLTFrntFirmware() != CatzConstants.SPARK_MAX_FIRMWARE)
    {
      canBusWarningMessages[CatzConstants.DRV_TRAIN_LT_FRNT] = "Drive Train LT FRNT CAN Issues";
    }
    if(driveTrain.getMtrCtrlRTBackFirmware() != CatzConstants.SPARK_MAX_FIRMWARE)
    {
      canBusWarningMessages[CatzConstants.DRV_TRAIN_RT_BACK] = "Drive Train RT Back CAN Issues";
    }
    if(driveTrain.getMtrCtrlRTMidlFirmware() != CatzConstants.SPARK_MAX_FIRMWARE)
    {
      canBusWarningMessages[CatzConstants.DRV_TRAIN_RT_MIDL] = "Drive Train RT Middle CAN Issues";
    }
    if(driveTrain.getDrvMtrCtrlRTBackFirmware() != CatzConstants.SPARK_MAX_FIRMWARE)
    {
      canBusWarningMessages[CatzConstants.DRV_TRAIN_RT_FRNT] = "Drive Train RT Front CAN Issues";
    }*/


  }

  
}

