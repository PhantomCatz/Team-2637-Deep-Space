package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

import frc.Vision.UDPServerThread;
import edu.wpi.first.wpilibj.GenericHID.Hand;
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

  public static AHRS navx;

  private static XboxController xboxDrv;
  private static XboxController xboxAux;

  private static final int XBOX_DRV_PORT = 0;
  private static final int XBOX_AUX_PORT = 1;

  private static final double MAX_POWER  = 1;

  private double heading;
  private double distance;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() 
  {
    navx       = new AHRS(SPI.Port.kMXP,(byte)200);

    server     = new UDPServerThread();

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

  }

  @Override
  public void autonomousInit() 
  {
   
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() 
  {

    double motorPower = 0.5;
    double rotation = heading / 20;

    if(distance == 0)
    {
      motorPower = 0;
    }

    System.out.println("rotation" + rotation);
    CatzDriveTrain.arcadeDrive(motorPower, rotation);
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
      lift.lift(1);
    }
    else if(xboxDrv.getBumper(Hand.kRight))
    {
      lift.lift(-1);
    }
    else
    {
      lift.lift(0);
    }
    
    //moves arm pivot
    arm.turnPivot(xboxAux.getY(Hand.kLeft));

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
    if(xboxDrv.getBumper(Hand.kRight))
    {
      intake.hatchEject();
    }
    else if(xboxDrv.getBumper(Hand.kLeft))
    {
      intake.hatchDeployed();
    }

    // open/close the intake
    if(xboxAux.getXButton())
    {
      intake.openCargoClamp();
    }
    else if(xboxAux.getBButton())
    {
      intake.closeCargoClamp();
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
