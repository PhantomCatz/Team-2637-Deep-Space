package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

import frc.Vision.UDPServerThread;
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

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
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
    
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() 
  { 

    VisionObject vo = VisionObjContainer.get("vis");

    distance = 0;
    heading = 0;

    /*
    VisionObject vo = VisionObjContainer.get();

    if (vo != null)
    {
      System.out.println(vo);
    }
    

    //For print out
    Enumeration<VisionObject> vobjs = VisionObjContainer.getElements();
  
    boolean newLine = false;

    if (vobjs != null)
    {
      //System.out.print("vobjs is not null");
      while (vobjs.hasMoreElements())
      {
        String str = vobjs.nextElement().toString();

        System.out.print(str + '\t');          

        newLine = true;
      }
    }

    if (newLine)
    {
      System.out.println();
    }

    */
    
    if (vo != null)
    {

      distance = VisionObjContainer.get("vis").getDistance();
      heading = VisionObjContainer.get("vis").getHeading();

      SmartDashboard.putNumber("Distance", distance);
      SmartDashboard.putNumber("Heading", heading);

    }

  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
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
    lift.lift(xboxDrv.getTriggerAxis(Hand.kRight) - xboxDrv.getTriggerAxis(Hand.kLeft));

    //moves arm pivot
    arm.turnPivot(xboxAux.getY(Hand.kLeft));

    //extends retracts arm
    arm.extendArm(xboxAux.getTriggerAxis(Hand.kRight) - xboxAux.getTriggerAxis(Hand.kLeft));

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

    if(xboxDrv.getBumper(Hand.kRight))
    {
      intake.hatchEject();
    }
    else if(xboxDrv.getBumper(Hand.kLeft))
    {
      intake.hatchDeployed();
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
