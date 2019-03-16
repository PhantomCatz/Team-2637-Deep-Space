
/**

 * Author : Jeffrey Li

 *  Methods : getCargo, relaseCargo, rotateWrist, stopWrist, closeCargoClamp, openCargoClamp

 *  Functionality : gets cargo and release cargo, start and stop wrist

 *    

 *  02-13-19

 * revision history: changed enum solenoid to kForward JL

 */

package frc.Mechanisms;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.CatzConstants;
import frc.robot.Robot;

public class CatzIntake
{

    private static WPI_VictorSPX intakeRollerMtrCtrl;
    private static WPI_TalonSRX intakeWristMtrCtrl;

    public static AnalogInput intakeWristEnc;

    public static DoubleSolenoid hatchEjectSolenoid;

    private final int INTAKE_ROLLER_MC_CAN_ID = 31;
    private final int INTAKE_WRIST_MC_CAN_ID = 30;

    private final int HATCH_EJECT_PCM_PORT_A = 2;
    private final int HATCH_EJECT_PCM_PORT_B = 3;

    private final double INTAKE_WRIST_ENC_MAX_VOLTAGE     = 5.0;
    private final int    INTAKE_WRIST_ENCODER_ANALOG_PORT = 0;

    private final double WRIST_ANGLE_TOLERANCE = 2.0;      // TBD
    private final double WRIST_ANGLE_UP_MAX    = -95.0;    // 0 Deg is when intake is parallel to ground
    private final double WRIST_ANGLE_DN_MAX    = +95.0;   

    private final double WRIST_SN02_ENCODER_REF_ANGLE      = 127;//180  // This is the angle when intake is parrallel to ground
    private final double WRIST_SN02_ENCODER_REF_VOLTAGE    = ((WRIST_SN02_ENCODER_REF_ANGLE / 360.0) * INTAKE_WRIST_ENC_MAX_VOLTAGE);
    private final double WRIST_SN02_ENCODER_VOLTAGE_OFFSET = INTAKE_WRIST_ENC_MAX_VOLTAGE - WRIST_SN02_ENCODER_REF_VOLTAGE;
    private final double WRIST_SN02_ENCODER_ANGLE_OFFSET   = ( WRIST_SN02_ENCODER_VOLTAGE_OFFSET / 5.0 ) * 360.0;
     
    private double WRIST_SN02_ENCODER_MIN_VOLTAGE    = ( (WRIST_ANGLE_DN_MAX - WRIST_SN02_ENCODER_ANGLE_OFFSET)/360) * 5 ;

    private Boolean INTAKE_OPEN = false;

    private static volatile double targetAngle;
    private final double WRIST_STOWED_ANGLE = 0;

    public static volatile double WRIST_DEBUG_KP = 0;
    public static volatile double WRIST_DEBUG_KD = 0;
    public static volatile double WRIST_DEBUG_KA = 0;
    public static volatile double WRIST_DEBUG_KS = 0;

    public CatzIntake()
    {
        intakeRollerMtrCtrl = new WPI_VictorSPX(INTAKE_ROLLER_MC_CAN_ID);
        intakeWristMtrCtrl = new WPI_TalonSRX(INTAKE_WRIST_MC_CAN_ID);

        intakeRollerMtrCtrl.setNeutralMode(NeutralMode.Coast);
        intakeWristMtrCtrl.setNeutralMode(NeutralMode.Brake);

        intakeRollerMtrCtrl.setInverted(true);

        hatchEjectSolenoid = new DoubleSolenoid(HATCH_EJECT_PCM_PORT_A, HATCH_EJECT_PCM_PORT_B);

        intakeWristEnc = new AnalogInput(INTAKE_WRIST_ENCODER_ANALOG_PORT);

        if (CatzConstants.USING_COMPETITION_ROBOT) 
        {
            //TBD
        } 
        else 
        {
            //TBD
        }
    }

    public void hatchEject()
    {
        hatchEjectSolenoid.set(Value.kForward);
    }

    public void hatchDeployed()
    {
        hatchEjectSolenoid.set(Value.kReverse);
    }

    public void getCargo(double power)
    {
        intakeRollerMtrCtrl.set(power);
    }

    public void releaseCargo(double power)
    {
        intakeRollerMtrCtrl.set(-power);
    }

    public boolean isIntakeOpen()
    {
        return INTAKE_OPEN;
    }

    public double getIntakePower()
    {
        return intakeRollerMtrCtrl.get();
    }

    public double getWristPower()
    {
        return intakeWristMtrCtrl.get();
    }

    public void wristToAngle(double angle)
    {
        System.out.println("***** asdas ***********");
        Thread t = new Thread(() ->
        {
            final double targetAngle = angle; //TBD
            final double ARM_PIVOT_THREAD_WAITING_TIME = 0.005;
            final double kP = 0.003; //TODO
            final double kD = 0.001;
           
            double power;            

            Timer wristTimer = new Timer();
            wristTimer.start();

            double previousError = 0;
            double currentError;
            double deltaError = 0;
            
            double previousTime = 0;
            
            double deltaTime;
            
            double currentTime  = wristTimer.get();
            double currentAngle = getWristAngle();

            while (Robot.xboxAux.getY(Hand.kRight) == 0)
            {
                currentError = targetAngle - currentAngle;
                
                deltaError = currentError - previousError;
                deltaTime = currentTime - previousTime;
                
                power = kP * currentError +
                        kD * (deltaError / deltaTime);//ka compensates for angle of arm
                        //arm extension distace + 13 is the distance from pivot to wrist

                // Limit power to +/- 0.5 max
                if (power > 0.55)
                {
                    power = 0.55;
                }
                else if (power < -0.4)
                {
                    power = -0.4;
                }

                intakeWristMtrCtrl.set(power);
                System.out.println("WRIST: TAEDP=, " + deltaTime + ", "  + currentAngle + ", " + currentError + ", " + + deltaError + ",  " + power);

                previousError = currentError;
                previousTime = currentTime;
 
                Timer.delay(ARM_PIVOT_THREAD_WAITING_TIME);

                currentTime  = wristTimer.get();
                currentAngle = getWristAngle();
            }
            intakeWristMtrCtrl.set(0);
            Thread.currentThread().interrupt();    
        });
        t.start();
    }

    public void rotateWrist(double power)
    {   
        double wristAngle = this.getWristAngle();
        if(CatzConstants.USING_SOFT_LIMITS)
        {
            System.out.println("W-PA " + power + ", " + wristAngle);
            if(Math.abs(power) < 0.09) 
            {
                System.out.println("W-Hold");
                //setWristTargetAngle(wristAngle);
            }
            else if(power < -0.09)
            {
                // Wrist is being commanded Up (Increasing Angle)
                setWristTargetAngle(CatzConstants.INVALID_ANGLE);

                if(wristAngle < WRIST_ANGLE_UP_MAX)
                {
                    System.out.println("W-UP_LIMIT");
                    intakeWristMtrCtrl.set(0);
                    setWristTargetAngle(WRIST_ANGLE_UP_MAX);
                }
                else
                {
                    System.out.println("W-Up");
                    intakeWristMtrCtrl.set(0.7 * power);
                }
            }
            else if(power > 0.09)
            {
                // Wrist is being commanded Down (Increasing Angle)
                setWristTargetAngle(CatzConstants.INVALID_ANGLE);

                if(wristAngle > WRIST_ANGLE_DN_MAX)
                {
                    System.out.println("W-DN-LIMIT");
                    intakeWristMtrCtrl.set(0);
                    setWristTargetAngle(WRIST_ANGLE_DN_MAX);

                }
                else
                {
                    System.out.println("W-Down");
                    intakeWristMtrCtrl.set(0.3 * power);
                }
            }        
        }
        else
        {
            intakeWristMtrCtrl.set(power);
            //setWristTargetAngle(wristAngle);
        }
    }

    public double getTargetAngle()
    {
        return targetAngle;
    }
    
    public double getWristAngle()
    {
        double encVoltage = intakeWristEnc.getVoltage();
        double adjVoltage = 0.0;
        double wristAngle = 999.0;

        wristAngle = (((encVoltage - 1.5)/ INTAKE_WRIST_ENC_MAX_VOLTAGE) * 360);

        return wristAngle;
    }



    public void moveWristThread(double targetAngle, double power, double timeOut)
    {
        final double WRIST_THREAD_WAITING_TIME = 0.005;

        Timer threadTimer = new Timer();
        threadTimer.start();

        Thread wristThread = new Thread(() ->
        {
            double currentAngle = getWristAngle();

            double errorAngle = Math.abs(targetAngle - currentAngle);

            double upperLimit = targetAngle + WRIST_ANGLE_TOLERANCE;
            double lowerLimit = targetAngle - WRIST_ANGLE_TOLERANCE;

            if (errorAngle < WRIST_ANGLE_UP_MAX / 2.0)
            {
                intakeWristMtrCtrl.set(power);
            }

            else if (errorAngle > WRIST_ANGLE_UP_MAX / 2.0)
            {
                intakeWristMtrCtrl.set(-power);
            }

            while (!Thread.interrupted())
            {
                currentAngle = getWristAngle(); // update the currentAngle

                if ((lowerLimit < currentAngle && upperLimit > currentAngle) || threadTimer.get() > timeOut)
                {

                    intakeWristMtrCtrl.stopMotor();

                    Thread.currentThread().interrupt();

                }
                Timer.delay(WRIST_THREAD_WAITING_TIME);
            }
        });

        wristThread.start();

    }

    public void wristPDThread(double targetAngle, double timeOut)
    {
        Thread wristThread = new Thread(() ->
        {
            final double WRIST_THREAD_WAITING_TIME = 0.005;

            final double kP = 0;
            final double kD = 0;

            Timer threadTimer = new Timer();
            threadTimer.start();

            double previousError = targetAngle;
            double deltaError;
            double previousTime = 0;
            double deltaTime;
            double power;

            double currentAngle = getWristAngle();
            double currentError;
            double currentTime = threadTimer.get();

            while ((Math.abs(targetAngle - currentAngle) < WRIST_ANGLE_TOLERANCE) && (currentTime < timeOut))
            {

                currentError = targetAngle - currentAngle;

                deltaError = currentError - previousError;

                deltaTime = currentTime - previousTime;

                power = kP * currentError +

                        kD * (deltaError / deltaTime);

                rotateWrist(power);

                previousError = currentError;

                previousTime = currentTime;

                Timer.delay(WRIST_THREAD_WAITING_TIME);

                currentAngle = getWristAngle();

                currentTime = threadTimer.get();

            }

            rotateWrist(0);
            ;

            Thread.currentThread().interrupt();

        });
        wristThread.start();
    }

    public void setWristTargetAngle(double angle)
    {
        targetAngle = angle;
    }



    public void wristPID()
    {

        final double WRIST_THREAD_WAITING_TIME = 0.005;

        Thread t = new Thread(() ->
        {
            setWristTargetAngle(WRIST_STOWED_ANGLE);

            //final double kP = 0.08; //TODO
            //final double kD = 0.0001;
            //final double kA = 0.01;
            //final double kS = 10;

            final double MINIMUM_POWER = 0.05;

            double power;            

            Timer wristTimer = new Timer();
            wristTimer.start();

            double previousError = 0;
            double currentError;
            double deltaError = 0;
            double errorRate;

            double previousTime = 0;
            
            double currentDerivative;
            double previousDerivative = 0; // in case you want to filter derivative
            double filteredDerivative;
            
            double deltaTime;
            
            double currentTime  = wristTimer.get();
            double currentAngle = getWristAngle();

            boolean firstTime = false;
            while(true)
            {

                double kP = WRIST_DEBUG_KP;
                double kD = WRIST_DEBUG_KD;
                double kA = WRIST_DEBUG_KA;
                double kS = WRIST_DEBUG_KS;

                if(targetAngle >= CatzConstants.INVALID_ANGLE)
                {
                    Timer.delay(CatzConstants.CONTROLLER_INPUT_WAIT_TIME);
                }
                else
                {
                    currentTime  = wristTimer.get();
                    currentAngle = getWristAngle();

                    currentError = targetAngle - currentAngle;
                
                    deltaError = currentError - previousError;
                    deltaTime  = currentTime  - previousTime;

                    currentDerivative = (deltaError/deltaTime);

                    filteredDerivative = (0.4 * currentDerivative + 0.6 * previousDerivative);

                    if(firstTime == true)
                    {
                        power = ( kS * (kP * currentError +
                                        kD * currentDerivative));
                    }
                    else
                    {
                        power = ( kS * (kP * currentError +
                                        kD * filteredDerivative));
                    }
                    //power limiting
                    if (power > 0.35)
                    {
                        power = 0.35;
                    }
                    else if (power < -0.40)
                    {
                        power = -0.40;
                    }

                    //power floor
                    if(power < MINIMUM_POWER)
                    {
                        power = MINIMUM_POWER;
                    }

                    rotateWrist(-power);
                    System.out.println("W-A E DE ER P " + currentAngle + ", " + currentError + "," + deltaError + ", " + currentDerivative + ", " +-power);

                    previousError = currentError;
                    previousTime = currentTime;
                    previousDerivative = currentDerivative;
                

                    firstTime = false;
                    Timer.delay(WRIST_THREAD_WAITING_TIME);
                }
                
            }
             
        });
        t.start();
    }
}
