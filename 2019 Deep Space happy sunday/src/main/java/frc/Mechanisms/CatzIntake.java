
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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.CatzConstants;

public class CatzIntake
{

    private static WPI_VictorSPX intakeRollerMtrCtrl;
    private static WPI_TalonSRX intakeWristMtrCtrl;

    public static AnalogInput intakeWristEnc;

    public static DigitalInput bumpSwitch;

    public static DoubleSolenoid hatchEjectSolenoid;

    private final int INTAKE_ROLLER_MC_CAN_ID = 31;
    private final int INTAKE_WRIST_MC_CAN_ID = 30;

    //private final int CARGO_CLAMP_PCM_PORT_A = 4;
    //private final int CARGO_CLAMP_PCM_PORT_B = 5; //unplugged on kinectabot, used for single-solenoid ops

    private final int HATCH_PCM_PORT_A = 4; 
    private final int HATCH_PCM_PORT_B = 6;

    private final double INTAKE_WRIST_ENC_MAX_VOLTAGE     = 5.0;
    private final int    INTAKE_WRIST_ENCODER_ANALOG_PORT = 0;

    private final double WRIST_ANGLE_TOLERANCE = 2.0;      // TBD
    private final double WRIST_ANGLE_UP_MAX    = -95.0;    // 0 Deg is when intake is parallel to ground
    private final double WRIST_ANGLE_DN_MAX    = +95.0;   

    private final double WRIST_SN02_ENCODER_REF_ANGLE;
    private final double WRIST_SN02_ENCODER_REF_VOLTAGE;
    private final double WRIST_SN02_ENCODER_VOLTAGE_OFFSET;
    private final double WRIST_SN02_ENCODER_ANGLE_OFFSET;
    private double WRIST_SN02_ENCODER_MIN_VOLTAGE;

    private Boolean INTAKE_OPEN = false;

    private static volatile double targetAngle = 999.0;
    private final double WRIST_STOWED_ANGLE = 0;

    private final int BUMP_SWITCH_PORT = 9;   

    private Thread wristThread;

    public static double kPPow;
    public static double kIPow;  
    public static double kDPow;

    public CatzIntake()
    {
        intakeRollerMtrCtrl = new WPI_VictorSPX(INTAKE_ROLLER_MC_CAN_ID);
        intakeWristMtrCtrl = new WPI_TalonSRX(INTAKE_WRIST_MC_CAN_ID);


        intakeRollerMtrCtrl.setNeutralMode(NeutralMode.Coast);
        intakeWristMtrCtrl.setNeutralMode(NeutralMode.Brake);

        intakeRollerMtrCtrl.setInverted(true);

        hatchEjectSolenoid = new DoubleSolenoid(HATCH_PCM_PORT_A, HATCH_PCM_PORT_B);
        //cargoClampSolenoid = new DoubleSolenoid(CARGO_CLAMP_PCM_PORT_A, CARGO_CLAMP_PCM_PORT_B);

        intakeWristEnc = new AnalogInput(INTAKE_WRIST_ENCODER_ANALOG_PORT);

        bumpSwitch = new DigitalInput(BUMP_SWITCH_PORT);

        if (CatzConstants.USING_COMPETITION_ROBOT) 
        { 
            WRIST_SN02_ENCODER_REF_ANGLE      = 223.2;//230.4;// This is the angle when intake is parrallel to ground
            WRIST_SN02_ENCODER_REF_VOLTAGE    = ((WRIST_SN02_ENCODER_REF_ANGLE / 360.0) * INTAKE_WRIST_ENC_MAX_VOLTAGE);
            WRIST_SN02_ENCODER_VOLTAGE_OFFSET = WRIST_SN02_ENCODER_REF_VOLTAGE;
            WRIST_SN02_ENCODER_ANGLE_OFFSET   = ( WRIST_SN02_ENCODER_VOLTAGE_OFFSET / 5.0 ) * 360.0;
            WRIST_SN02_ENCODER_MIN_VOLTAGE    = ( (WRIST_ANGLE_DN_MAX - WRIST_SN02_ENCODER_ANGLE_OFFSET)/360) * 5 ;

            
        } 
        else 
        {
            WRIST_SN02_ENCODER_REF_ANGLE      = 256.1;// This is the angle when intake is parrallel to ground
            WRIST_SN02_ENCODER_REF_VOLTAGE    = ((WRIST_SN02_ENCODER_REF_ANGLE / 360.0) * INTAKE_WRIST_ENC_MAX_VOLTAGE);
            WRIST_SN02_ENCODER_VOLTAGE_OFFSET = WRIST_SN02_ENCODER_REF_VOLTAGE;//INTAKE_WRIST_ENC_MAX_VOLTAGE - WRIST_SN02_ENCODER_REF_VOLTAGE;
            WRIST_SN02_ENCODER_ANGLE_OFFSET   = ( WRIST_SN02_ENCODER_VOLTAGE_OFFSET / 5.0 ) * 360.0;
            WRIST_SN02_ENCODER_MIN_VOLTAGE    = ( (WRIST_ANGLE_DN_MAX - WRIST_SN02_ENCODER_ANGLE_OFFSET)/360) * 5 ;

        }
    }

    public Value getHatchState()
    {
        return hatchEjectSolenoid.get();
    }

    public boolean isBumpSwitchPressed()
    {
        return !bumpSwitch.get();
    }

    public void hatchMechOpen() //open
    {
        hatchEjectSolenoid.set(Value.kReverse);
       // Timer.delay(0.2); //testing DELETE ME 
       // hatchEjectSolenoid.open(); // testing
       //System.out.println("REV SOLENOID BLACKLIST" + hatchEjectSolenoid.isRevSolenoidBlackListed());
    }

    public void hatchMechClosed() //closed
    {
        hatchEjectSolenoid.set(Value.kForward);
        //System.out.println("FWD SOLENOID BLACKLIST" + hatchEjectSolenoid.isFwdSolenoidBlackListed());
    } 

    public void getCargo(double power)
    {
        intakeRollerMtrCtrl.set(-power);
    }

    public void releaseCargo(double power)
    {
        intakeRollerMtrCtrl.set(power);
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

    public double getWristEncVoltage()
    {
        return intakeWristEnc.getVoltage();
    }
    
    public double getWristAngle()
    {
        double encVoltage = intakeWristEnc.getVoltage();
        double wristAngle = 999.0;

        wristAngle = (((encVoltage - WRIST_SN02_ENCODER_VOLTAGE_OFFSET) / INTAKE_WRIST_ENC_MAX_VOLTAGE) * 360.0);

        return wristAngle;
    }

    public void setWristTargetAngle(double angle)
    {
        targetAngle = angle;
    }

    

    public void wristPID()
    {
        final double WRIST_THREAD_WAITING_TIME = 0.005;

        wristThread = new Thread(() ->
        {
            final double kP = 0.012;//0.03 //0.0024 //TODO
            final double kI = 0.0000015;//0.000015
            final double kD = 0.0001;//0.0002; //0.007
            final double kF = 0.0;//-0.1;

            double power;            

            Timer wristTimer = new Timer();
            wristTimer.start();

            double integral = 0;

            double previousError = 0;
            double currentError;
            double deltaError = 0;
            double errorRate;

            double previousTime = 0;
            
            double currentDerivative;
            double previousDerivative = 0; // in case you want to filter derivative
            
            double deltaTime;
            
            double currentTime  = wristTimer.get();
            double currentAngle = getWristAngle();

            while(true)
            {
                if(targetAngle == CatzConstants.INVALID_ANGLE)
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

                    //Riemann Sum
                    integral += deltaTime * currentError;

                    currentDerivative = (deltaError/deltaTime);
                    
                    kPPow = kP * currentError;
                    kIPow = integral*kI;
                    kDPow = kD * currentDerivative;

                    //SmartDashboard.putNumber("wrist error", currentError);

                    power = kPPow + kIPow + kDPow + kF;
                    
                    rotateWrist(power);

                    previousError = currentError;
                    previousTime = currentTime;
                    previousDerivative = currentDerivative;
                
                    Timer.delay(WRIST_THREAD_WAITING_TIME);
                }
                
            }
             
        });
        wristThread.start();
    }

    public void stopWristPID()
    {
        wristThread.interrupt();
    }
}
