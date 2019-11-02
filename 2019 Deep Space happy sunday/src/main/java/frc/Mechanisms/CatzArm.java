/*
 *  Author : Jean Kwon
 * 
 * Functionality : controls the arm extension by the power, controls the arm pivot by the power,
 *                 gets the status of each limit switch, gets the angle of the arm pivot,  
 *                 moves the arm extension to the target distance, moves the arm pivot to the targetAngle
 * 
 *  Methods : moveArm, movePivot, getExtensionEncoderCounts, isArmLimitExtendedActivated, isArmLimitRetractedActivated,
 *           getPivotAngle, moveArmThread, moveArmThread
 * 
 *  Revision History : 
 *  02-04-19 Added the thread and the encoder JK
 * 
 */
package frc.Mechanisms;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.CatzConstants;
import frc.robot.Robot;

public class CatzArm 
{
    private boolean PIVOT_BRAKE_ENGAGED = true;
    private boolean EXTENSION_BRAKE_ENGAGED = true;

    private final int ARM_EXTENSION_BRAKE_SOLENOID_PORT_A = 2; //TODO, tbd
    private final int ARM_EXTENSION_BRAKE_SOLENOID_PORT_B = 3;

    private final int ARM_PIVOT_BRAKE_SOLENOID_PORT = 6; //TODO, TBD

    private final int ARM_EXTENSION_A_MC_CAN_ID = 20;
    private final int ARM_EXTENSION_B_MC_CAN_ID = 21;

    private final int ARM_PIVOT_LT_MC_CAN_ID = 40;
    private final int ARM_PIVOT_RT_MC_CAN_ID = 41;

    private final int ARM_PIVOT_MTR_CTRL_CURRENT_LIMIT = 60;

    private static DigitalInput armRetractedLimitSwitch;

    private final int ARM_EXTENSION_LIMIT_EXTENDED_DIO_PORT  = 0; //TODO, TBD, same placeholding values woul conflict
    private final int ARM_EXTENSION_LIMIT_RETRACTED_DIO_PORT = 1;
  
    private static final int    ARM_PIVOT_ENCODER_ANALOG_PORT = 1;
    private static final double ARM_PIVOT_ENC_MAX_VOLTAGE     = 5.0;
  
    private static final int    ARM_PIVOT_ANGLE_TOLERANCE = 3; //TBD

    private static final double INPUT_THRESHOLD = 1.0E-3;

    private static final double SOLENOID_ACTION_DELAY = 0.01;

    public double breakStart;
    
    

    //private static final double ARM_PIVOT_ANGLE_MAX = 270.0;

    //private static final double MAX_EXTENSION_LIMIT_INCHES = 30 / Math.cos(Math.abs(getPivotAngle()));
    private static WPI_TalonSRX  armExtensionMtrCtrlA;  //A and B are designators
    private static WPI_VictorSPX armExtensionMtrCtrlB;

    private static CANSparkMax armPivotMtrCtrlLT;
    private static CANSparkMax armPivotMtrCtrlRT;

    private static DoubleSolenoid armExtensionBrakeSolenoid;

    //private static Solenoid armPivotBrakeSolenoid;
    
    public static AnalogInput armPivotEnc;

    public static double currentExt;

    

    //public static Encoder armExtensionEnc;

    /***************************************************************************
    * Arm Extension Encoder - pulses to inches 
    * SRX Magnetic Encoder which provides 4096 pulses per revolution. 
    * The gear reduction is 2 to 1.
    * The diameter of winch is 0.984 inch 
    * It attached to the same shaft
    *****************************************************************************/
    /*
    private static final double ARM_EXTENSION_ENCODER_PULSE_PER_REV = 4096;
    private static final double ARM_EXTENSION_WINCH_DIAMETER        = 0.984252; //2.5 cm
    private static final double ARM_EXTENSION_GEAR_RATIO            = 1.0/3.0;
    private static final double ARM_COUNTS_PER_INCHES = (ARM_EXTENSION_ENCODER_PULSE_PER_REV / 
                                                        (ARM_EXTENSION_WINCH_DIAMETER * Math.PI)) * ARM_EXTENSION_GEAR_RATIO ;
    */ //too complex for me, determined experamentally KH

    private static final double ARM_COUNTS_PER_INCH = 3594.0923 ; //-58404.0/16.25

    private static final double ARM_EXTENSION_COUNT_TOLERANCE = 100 * ARM_COUNTS_PER_INCH; //TBD Type it in inches

    private static final double ARM_PIVOT_MAX_ANGLE = 120.0;   //Robot 0 deg = Arm pointing down -45 deg
    private static final double ARM_PIVOT_MIN_ANGLE = 5.0;          //TBD

    private static double ARM_EXTENSION_MAX_POWER_RAMP_TIME = 0.5; //sec
    private static double ARM_PIVOT_MAX_POWER_RAMP_TIME     = 0.5; //sec

    private final double ARM_PIVOT_UP_LIMIT = 0.8;
    private final double ARM_PIVOT_DN_LIMIT = 0.4;

    private static volatile double targetAngle;

    private static volatile double targetExt;
    private static int armZero;

    public static boolean hasBeenReset = false;

    public static boolean armTargetHit = false;

    public static boolean runPivotPID = false;

    public static boolean pivotEncFailed = false;

    private final double PIVOT_ENC_REF_ANGLE;
    private final double PIVOT_ENC_REF_VOLTAGE;
    private final double PIVOT_ENC_VOLTAGE_OFFSET;

    //forgive my bad code KH
    public static double armPPow;
    public static double armIPow;
    public static double armDPow;
    public static double armAPow;


    private Thread armThread;
    private Thread pivotThread;

    public CatzArm()
    {
        armExtensionMtrCtrlA = new WPI_TalonSRX (ARM_EXTENSION_A_MC_CAN_ID);
        armExtensionMtrCtrlB = new WPI_VictorSPX(ARM_EXTENSION_B_MC_CAN_ID);

        armExtensionMtrCtrlA.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        
        armExtensionMtrCtrlA.setNeutralMode(NeutralMode.Brake);
        armExtensionMtrCtrlB.setNeutralMode(NeutralMode.Brake);

        armExtensionMtrCtrlB.follow(armExtensionMtrCtrlA);
        armExtensionMtrCtrlB.setInverted(true);

        //armExtensionMtrCtrlA.configOpenloopRamp(ARM_EXTENSION_MAX_POWER_RAMP_TIME);
        //armExtensionMtrCtrlB.configOpenloopRamp(ARM_EXTENSION_MAX_POWER_RAMP_TIME);

        //armExtensionEnc = new Encoder(6,7, false, EncodingType.k4X);
        
        //armExtensionEnc.setDistancePerPulse(1/ARM_COUNTS_PER_INCHES);

        armPivotMtrCtrlLT = new CANSparkMax(ARM_PIVOT_LT_MC_CAN_ID, MotorType.kBrushless);
        armPivotMtrCtrlRT = new CANSparkMax(ARM_PIVOT_RT_MC_CAN_ID, MotorType.kBrushless);

        armPivotMtrCtrlLT.follow(armPivotMtrCtrlRT);

        armPivotMtrCtrlRT.setIdleMode(IdleMode.kBrake);
        armPivotMtrCtrlLT.setIdleMode(IdleMode.kBrake);

        armPivotMtrCtrlRT.setSmartCurrentLimit(ARM_PIVOT_MTR_CTRL_CURRENT_LIMIT);
        armPivotMtrCtrlLT.setSmartCurrentLimit(ARM_PIVOT_MTR_CTRL_CURRENT_LIMIT);

        //armPivotMtrCtrlLT.setOpenLoopRampRate(ARM_PIVOT_MAX_POWER_RAMP_TIME);
        //armPivotMtrCtrlRT.setOpenLoopRampRate(ARM_PIVOT_MAX_POWER_RAMP_TIME);
 
        armPivotEnc = new AnalogInput(ARM_PIVOT_ENCODER_ANALOG_PORT);

        //armPivotBrakeSolenoid = new Solenoid(ARM_PIVOT_BRAKE_SOLENOID_PORT);

        armExtensionBrakeSolenoid = new DoubleSolenoid(ARM_EXTENSION_BRAKE_SOLENOID_PORT_A, ARM_EXTENSION_BRAKE_SOLENOID_PORT_B);

        armRetractedLimitSwitch = new DigitalInput(ARM_EXTENSION_LIMIT_RETRACTED_DIO_PORT);

        if(CatzConstants.USING_COMPETITION_ROBOT)
        {
            PIVOT_ENC_REF_ANGLE = 136.8 - 45; // angle pivot makes when parallel to ground
            PIVOT_ENC_REF_VOLTAGE = ((PIVOT_ENC_REF_ANGLE * ARM_PIVOT_ENC_MAX_VOLTAGE) / 360.0);
            PIVOT_ENC_VOLTAGE_OFFSET = PIVOT_ENC_REF_VOLTAGE;

            //PIVOT_ENC_VOLTAGE_OFFSET = 0.616;
        }
        else
        {

            PIVOT_ENC_REF_ANGLE = 119.088 - 45; // angle pivot makes when parallel to ground
            PIVOT_ENC_REF_VOLTAGE = ((PIVOT_ENC_REF_ANGLE * ARM_PIVOT_ENC_MAX_VOLTAGE) / 360.0);
            PIVOT_ENC_VOLTAGE_OFFSET = PIVOT_ENC_REF_VOLTAGE;

            //PIVOT_ENC_VOLTAGE_OFFSET = 0.586;
        }
        
    }

    public double getPivotEncVoltage()
    {
        return armPivotEnc.getVoltage();
    }

    public double getArmMtrCtrlACurrent()
    {
        return armExtensionMtrCtrlA.getOutputCurrent();
    }
    
    public double getArmMtrCtrlBCurrent()
    {
        return armExtensionMtrCtrlB.getBusVoltage();
    }

    public void engageArmExtensionBrake()
    {
        armExtensionBrakeSolenoid.set(Value.kForward);
        EXTENSION_BRAKE_ENGAGED = true;
    }

    public void releaseArmExtensionBrake()
    {
        armExtensionBrakeSolenoid.set(Value.kReverse);
        EXTENSION_BRAKE_ENGAGED = false;
    }

    public void engageArmPivotBrake()
    {
        //armPivotBrakeSolenoid.set(true);
        PIVOT_BRAKE_ENGAGED = true;
    }

    public void releaseArmPivotBrake()
    {
        //armPivotBrakeSolenoid.set(false);
        PIVOT_BRAKE_ENGAGED = false;
    }

    public double getPivotPower()
    {
        return armPivotMtrCtrlRT.get();
    }

    public double getExtensionPower()
    {
        return armExtensionMtrCtrlA.get();
    }

    public boolean isArmExtBrakeEngaged()
    {
        return PIVOT_BRAKE_ENGAGED;
    }

    public void extendArm(double power) 
    {
        releaseArmExtensionBrake();
        Timer.delay(SOLENOID_ACTION_DELAY);
        armExtensionMtrCtrlA.set(power);
    }

    public void holdArm()
    {
        armExtensionMtrCtrlA.set(0.0);
        engageArmExtensionBrake();
    }

    public void setArmTargetExt(double ext)
    {
        Robot.armExtPIDState = true;
        targetExt = ext;
        armTargetHit = false;
        breakStart = -1.0;
    } 

    public double getArmTargetExt()
    {
        return targetExt;
    }

    public void setArmTargetHit(boolean state)
    {
        armTargetHit = state;
    }

    public boolean getArmTargetHit()
    {
        return armTargetHit;
    }

    public double getArmExtPower()
    {
        return armExtensionMtrCtrlA.get();
    }

    public void ArmPID()
    {
        final double ARM_THREAD_WAITING_TIME = 0.005;

        armThread = new Thread(() ->
        {
            final double  kP = 0.037; //0.037
            final double  kI = 0.0;
            final double  kD = 0.6; //1.0
            final double  kA = 0.3;

            double temp;
            double power;            

            Timer armTimer = new Timer();
            armTimer.start();

            double integral = 0;

            double previousError = 0;
            double currentError;
            double deltaError = 0;

            double previousTime = 0;
            
            double currentDerivative;
            double previousDerivative = 0; // in case you want to filter derivative
            
            double deltaTime;
            
            double currentTime;   

            double breakDelay;

            breakStart = -1.0;

            while(true)
            {
                //If we hit limit switch, zero arm extension and zero the "zero position" on the extension
                if(isArmLimitRetractedActivated())
                {
                    //armZero = getArmExtensionEncoderCounts();
                    resetArmExtensionEncoderCounts();
                    currentExt = getArmExtensionDistance(); 
                }

                if(targetExt == CatzConstants.INVALID_EXT)
                {
                    Timer.delay(CatzConstants.CONTROLLER_INPUT_WAIT_TIME);
                }
                else
                {
                    currentTime = armTimer.get();
                    currentExt  = getArmExtensionDistance();

                    currentError = targetExt - currentExt;
                
                    if(Math.abs(currentError) < 1.0)
                    {
                        if(breakStart < 0)
                        {
                            breakStart = currentTime;
                        }

                        breakDelay = currentTime - breakStart;

                        if(breakDelay > 0.05)
                        {
                            holdArm();
                            setArmTargetHit(true);//armTargetHit = true;
                            Robot.armExtPIDState = false;
                        }

                    }
                    else
                    {
                        breakStart = -1.0;
                    }

                    if(armTargetHit == false)
                    {
                        deltaError = currentError - previousError;
                        deltaTime  = currentTime  - previousTime;

                        //Riemann Sum
                        integral += deltaTime * currentError;

                        //Derivative
                        currentDerivative = (deltaError / deltaTime);
                        
                        releaseArmExtensionBrake();

                        temp = Math.sin(Math.toRadians(getPivotAngle() - 45));
                        SmartDashboard.putNumber("arm error", currentError);

                        armPPow = kP * currentError;
                        armIPow = kI * integral;
                        armDPow = kD * currentDerivative;
                        armAPow = kA * temp;

                        power = armPPow + armIPow + armDPow + armAPow;
                        armExtensionMtrCtrlA.set(power);

                        previousError = currentError;
                        previousTime = currentTime;
                        previousDerivative = currentDerivative;
                        
                        Timer.delay(ARM_THREAD_WAITING_TIME);
                    }
                }
            }
        });
        armThread.start();
    }

    

    public void turnPivot(double power)
    {
        if(CatzConstants.USING_SOFT_LIMITS)
        {
            double pivotAngle = this.getPivotAngle();

            if (power >  0.0)
            {
                // Pivot is being commanded CCW (Increasing Angle)
                setPivotTargetAngle(CatzConstants.INVALID_ANGLE);
                
                if(pivotAngle >= ARM_PIVOT_MAX_ANGLE)
                {
                    armPivotMtrCtrlRT.set(0);   
                }
                else
                {
                    if(PIVOT_BRAKE_ENGAGED == false)
                    {
                        armPivotMtrCtrlRT.set(ARM_PIVOT_UP_LIMIT * power);
                    }
                    else
                    {
                        releaseArmPivotBrake();
                        Timer.delay(SOLENOID_ACTION_DELAY);
                        armPivotMtrCtrlRT.set(ARM_PIVOT_UP_LIMIT * power);

                    }
                }
            }       
            else
            { 
                // Pivot is being commanded to Stop or go CW (Decreasing Angle)
                setPivotTargetAngle(CatzConstants.INVALID_ANGLE);
                
                if (pivotAngle <= ARM_PIVOT_MIN_ANGLE)
                {
                    armPivotMtrCtrlRT.set(0);
                }
                else
                {
                    if(PIVOT_BRAKE_ENGAGED == false)
                    {
                        armPivotMtrCtrlRT.set(ARM_PIVOT_DN_LIMIT * power);
                    }
                    else
                    {
                        releaseArmPivotBrake();
                        Timer.delay(SOLENOID_ACTION_DELAY);
                        armPivotMtrCtrlRT.set(ARM_PIVOT_DN_LIMIT * power);

                    }
                }
            }
            
        }
        else
        {
            if(Math.abs(power) >= INPUT_THRESHOLD)
            {      
                releaseArmPivotBrake();
                //Timer.delay(SOLENOID_ACTION_DELAY);
                if (power >  0.0)
                {
                    // Pivot is being commanded CCW (Increasing Angle)
                    armPivotMtrCtrlRT.set(ARM_PIVOT_UP_LIMIT * power);
                } 
                else 
                {
                    armPivotMtrCtrlRT.set(ARM_PIVOT_DN_LIMIT * power);

                }

                System.out.println(armPivotMtrCtrlRT.get());
            }
            else
            {
                //engageArmPivotBrake();
                armPivotMtrCtrlRT.set(0);
                System.out.println("*************************************************************");

            }  
        }
    }

    public int getArmExtensionEncoderCounts()
    {
        return -armExtensionMtrCtrlA.getSensorCollection().getQuadraturePosition();
    }

    public int getArmSpeed()
    {
        return armExtensionMtrCtrlA.getSelectedSensorVelocity();
    }

    //returns extension in inches
    public double getArmExtensionDistance()
    {
        return (((double) getArmExtensionEncoderCounts() / ARM_COUNTS_PER_INCH));
    }

    public boolean isArmLimitRetractedActivated()
    {
        return armExtensionMtrCtrlA.getSensorCollection().isRevLimitSwitchClosed();
    }

    public void resetArmExtensionEncoderCounts()
    {
        hasBeenReset = true;
        armExtensionMtrCtrlA.getSensorCollection().setQuadraturePosition(0, 5);
    }

    public double getPivotAngle() 
    {   
        return (((armPivotEnc.getVoltage() - PIVOT_ENC_VOLTAGE_OFFSET) / ARM_PIVOT_ENC_MAX_VOLTAGE) * 360.0);
    }
  
    public double getPivotTargetAngle()
    {
        return targetAngle;
    }

    public void setPivotTargetAngle(double angle)
    {
        targetAngle = angle;
    }

    public boolean getPivotBrakeStatus()
    {
        return PIVOT_BRAKE_ENGAGED;
    }
    public void pivotPID()
    {
        pivotThread = new Thread(() ->
        {
            final double ARM_PIVOT_THREAD_WAITING_TIME = 0.005;
            final double kP = 0.005;//0.007
            final double kD = 0.0005; 
            final double kI = 0.00001;
            final double kA = 0.0022;//0.0077;
            final double kF = 0.0;//-0.05;

            double power;            

            Timer pivotTimer = new Timer();
            pivotTimer.start();

            double previousError = 0;
            double currentError; 
            double deltaError = 0; 

            double previousDerivative = 0;
            double currentDerivative;    // in case you want to filter derivative
            double filteredDerivative;
            
            double previousTime = 0;
            
            double deltaTime;
            
            double currentTime;
            double currentAngle;

            double integral = 0;

            double kAPow;
            double kPPow;
            double kDPow;
            double kIPow;

            while(true)
            {
                if(targetAngle == CatzConstants.INVALID_ANGLE)
                {
                    Timer.delay(CatzConstants.CONTROLLER_INPUT_WAIT_TIME);
                }
                else
                {
                    //SmartDashboard.putBoolean("pivot pid state", runPivotPID);
                    currentTime  = pivotTimer.get();
                    currentAngle = getPivotAngle();

                    currentError = targetAngle - currentAngle;
                    
                    /*if(Math.abs(currentError-previousError) < 1.0)
                    {
                       if((armTimer.get() - currentTime) > 0.1)
                       {
                            runPivotPID = false;
                            turnPivot(0.0);
                            setPivotTargetAngle(CatzConstants.INVALID_ANGLE);
                            Thread.currentThread().interrupt();
                       }    
                          
                    }*/

                    //if(runPivotPID == true)
                    //{
                        deltaError = currentError - previousError;
                        deltaTime  = currentTime  - previousTime;

                        integral += deltaTime * currentError;

                        currentDerivative = (deltaError / deltaTime);
                        filteredDerivative = (0.7 * currentDerivative) + (0.3 * previousDerivative);

                        kAPow = 0; //kA * ((getArmExtensionDistance() + 13) * Math.cos(Math.toRadians(currentAngle - 45)));
                        kPPow = kP * currentError;
                        kDPow = kD * filteredDerivative; //currentDerivative
                        kIPow = kI * integral;

                        SmartDashboard.putNumber("ka pow", kAPow);
                        SmartDashboard.putNumber("kp pow", kPPow);
                        SmartDashboard.putNumber("kd pow", kDPow);
                        SmartDashboard.putNumber("ki pow", kIPow);

                        power = kPPow + kIPow + kDPow + kAPow;//ka compensates for angle of arm
                                //arm extension distance + 13 is the distance from pivot to wrist
                                // + kF; //+ (kI * integral)

                        //turnPivot(-power);

                        armPivotMtrCtrlRT.set(-power);

                        SmartDashboard.putNumber("pivot error", currentError);
                    
                        previousError = currentError;
                        previousTime = currentTime;

                        previousDerivative = currentDerivative;
                    
                        Timer.delay(ARM_PIVOT_THREAD_WAITING_TIME);
                    //}
                }
            }
        });
        pivotThread.start();
    }

    public void stopPivotPID()
    {
        pivotThread.interrupt();    
    }

    public void stopArmPID()
    {
        armThread.interrupt();
    }

    public int getArmExtensionMtrCtrlAFirmWare()
    {
        return armExtensionMtrCtrlA.getFirmwareVersion();
    }
}