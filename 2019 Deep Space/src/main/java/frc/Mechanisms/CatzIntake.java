/**
 * Author : Jeffrey Li
 *  Methods : getCargo, relaseCargo, rotateWrist, stopWrist, closeCargoClamp, openCargoClamp
 *  Functionality : gets cargo and release cargo, start and stop wrist
 *    
 *  02-13-19
 * revision history: changed enum solenoid to kForward JL
 */
package frc.Mechanisms;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.AnalogInput;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.CatzConstants;

public class CatzIntake 
{
    private static WPI_VictorSPX intakeRollerMtrCtrl;
    private static WPI_TalonSRX intakeWristMtrCtrl;

    private static AnalogInput intakeWristEnc;

    public static DoubleSolenoid hatchEjectSolenoid;
    public static DoubleSolenoid cargoClampSolenoid;

    private final int INTAKE_ROLLER_MC_CAN_ID = 31;
    private final int INTAKE_WRIST_MC_CAN_ID = 30;
   
    private final int HATCH_EJECT_PCM_PORT_A = 2;
    private final int HATCH_EJECT_PCM_PORT_B = 3;
    private final int CARGO_CLAMP_PCM_PORT_A = 4;
    private final int CARGO_CLAMP_PCM_PORT_B = 5;
    
    private final int INTAKE_WRIST_ENC_MAX_VOLTAGE = 5;
    private final int INTAKE_WRIST_ENCODER_ANALOG_PORT = 0;

    private final double WRIST_ANGLE_TOLERANCE = 0.0; //TBD

    private final double WRIST_ANGLE_MAX = 0; //TBD

    private double WRIST_VOLTAGE_OFFSET; //TBD use to make one end point of the encoder read 0 voltage

    public CatzIntake() 
    {

        intakeRollerMtrCtrl = new WPI_VictorSPX(INTAKE_ROLLER_MC_CAN_ID);
        intakeWristMtrCtrl  = new WPI_TalonSRX(INTAKE_WRIST_MC_CAN_ID);
        
        hatchEjectSolenoid = new DoubleSolenoid(HATCH_EJECT_PCM_PORT_A,HATCH_EJECT_PCM_PORT_B);
        cargoClampSolenoid = new DoubleSolenoid(CARGO_CLAMP_PCM_PORT_A,CARGO_CLAMP_PCM_PORT_B);

        intakeWristEnc = new AnalogInput(INTAKE_WRIST_ENCODER_ANALOG_PORT);
        
        if(CatzConstants.USING_COMPETITION_ROBOT)
        {
            WRIST_VOLTAGE_OFFSET = 0; // value for competition robot
        }
        else
        {
            WRIST_VOLTAGE_OFFSET = 0; // value for robot 2
        }
    }

    public void closeCargoClamp() 
    {
        cargoClampSolenoid.set(Value.kReverse); // might be kForward
    }

    public void openCargoClamp() 
    {
        cargoClampSolenoid.set(Value.kForward); // might be kReverse 
    }
       
    
    public void hatchEject()
    {
        hatchEjectSolenoid.set(Value.kReverse);
    }

    public void hatchDeployed()
    {
        hatchEjectSolenoid.set(Value.kForward);
    }

    public void getCargo(double power) 
    { 
        intakeRollerMtrCtrl.set(power);
    }

    public void releaseCargo(double power) 
    {
        intakeRollerMtrCtrl.set(-power); 
    }
    
    public void rotateWrist(double power) 
    {
        intakeWristMtrCtrl.set(power);
    }

    public double getWristAngle()
    {
        return ( (intakeWristEnc.getVoltage()  + WRIST_VOLTAGE_OFFSET) /INTAKE_WRIST_ENC_MAX_VOLTAGE) * 360.0;
    }

    public void moveWristThread(double targetAngle, double power, double timeOut)
    {
        final double WRIST_THREAD_WAITING_TIME = 0.005;

        Timer threadTimer = new Timer();
        threadTimer.start();

        Thread wristThread = new Thread(() ->
        {
            double currentAngle = getWristAngle();

            double errorAngle = Math.abs(targetAngle-currentAngle);

            double upperLimit = targetAngle + WRIST_ANGLE_TOLERANCE;
            double lowerLimit = targetAngle - WRIST_ANGLE_TOLERANCE;

            if (errorAngle < WRIST_ANGLE_MAX/2.0)
            {  
                intakeWristMtrCtrl.set(power);
            } 
            else if(errorAngle > WRIST_ANGLE_MAX/2.0) 
            {
                intakeWristMtrCtrl.set(-power);
            }

            while(!Thread.interrupted()) 
            {
                currentAngle = getWristAngle(); //update the currentAngle

                if((lowerLimit < currentAngle && upperLimit > currentAngle) || threadTimer.get() > timeOut) 
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
            
            while((Math.abs(targetAngle - currentAngle) < WRIST_ANGLE_TOLERANCE) && (currentTime < timeOut)) 
            {                 

                currentError = targetAngle - currentAngle;
                
                deltaError = currentError - previousError;
                deltaTime  = currentTime - previousTime;

                power = kP * currentError +
                        kD * (deltaError / deltaTime);

                rotateWrist(power);

                previousError = currentError;
                previousTime  = currentTime;

                Timer.delay(WRIST_THREAD_WAITING_TIME);

                currentAngle = getWristAngle();
                currentTime  = threadTimer.get();

            }
            rotateWrist(0);;
            Thread.currentThread().interrupt();
        });
        wristThread.start(); 
    }
}
