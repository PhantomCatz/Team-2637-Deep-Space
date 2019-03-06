
/**

 * Author : Jeffrey Li

 *  Methods : getCargo, relaseCargo, rotateWrist, stopWrist, closeCargoClamp, openCargoClamp

 *  Functionality : gets cargo and release cargo, start and stop wrist

 *    

 *  02-13-19

 * revision history: changed enum solenoid to kForward JL

 */

package frc.Mechanisms;

import javax.lang.model.util.ElementScanner6;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.AnalogInput;
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

    public static DoubleSolenoid hatchEjectSolenoid;
    public static DoubleSolenoid cargoClampSolenoid;

    private final int INTAKE_ROLLER_MC_CAN_ID = 31;
    private final int INTAKE_WRIST_MC_CAN_ID = 30;

    private final int HATCH_EJECT_PCM_PORT_A = 2;
    private final int HATCH_EJECT_PCM_PORT_B = 3;

    private final int CARGO_CLAMP_PCM_PORT_A = 4;
    private final int CARGO_CLAMP_PCM_PORT_B = 5;

    private final double INTAKE_WRIST_ENC_MAX_VOLTAGE     = 5.0;
    private final int    INTAKE_WRIST_ENCODER_ANALOG_PORT = 0;

    private final double WRIST_ANGLE_TOLERANCE = 2.0;      // TBD
    private final double WRIST_ANGLE_UP_MAX    = -90.0;    // 0 Deg is when intake is parallel to ground
    private final double WRIST_ANGLE_DN_MAX    = +90.0;   

    private final double WRIST_SN02_ENCODER_REF_ANGLE      = 343.2;  // This is the angle when intake is parrallel to ground
    private final double WRIST_SN02_ENCODER_REF_VOLTAGE    = ((WRIST_SN02_ENCODER_REF_ANGLE / 360.0) * INTAKE_WRIST_ENC_MAX_VOLTAGE);
    private final double WRIST_SN02_ENCODER_VOLTAGE_OFFSET = INTAKE_WRIST_ENC_MAX_VOLTAGE - WRIST_SN02_ENCODER_REF_VOLTAGE;
    private final double WRIST_SN02_ENCODER_ANGLE_OFFSET   = ( WRIST_SN02_ENCODER_VOLTAGE_OFFSET / 5.0 ) * 360.0;
     
    private double WRIST_SN02_ENCODER_MIN_VOLTAGE    = ( (WRIST_ANGLE_DN_MAX - WRIST_SN02_ENCODER_ANGLE_OFFSET)/360) * 5 ;

    public CatzIntake()
    {
        intakeRollerMtrCtrl = new WPI_VictorSPX(INTAKE_ROLLER_MC_CAN_ID);
        intakeWristMtrCtrl = new WPI_TalonSRX(INTAKE_WRIST_MC_CAN_ID);

        intakeRollerMtrCtrl.setNeutralMode(NeutralMode.Brake);
        intakeWristMtrCtrl.setNeutralMode(NeutralMode.Brake);

        hatchEjectSolenoid = new DoubleSolenoid(HATCH_EJECT_PCM_PORT_A, HATCH_EJECT_PCM_PORT_B);
        cargoClampSolenoid = new DoubleSolenoid(CARGO_CLAMP_PCM_PORT_A, CARGO_CLAMP_PCM_PORT_B);

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
        double wristAngle = this.getWristAngle();
        System.out.println(power + ", " + wristAngle);
        if(power < 0.0)
        {
            // Wrist is being commanded Up (Increasing Angle)
            if(wristAngle < WRIST_ANGLE_UP_MAX)
            {
                System.out.println("stop going up");
                intakeWristMtrCtrl.set(0);
            }
            else
            {
                System.out.println("going up");
                intakeWristMtrCtrl.set(0.7 * power);
            }
        }
        else if(power > 0.0)
        {
            // Wrist is being commanded Down (Increasing Angle)
            if(wristAngle > WRIST_ANGLE_DN_MAX)
            {
                System.out.println("stop going down");
                intakeWristMtrCtrl.set(0);
            }
            else
            {
                System.out.println("going down");
                intakeWristMtrCtrl.set(0.3 * power);
            }
        }
    }


    
    public double getWristAngle()
    {
        double encVoltage = intakeWristEnc.getVoltage();
        double adjVoltage = 0.0;
        double wristAngle = 999.0;

        System.out.println("Enc voltage : " + encVoltage +"Min vol : " + WRIST_SN02_ENCODER_MIN_VOLTAGE);
     
        if (encVoltage < WRIST_SN02_ENCODER_MIN_VOLTAGE)
        {
            //System.out.println("*** Rollover *****");
            adjVoltage = encVoltage + WRIST_SN02_ENCODER_VOLTAGE_OFFSET;
            wristAngle = (adjVoltage / INTAKE_WRIST_ENC_MAX_VOLTAGE) * 360.0;            
            System.out.println("EV: " + encVoltage);
            System.out.println("AV: " + adjVoltage);
            System.out.println("WA: " + wristAngle);
           
        }
        else
        {
            wristAngle = (encVoltage / INTAKE_WRIST_ENC_MAX_VOLTAGE) * 360.0;
            System.out.println("Raw Wrist Angle : " + wristAngle);
            if(wristAngle > WRIST_SN02_ENCODER_REF_ANGLE)
            {
                wristAngle = wristAngle - WRIST_SN02_ENCODER_REF_ANGLE;
            }
            else if(wristAngle > 120.0) 
            {
                wristAngle = wristAngle - 360.0;
            }
            else
            {
                wristAngle = wristAngle + WRIST_SN02_ENCODER_ANGLE_OFFSET;
            } 

        }
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
}
