/*
 *  Author :

 *  Methods : 
 *  Functionality : 
 *   
 *  Revision History : 
 * 
 */
package frc.Mechanisms;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.CatzConstants;

public class CatzDriveTrain 
{     

    private static CANSparkMax drvTrainMtrCtrlLTFrnt;
    private static CANSparkMax drvTrainMtrCtrlLTMidl;
    private static CANSparkMax drvTrainMtrCtrlLTBack;

    private static CANSparkMax drvTrainMtrCtrlRTFrnt;
    private static CANSparkMax drvTrainMtrCtrlRTMidl;
    private static CANSparkMax drvTrainMtrCtrlRTBack;

    private final int DRVTRAIN_LT_FRNT_MC_CAN_ID = 1; 
    private final int DRVTRAIN_LT_MIDL_MC_CAN_ID = 2;
    private final int DRVTRAIN_LT_BACK_MC_CAN_ID = 3;

    private final int DRVTRAIN_RT_FRNT_MC_CAN_ID = 4; 
    private final int DRVTRAIN_RT_MIDL_MC_CAN_ID = 5; 
    private final int DRVTRAIN_RT_BACK_MC_CAN_ID = 6;

    private final int DRVTRAIN_MTR_CTRL_CURRENT_LIMIT = 80; //amps Recommended value from REV Robotics

    private static DifferentialDrive drvTrainDifferentialDrive;

    private static SpeedControllerGroup drvTrainLT;
    private static SpeedControllerGroup drvTrainRT;

     /* **************************************************************************
    * Drive Train Encoder - use spark Max 
    * It attaches to the wheel 
    * The diameter of the wheel is 6 inches 
    *****************************************************************************/

    private static Encoder drvTrainEnc;

    private static final double DRVTRAIN_WHEEL_DIAMETER = 6.0;

    private static double drvTrainEncCountsPerInch = DRVTRAIN_WHEEL_DIAMETER * Math.PI;

    private final int DRVTRAIN_LT_ENCODER_A_DIO_PORT = 0; //TBD
    private final int DRVTRAIN_LT_ENCODER_B_DIO_PORT = 1;
    
    private static DoubleSolenoid drvTrainToClimberShifter;

    private static final int DRVTRAIN_TO_CLIMBER_SOLENOID_PCM_PORT_A = 0;
    private static final int DRVTRAIN_TO_CLIMBER_SOLENOID_PCM_PORT_B = 1;

    private static final double DRVTRAIN_RAMP_RATE = 0.25;

    public CatzDriveTrain() 
    {  
        drvTrainMtrCtrlLTFrnt = new CANSparkMax(DRVTRAIN_LT_FRNT_MC_CAN_ID, MotorType.kBrushless);
        drvTrainMtrCtrlLTMidl = new CANSparkMax(DRVTRAIN_LT_MIDL_MC_CAN_ID, MotorType.kBrushless);
        drvTrainMtrCtrlLTBack = new CANSparkMax(DRVTRAIN_LT_BACK_MC_CAN_ID, MotorType.kBrushless);

        drvTrainMtrCtrlRTFrnt = new CANSparkMax(DRVTRAIN_RT_FRNT_MC_CAN_ID, MotorType.kBrushless);
        drvTrainMtrCtrlRTMidl = new CANSparkMax(DRVTRAIN_RT_MIDL_MC_CAN_ID, MotorType.kBrushless);
        drvTrainMtrCtrlRTBack = new CANSparkMax(DRVTRAIN_RT_BACK_MC_CAN_ID, MotorType.kBrushless);
        
        
        /*drvTrainMtrCtrlLTFrnt.setIdleMode(IdleMode.kBrake);
        drvTrainMtrCtrlLTMidl.setIdleMode(IdleMode.kBrake);
        drvTrainMtrCtrlLTBack.setIdleMode(IdleMode.kBrake);

        drvTrainMtrCtrlRTFrnt.setIdleMode(IdleMode.kBrake);
        drvTrainMtrCtrlRTMidl.setIdleMode(IdleMode.kBrake);
        drvTrainMtrCtrlRTBack.setIdleMode(IdleMode.kBrake);
        */
        
        drvTrainMtrCtrlLTFrnt.setIdleMode(IdleMode.kCoast);
        drvTrainMtrCtrlLTMidl.setIdleMode(IdleMode.kCoast);
        drvTrainMtrCtrlLTBack.setIdleMode(IdleMode.kCoast);

        drvTrainMtrCtrlRTFrnt.setIdleMode(IdleMode.kCoast);
        drvTrainMtrCtrlRTMidl.setIdleMode(IdleMode.kCoast);
        drvTrainMtrCtrlRTBack.setIdleMode(IdleMode.kCoast);
        

        drvTrainMtrCtrlLTFrnt.setOpenLoopRampRate(DRVTRAIN_RAMP_RATE);
        drvTrainMtrCtrlLTMidl.setOpenLoopRampRate(DRVTRAIN_RAMP_RATE);
        drvTrainMtrCtrlLTBack.setOpenLoopRampRate(DRVTRAIN_RAMP_RATE);

        drvTrainMtrCtrlRTFrnt.setOpenLoopRampRate(DRVTRAIN_RAMP_RATE);
        drvTrainMtrCtrlRTMidl.setOpenLoopRampRate(DRVTRAIN_RAMP_RATE);
        drvTrainMtrCtrlRTBack.setOpenLoopRampRate(DRVTRAIN_RAMP_RATE);

        drvTrainMtrCtrlRTFrnt.setSmartCurrentLimit(DRVTRAIN_MTR_CTRL_CURRENT_LIMIT);
        drvTrainMtrCtrlRTMidl.setSmartCurrentLimit(DRVTRAIN_MTR_CTRL_CURRENT_LIMIT);
        drvTrainMtrCtrlRTBack.setSmartCurrentLimit(DRVTRAIN_MTR_CTRL_CURRENT_LIMIT);

        drvTrainMtrCtrlLTFrnt.setSmartCurrentLimit(DRVTRAIN_MTR_CTRL_CURRENT_LIMIT);
        drvTrainMtrCtrlLTMidl.setSmartCurrentLimit(DRVTRAIN_MTR_CTRL_CURRENT_LIMIT);
        drvTrainMtrCtrlLTBack.setSmartCurrentLimit(DRVTRAIN_MTR_CTRL_CURRENT_LIMIT);

        drvTrainLT = new SpeedControllerGroup(drvTrainMtrCtrlLTFrnt, drvTrainMtrCtrlLTMidl, drvTrainMtrCtrlLTBack);
        drvTrainRT = new SpeedControllerGroup(drvTrainMtrCtrlRTFrnt, drvTrainMtrCtrlRTMidl, drvTrainMtrCtrlRTBack);
 
        drvTrainDifferentialDrive = new DifferentialDrive(drvTrainLT, drvTrainRT);
        
        //drvTrainEnc = new Encoder(DRVTRAIN_LT_ENCODER_A_DIO_PORT,  DRVTRAIN_LT_ENCODER_B_DIO_PORT, false, EncodingType.k4X);

        drvTrainToClimberShifter = new DoubleSolenoid(DRVTRAIN_TO_CLIMBER_SOLENOID_PCM_PORT_A, DRVTRAIN_TO_CLIMBER_SOLENOID_PCM_PORT_B); 
    }


    public static void arcadeDrive(double xSpeed, double zRotataion) 
    {
        drvTrainDifferentialDrive.arcadeDrive(xSpeed, zRotataion);
    }

    public void setOpenLoopRampRate(double rampRate)
    {
        drvTrainMtrCtrlLTBack.setOpenLoopRampRate(rampRate);
        drvTrainMtrCtrlLTMidl.setOpenLoopRampRate(rampRate);
        drvTrainMtrCtrlLTFrnt.setOpenLoopRampRate(rampRate);

        drvTrainMtrCtrlRTBack.setOpenLoopRampRate(rampRate);
        drvTrainMtrCtrlRTMidl.setOpenLoopRampRate(rampRate);
        drvTrainMtrCtrlRTFrnt.setOpenLoopRampRate(rampRate);
    }
    
    public static double getDriveTrainEncoderDistance()
    {
        return drvTrainMtrCtrlLTBack.getEncoder().getPosition() / drvTrainEncCountsPerInch;
    }

    public void shiftToClimber()
    {
        drvTrainToClimberShifter.set(Value.kReverse);
    }

    public void shiftToDriveTrain()
    {
        drvTrainToClimberShifter.set(Value.kForward);
    }

    public Value getDriveTrainSolenoidState()
    {
        return drvTrainToClimberShifter.get();
    }

    public void climb(double power)
    {
        drvTrainRT.set(power);
        drvTrainLT.set(power);
    } 
}