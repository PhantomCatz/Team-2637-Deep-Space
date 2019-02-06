package frc.Mechanisms;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;

/*
 *  Author : Derek

 *  Methods : intake, outtake, rotateWrist, wristEncoderCounts, closeClamp, openClamp, eject hatch
 *  Functionality : controls all the motors and pnuematics in the intake
 *   
 *  Revision History : 
 *  02-01-19 Initial code set up DD
 *  02-05-19 Added Solenoids for hatch and cargo pick up
 * 
 */
public class CatzIntake {

    private static WPI_VictorSPX intakeRoller;
    private final int INTAKE_ROLLER_MC_CAN_ID = 31;

    private static WPI_TalonSRX intakeWrist;
    private final int INTAKE_WRIST_MC_CAN_ID = 30;
   
    public static Solenoid solenoidHatchEject; 
    public static Solenoid solenoidCargoClamp; // Opening and Closing

    private final int CARGO_CLAMP_PCM_PORT_A = 1;
    private final int CARGO_CLAMP_PCM_PORT_B = 2;

    private final int HATCH_EJECT_PCM_PORT_A = 3;
    private final int HATCH_EJECT_PCM_PORT_B = 4;

    public static boolean cargoOpen = false;

    public static SolenoidState cargoState = SolenoidState.Closed;

    public enum SolenoidState
    {
        Open(true), Closed(false);
        private boolean state;
            
        SolenoidState(boolean state)
        {
            this.state = state;
        }
            
        public boolean getState() 
        {
            return state;	
        }
    }

    public CatzIntake() 
    {
        intakeRoller = new WPI_VictorSPX(INTAKE_ROLLER_MC_CAN_ID);
        intakeWrist = new WPI_TalonSRX(INTAKE_WRIST_MC_CAN_ID);
        
        solenoidHatchEject = new Solenoid(HATCH_EJECT_PCM_PORT_A, HATCH_EJECT_PCM_PORT_B);
        solenoidCargoClamp = new Solenoid(CARGO_CLAMP_PCM_PORT_A, CARGO_CLAMP_PCM_PORT_B);
    }

    public void closeClamp() 
    {
        cargoState = SolenoidState.Closed;
        solenoidCargoClamp.set(cargoState.getState());
        //printOutDebugData("Cargo Clamp set to Closed");
    }

    public void openClamp() 
    {
        cargoState = SolenoidState.Open;
        solenoidCargoClamp.set(cargoState.getState());
        
        //printOutDebugData("Cargo Clamp set to Open");
    }
    public void ejectHatch()
    {
        solenoidHatchEject.set(true);  // ejects hatch
        Timer.delay(0.005);            // waits to make sure hatch is kicked off
        solenoidHatchEject.set(false); 
    }
          
    public void intake(double speed) { 
        intakeRoller.set(speed);
    }
    public void outtake(double speed)
    {
        intakeRoller.set(-speed);
    }
    public void rotateWrist(double speed)
    {
        intakeWrist.set(speed);
    }
    public static double wristEncoderCounts()
    {
        return intakeWrist.getSelectedSensorPosition();
    }
}