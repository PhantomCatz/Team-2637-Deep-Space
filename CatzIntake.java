package frc.Mechanisms;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.Solenoid;

//        Header
public class CatzIntake {

    private static WPI_VictorSPX intakeRoller;
    private final int INTAKE_ROLLER_MC_CAN_ID = 31;

    private static WPI_TalonSRX intakeWrist;
    private final int INTAKE_WRIST_MC_CAN_ID = 30;
   
    public static Solenoid solenoidHatchEject;
    public static Solenoid solenoidCargoClamp; // Opening and Closing

    private final int CARGO_CLAMP_PCM_PORT_NUMBER = 2;
    private final int HATCH_EJECT_PCM_PORT_NUMBER = 4;

	public static boolean HatchDeployed = false; 
    public static boolean CargoOpen = false;

    public static SolenoidState hatchState = SolenoidState.Closed;
    public static SolenoidState cargoState = SolenoidState.Closed;

public enum SolenoidState {
	    Open(true), Closed(false);
		
	private boolean state;
		
		SolenoidState(boolean state){
			this.state = state;
		}
		
	public boolean getState() {
			return state;
		}
    }

    public CatzIntake() {

        intakeRoller = new WPI_VictorSPX(INTAKE_ROLLER_MC_CAN_ID);
        intakeWrist = new WPI_TalonSRX(INTAKE_WRIST_MC_CAN_ID);
        
        solenoidHatchEject = new Solenoid(HATCH_EJECT_PCM_PORT_NUMBER);
        solenoidCargoClamp = new Solenoid(CARGO_CLAMP_PCM_PORT_NUMBER);
       }

      public void closeClamp() {
    cargoState = SolenoidState.Closed;
    solenoidCargoClamp.set(cargoState.getState());
    //printOutDebugData("Cargo Clamp set to Closed");
      }

    public void openClamp() {
        cargoState = SolenoidState.Open;
        solenoidCargoClamp.set(cargoState.getState());
        //printOutDebugData("Cargo Clamp set to Open");
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
