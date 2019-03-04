package frc.Vision;


//import frc.Vision.*;

public class VisionObject implements java.io.Serializable
{
    public static final long serialVersionUID = 2637L;

    //Variables have to match the form of the those in the json string
    private int V;
    private double H;
    private double D;

    public VisionObject(int validFlag, double heading, double distance)
    {
        this.V = validFlag;
        this.H = heading;
        this.D = distance;
    }

    public boolean isValid()
    {
        return V == 1;
    }

    public void setValidFlag(int validFlag)
    {
        this.V = validFlag;
    }

    public double getHeading()
    {
        return this.H;
    }

    public void setHeading(double heading)
    {
        this.H = heading;
    }

    public double getDistance()
    {
        return this.D;
    }

    public void setDistance(double distance)
    {
        this.D = distance;
    }

    public String toString()
    {
        return  "(V: " + V + 
                "|H: " + H + 
                "|D: " + D + ")";
    }
}