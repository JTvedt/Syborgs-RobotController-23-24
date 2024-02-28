package org.firstinspires.ftc.teamcode.subsystem.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

//WHOOOOAAAA SQUIDWARD ON CHAIRRRR TAKE MY HANDS WE'LL MAKE IT I SWEAR OOOOOOOHHHH IT'S 3 AM IN THE MORNING BY JOVE HOW LONG CAN I MAKE THIS AND WILL ANYONE EVER FIND IT WILL ANYONE READ IT IS IT ALONE HERE JUST TO TALK ABOUT CRACKERS AND HOW MUCH I LOVE SALTEEN CRACKERS OR HOW I SPELT THAT WRONG OR HOW I'M HOLDING DOWN SHIFT AND NOT USING CAPS LOCKS OR HOW THIS CODE MOST LIKELY WON'T WORK AND I'VE WASTED MY TIME OR PERHAPS EVEN THE FACT THAT I'M USING MY SISTERS LAPTOP BECAUSE WINDOWS SUCKS FOR CODING AND I DON'T KNOW HOW JEFFERY DOES IT OR PERHAPS THAT I HAVE A TONE OF WORK DUE IN 4 HOURS AND I HAVEN'T STARTED IT AND HAVE TO TAKE A SHOWER IN 2 AND 1/2 TO MAKE IT TO SCHOOL EVEN THOUGH I'M NOT GONNA LEARN ANYTHING BECAUSE MELE'S GONNA POST A VIDEO, CHIPES GONNA POST A READING, I DON'T HAVE A MATH TEST THIS WEEK, AND DEROSA WELL OKAY THAT MIGHT BE AN ACCEPTION ANYYYYYYWAYYYS I'M GONNA STOP NOW FOR RISK OF THIS GETTING TOO LONG LMAO LET'S SEE IF ANYONE READS THIS EVER OR IF I FIND IT AGAIN IN LIKE 2 YEARS LOL

public class OdoImpl implements Odometry {
    //Odometers
    private final DcMotor ER;
    private final DcMotor EL;
    private final DcMotor EB;

    //Old ER position
    int OERP;
    int OELP;
    int OEBP;

    //Change in encoder distance
    int dn1; //Distance traveled on EL
    int dn2; //Distance for ER
    int dn3; //Distance for EB

    //constants for calculating stuff
    final static double L = 16.0; //Distance from EL to ER
    final static double B = 22.776092; //(Update, I do) This is where I'd put the measurement for midpoint of ER & EL IF I HAD ONE
    final static double R = 4.8; //Wheel radius in cm
    final static double N = 2000; //Odo ticks per revolution
    final static double cm_per_tick = 2.0 * Math.PI * R / N;

    // used for finding distance traveled between positions (real)
    public int CERP = 0; //Current ER position
    public int CELP = 0;
    public int CEBP = 0;

    //Initial values of x,y, and theta
    double x = 0.0;
    double y = 0.0;
    double h = 0.0;

    public OdoImpl(HardwareMap hardwareMap) {
        ER = hardwareMap.dcMotor.get("FR");
        EL = hardwareMap.dcMotor.get("FL");
        EB = hardwareMap.dcMotor.get("BR");
    }

    public void update() {
            OERP = CERP;
            OELP = CELP;
            OEBP = CEBP;

            CERP = -ER.getCurrentPosition();
            CELP = -EL.getCurrentPosition();
            CEBP = EB.getCurrentPosition();

            dn1 = CELP - OELP;
            dn2 = CERP - OERP;
            dn3 = CEBP - OEBP;

            //Actual travel distance for each encoder accounting for change in x,y, and theta
        double dx = cm_per_tick * ((dn1 + dn2) / 2.0);
        double dtheta = cm_per_tick * ((dn2 - dn1) / L);
        double dy = cm_per_tick * (dn3 - (dn2 - dn1) * B / L);

            //update the theta so that it has the correct dtheta angle going into the change
            double theta = h + (dtheta / 2.0); // basically "account for the change in angle grrr"

            //Update position and angle
            x += dx * Math.cos(theta) - dy * Math.sin(theta);
            y += dx * Math.sin(theta) + dy * Math.cos(theta);
            h += dtheta;
        }

    public double getX() {
        update();
        return x;
    }
    public double getY() {
        update();
        return y;
    }
    public double getAngle() {
        update();
        return h;
    }
}
