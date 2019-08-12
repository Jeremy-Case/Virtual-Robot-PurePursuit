package teamcode;

import PurePursuit.PurePursuit;
import RobotUtilities.MovementVars;
import com.company.ComputerDebugging;
import com.company.FloatPoint;
import com.company.Robot;
import org.opencv.core.Point;

import java.util.ArrayList;

import static com.company.Robot.worldXPosition;
import static com.company.Robot.worldYPosition;

public class MyOpMode extends OpMode
{
    Robot robot = new Robot();
    PurePursuit purePursuit = new PurePursuit();
    @Override
    public void init()
    {

        worldXPosition=20;
        worldYPosition=50;
        Robot.worldAngle_rad=0;

        //purePursuit.followPath(p,100,1);


    }

    boolean found=false;
    @Override
    public void loop()
    {


        ArrayList<Point> p = new ArrayList<Point>();
        p.add(new Point(50,30));
        p.add(new Point(320,140));
        p.add(new Point(150,200));
        p.add(new Point(300,300));

        purePursuit.followPath(p, 50, .01);




    }
}
