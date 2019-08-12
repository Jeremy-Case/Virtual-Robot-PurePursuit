package PurePursuit;

import RobotUtilities.MovementVars;
import com.company.ComputerDebugging;
import com.company.FloatPoint;
import org.opencv.core.Point;
import RobotUtilities.MovementVars.*;

import java.util.*;


import static com.company.Robot.*;


public class PurePursuit
{
    
    //will possibly center the search for goal point around project location in the future
    /*public Point projectedLocation()
    {


    }
    */
    //determine how far a point is along the path
    public double distanceAlongPath(Point location, Point p)
    {

        return Math.sqrt(Math.pow(location.x-p.x,2)+Math.pow(location.y-p.y,2));

    }
//returns the point that pure pursuit will move to by looking at the line segments and finding the intersections of a circle centered at the robot with radius of the look ahead distance
    public double[] goalPoint(int iRel, ArrayList<Point> p, double lookAheadDistance)
    {

        try {
            double x1 = p.get(iRel-1).x - worldXPosition;
            double y1 = p.get(iRel-1).y - worldYPosition;
            double x2 = p.get(iRel).x - worldXPosition;
            double y2 = p.get(iRel).y - worldYPosition;
            double r = lookAheadDistance;
            double dx = x2 - x1;
            double dy = y2 - y1;
            double dr = Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));
            double D = (x1 * y2) - (x2 * y1);
            int sgn = 1;
            if (dy < 0) {
                sgn = -1;
            }
            //if the last segment is selected and there is no goal on that segment go to the end of the path
            if(iRel==p.size()-2&&Math.sqrt(Math.pow(y2,2)+Math.pow(x2,2))<=lookAheadDistance-1)
                {return new double[]{p.get(iRel).x, p.get(iRel).y, iRel};}
            double radicand = Math.pow(r, 2) * Math.pow(dr, 2) - Math.pow(D, 2);
            //if no points on the line intersect the circle where the robot is looking move to the next line 
            if(radicand<0)
                {return goalPoint(iRel-1,p,lookAheadDistance); }
            //finding the 2 intersections
            double sqrt = Math.sqrt(radicand);
            double g1x = (D * dy + sgn * dx * sqrt) / Math.pow(dr, 2);
            double g2x = (D * dy - sgn * dx * sqrt) / Math.pow(dr, 2);
            double g1y = (-D * dx + Math.abs(dy) * sqrt )/ Math.pow(dr, 2);
            double g2y = (-D * dx - Math.abs(dy) * sqrt )/ Math.pow(dr, 2);
            double xLower=x1;
            double xUpper=x2;
            if(x2<x1)
            {
                xLower=x2;
                xUpper=x1;
            }

            //determines which of the points is further along the path then for that intersection if it falls on the line segment that is the goal point otherwise move to the next line segment
            if (distanceAlongPath(new Point(g1x+worldXPosition, g1y+worldYPosition), p.get(iRel)) < distanceAlongPath(new Point(g2x+worldXPosition, g2y+worldYPosition), p.get(iRel ))) {
                
                if(g1x>xLower&&g1x<xUpper)
                {

                    return new double[]{g1x + worldXPosition, g1y + worldYPosition,iRel};

                }
                return goalPoint(iRel-1,p,lookAheadDistance);
            } else {


                if(g2x>xLower&&g2x<xUpper)
                {

                    return new double[]{g2x + worldXPosition, g2y + worldYPosition,iRel};

                }
                
                    return goalPoint(iRel-1,p,lookAheadDistance);}

        }
        //if there are no intersections on any line go to the first point on the path or if the last segment is reached move directly to the endpoint
        catch(IndexOutOfBoundsException e)
            {return new double[]{p.get(iRel).x,p.get(iRel).y,iRel};}



    }
    
    //move the robot to to a specified point
    public void goToPosition(Point goalPoint, double movementSpeed,ArrayList<Point> p,int iRel)
    {

        double distanceToTarget = Math.hypot(goalPoint.x-worldXPosition,goalPoint.y-worldYPosition);
        double absoluteAngleToTarget = Math.atan2(goalPoint.y-worldYPosition,goalPoint.x-worldXPosition);
        double relativeAngleToPoint = absoluteAngleToTarget - (MathFunctions.angleWrap(worldAngle_rad-Math.PI/2));
        double relativeXToPoint = Math.cos(relativeAngleToPoint)*distanceToTarget;
        double relativeYToPoint = Math.sin(relativeAngleToPoint)*distanceToTarget;
        double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint)+Math.abs(relativeYToPoint));
        double movementYPower = relativeYToPoint / (Math.abs(relativeYToPoint)+Math.abs(relativeXToPoint));

        while(relativeAngleToPoint>Math.PI)
        {
            relativeAngleToPoint-=2*Math.PI;
        }
        while(relativeAngleToPoint<-Math.PI)
        {
            relativeAngleToPoint+=2*Math.PI;
        }

        if(iRel<p.size()-2) {
            try {
                ComputerDebugging.sendKeyPoint(new FloatPoint(p.get(iRel).x, p.get(iRel).y));
                double turnPowerToGoal = (relativeAngleToPoint - Math.PI / 2) / Math.sqrt(Math.abs(relativeAngleToPoint - Math.PI / 2))*2;
                LineSegment nextSegment = new LineSegment(p.get(iRel).x, p.get(iRel).y, p.get(iRel+1).x, p.get(iRel+1).y);
                double distanceToPoint = distanceAlongPath(new Point(worldXPosition, worldYPosition), p.get(iRel));
                LineSegment relevantSegment = new LineSegment(p.get(iRel -1).x, p.get(iRel-1).y, p.get(iRel).x, p.get(iRel).y);
                double goalPercentage = distanceToPoint / (relevantSegment.getMagnitude()) +.2;
                if(goalPercentage>1)
                    {goalPercentage=1;}
                else if(goalPercentage<0)
                    {goalPercentage=0;}
                double turnPowerToNextPoint = (nextSegment.getAngle()-worldAngle_rad ) / Math.sqrt(Math.abs(nextSegment.getAngle()-worldAngle_rad))*2;
                double turnPowerAbsolute = turnPowerToGoal * goalPercentage + turnPowerToNextPoint * (1 - goalPercentage);
                if(MovementVars.movement_turn-turnPowerAbsolute>.02)
                {
                    turnPowerAbsolute=MovementVars.movement_turn-.02;
                }else if(turnPowerAbsolute-MovementVars.movement_turn>.02)
                {
                    turnPowerAbsolute=MovementVars.movement_turn+.02;
                }
                MovementVars.movement_turn = turnPowerAbsolute;

            } catch (Exception e) {
                MovementVars.movement_turn = (relativeAngleToPoint - Math.PI / 2) / Math.sqrt(Math.abs(relativeAngleToPoint - Math.PI / 2)) ;
            }
        }else
        {MovementVars.movement_turn = (relativeAngleToPoint - Math.PI/2 ) / Math.sqrt(Math.abs(relativeAngleToPoint - Math.PI/2 )) /3; }
        MovementVars.movement_y = movementYPower;
        MovementVars.movement_x = movementXPower;

    }

    //follow path with pure pursuit
    boolean getBreak=false;

    public void followPath(ArrayList<Point> p, double lookAheadDistance, double speed)
    {

        p.add(p.get(p.size() - 1));
        for (int i = 0; i < p.size() - 2; i++)
        {

            ComputerDebugging.sendLine(new FloatPoint(p.get(i).x, p.get(i).y), new FloatPoint(p.get(i + 1).x, p.get(i + 1).y));

        }
        //add a point for calculations on the last segment

        if(!(worldXPosition>p.get(p.size()-2).x-30&&worldXPosition<p.get(p.size()-2).x+30&&worldYPosition>p.get(p.size()-2).y-30&&worldYPosition<p.get(p.size()-2).y+30)&&!getBreak)
        {


            double[] d  = goalPoint(p.size() - 2, p, lookAheadDistance);
            Point g = new Point(d[0],d[1]);
            goToPosition(g, speed,p,(int) d[2]);
            ComputerDebugging.sendKeyPoint(new FloatPoint(g.x,g.y));
        }else
        {


                MovementVars.movement_x=0;
                MovementVars.movement_y=0;
                MovementVars.movement_turn=0;
                getBreak=true;

        }
    }

}
