using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace AgOpenGPS.Classes
{
    public class CABRobot
    {
        private readonly FormGPS mf;

        //steer, pivot, and ref indexes
        //private int sA, sB, C, pA, pB;
        //private int rA, rB;

        public double distanceFromCurrentLineSteer, distanceFromCurrentLinePivot;
        public double steerAngleGu, rEastSteer, rNorthSteer, rEastPivot, rNorthPivot;

        public double inty, xTrackSteerCorrection = 0;
        public double steerHeadingError, steerHeadingErrorDegrees;

        public double distSteerError, lastDistSteerError, derivativeDistError;

        public double pivotDistanceError, stanleyModeMultiplier;

        //public int modeTimeCounter = 0;

        //derivative counter
        private int counter;


       
        
        /// <summary>
        /// NEEDED FOR ROBOT
        /// </summary>
        private (short,short) DoSteerAngleCalc()
        {
            if (mf.isReverse) steerHeadingError *= -1;
            //Overshoot setting on Stanley tab
            steerHeadingError *= mf.vehicle.stanleyHeadingErrorGain;

            double sped = Math.Abs(mf.avgSpeed);
            if (sped > 1) sped = 1 + 0.277 * (sped - 1);
            else sped = 1;
            double XTEc = Math.Atan((distanceFromCurrentLineSteer * mf.vehicle.stanleyDistanceErrorGain)
                / (sped));

            xTrackSteerCorrection = (xTrackSteerCorrection * 0.5) + XTEc * (0.5);

            //derivative of steer distance error
            distSteerError = (distSteerError * 0.95) + ((xTrackSteerCorrection * 60) * 0.05);
            if (counter++ > 5)
            {
                derivativeDistError = distSteerError - lastDistSteerError;
                lastDistSteerError = distSteerError;
                counter = 0;
            }

            steerAngleGu = glm.toDegrees((xTrackSteerCorrection + steerHeadingError) * -1.0);

            if (Math.Abs(distanceFromCurrentLineSteer) > 0.5) steerAngleGu *= 0.5;
            else steerAngleGu *= (1 - Math.Abs(distanceFromCurrentLineSteer));

            //pivot PID
            pivotDistanceError = (pivotDistanceError * 0.6) + (distanceFromCurrentLinePivot * 0.4);
            //pivotDistanceError = Math.Atan((distanceFromCurrentLinePivot) / (sped)) * 0.2;
            //pivotErrorTotal = pivotDistanceError + pivotDerivative;

            if (mf.avgSpeed > 1
                && mf.isAutoSteerBtnOn
                && Math.Abs(derivativeDistError) < 1
                && Math.Abs(pivotDistanceError) < 0.25)
            {
                //if over the line heading wrong way, rapidly decrease integral
                if ((inty < 0 && distanceFromCurrentLinePivot < 0) || (inty > 0 && distanceFromCurrentLinePivot > 0))
                {
                    inty += pivotDistanceError * mf.vehicle.stanleyIntegralGainAB * -0.03;
                }
                else
                {
                    inty += pivotDistanceError * mf.vehicle.stanleyIntegralGainAB * -0.01;
                }

                //integral slider is set to 0
                if (mf.vehicle.stanleyIntegralGainAB == 0) inty = 0;
            }
            else inty *= 0.7;

            if (mf.isReverse) inty = 0;

            if (mf.ahrs.imuRoll != 88888)
                steerAngleGu += mf.ahrs.imuRoll;

            if (steerAngleGu < -mf.vehicle.maxSteerAngle) steerAngleGu = -mf.vehicle.maxSteerAngle;
            else if (steerAngleGu > mf.vehicle.maxSteerAngle) steerAngleGu = mf.vehicle.maxSteerAngle;

            //used for smooth mode 
            mf.vehicle.modeActualXTE = (distanceFromCurrentLinePivot);

            //Convert to millimeters from meters
            mf.guidanceLineDistanceOff = (short)Math.Round(distanceFromCurrentLinePivot * 1000.0, MidpointRounding.AwayFromZero);
            mf.guidanceLineSteerAngle = (short)(steerAngleGu * 100);

            return ((short)Math.Round(distanceFromCurrentLinePivot * 1000.0, MidpointRounding.AwayFromZero), (short)(steerAngleGu * 100));
        }

        /// <summary>
        /// Function to calculate steer angle for AB Line Segment only
        /// No curvature calc on straight line
        /// NEEDED FOR ROBOT!
        /// </summary>
        /// <param name="curPtA"></param>
        /// <param name="curPtB"></param>
        /// <param name="pivot"></param>
        /// <param name="steer"></param>
        /// <param name="isValid"></param>
        public (short,short) StanleyGuidanceABLine(vec3 curPtA, vec3 curPtB, vec3 pivot, vec3 steer)
        {
            //get the pivot distance from currently active AB segment   ///////////  Pivot  ////////////
            double dx = curPtB.easting - curPtA.easting;
            double dy = curPtB.northing - curPtA.northing;
            if (Math.Abs(dx) < Double.Epsilon && Math.Abs(dy) < Double.Epsilon) return(0,0);

            //save a copy of dx,dy in youTurn
            mf.yt.dxAB = dx; mf.yt.dyAB = dy;

            //how far from current AB Line is fix
            distanceFromCurrentLinePivot = ((dy * pivot.easting) - (dx * pivot.northing) + (curPtB.easting
                        * curPtA.northing) - (curPtB.northing * curPtA.easting))
                            / Math.Sqrt((dy * dy) + (dx * dx));

            if (!mf.ABLine.isHeadingSameWay)
                distanceFromCurrentLinePivot *= -1.0;

            mf.ABLine.distanceFromCurrentLinePivot = distanceFromCurrentLinePivot;
            double U = (((pivot.easting - curPtA.easting) * dx)
                            + ((pivot.northing - curPtA.northing) * dy))
                            / ((dx * dx) + (dy * dy));

            rEastPivot = curPtA.easting + (U * dx);
            rNorthPivot = curPtA.northing + (U * dy);

            mf.ABLine.rEastAB = rEastPivot;
            mf.ABLine.rNorthAB = rNorthPivot;

            //get the distance from currently active AB segment of steer axle //////// steer /////////////
            vec3 steerA = new vec3(curPtA);
            vec3 steerB = new vec3(curPtB);


            //create the AB segment to offset
            steerA.easting += (Math.Sin(steerA.heading + glm.PIBy2) * (inty));
            steerA.northing += (Math.Cos(steerA.heading + glm.PIBy2) * (inty));

            steerB.easting += (Math.Sin(steerB.heading + glm.PIBy2) * (inty));
            steerB.northing += (Math.Cos(steerB.heading + glm.PIBy2) * (inty));

            dx = steerB.easting - steerA.easting;
            dy = steerB.northing - steerA.northing;

            if (Math.Abs(dx) < Double.Epsilon && Math.Abs(dy) < Double.Epsilon) return(0,0);

            //how far from current AB Line is fix
            distanceFromCurrentLineSteer = ((dy * steer.easting) - (dx * steer.northing) + (steerB.easting
                        * steerA.northing) - (steerB.northing * steerA.easting))
                            / Math.Sqrt((dy * dy) + (dx * dx));

            if (!mf.ABLine.isHeadingSameWay)
                distanceFromCurrentLineSteer *= -1.0;

            // calc point on ABLine closest to current position - for display only
            U = (((steer.easting - steerA.easting) * dx)
                            + ((steer.northing - steerA.northing) * dy))
                            / ((dx * dx) + (dy * dy));

            rEastSteer = steerA.easting + (U * dx);
            rNorthSteer = steerA.northing + (U * dy);

            double steerErr = Math.Atan2(rEastSteer - rEastPivot, rNorthSteer - rNorthPivot);
            steerHeadingError = (steer.heading - steerErr);
            //Fix the circular error
            if (steerHeadingError > Math.PI)
                steerHeadingError -= Math.PI;
            else if (steerHeadingError < -Math.PI)
                steerHeadingError += Math.PI;

            if (steerHeadingError > glm.PIBy2)
                steerHeadingError -= Math.PI;
            else if (steerHeadingError < -glm.PIBy2)
                steerHeadingError += Math.PI;

            mf.vehicle.modeActualHeadingError = glm.toDegrees(steerHeadingError);

            var angleNStuff = DoSteerAngleCalc();
            return angleNStuff;
        }
    }
}
