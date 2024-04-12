using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AgOpenGPS.Classes
{
    public static class alphaCalculation
    {
        /// <summary>
        /// Calculates Alpha based on the current position and the last position
        /// A = last position
        /// B = current position
        /// </summary>
        /// <param name="A"></param>
        /// <param name="B"></param>
        /// <returns></returns>
        public static double calculateAlpha(ROBOT_vector A, ROBOT_vector B)
        {
            var C = calculateC(A, B);
            var a = vectorTwoPoints(C, A);
            var b = vectorTwoPoints(B, A);

            double amountA = amountVector(a);
            double amountB = amountVector(b);

            return alpha(amountA, amountB);
        }


        private static ROBOT_vector calculateC(ROBOT_vector A, ROBOT_vector B)
        {
            ROBOT_vector C = new ROBOT_vector();
            C.X = A.X;
            C.Y = B.Y;
            return C;
        }

        private static ROBOT_vector vectorTwoPoints(ROBOT_vector spitze, ROBOT_vector schaft)
        {
            ROBOT_vector c = new ROBOT_vector();
            c.X = spitze.X - schaft.X;
            c.Y = spitze.Y - schaft.Y;
            return c;
        }

        private static double amountVector(ROBOT_vector vector)
        {
            return Math.Sqrt(Math.Pow(vector.X, 2) + Math.Pow(vector.Y, 2));
        }

        private static double alpha(double a, double b)
        {
            var alpha = Math.Atan(a / b);
            return alpha;
        }
    }
}
