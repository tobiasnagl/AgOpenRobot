using AgOpenGPS.Classes;
using System;
using System.Collections.Generic;
using System.IO;
using System.Net.Sockets;
using System.Text;
using System.Threading.Tasks;

namespace AgOpenGPS
{
    public class CSim
    {
        private readonly FormGPS mf;

        #region properties sim
        public double altitude = 300;

        public double latitude, longitude;
        public double latitude1, longitude1;


        public double headingTrue, stepDistance = 0.0, steerAngle, steerangleAve = 0.0;
        public double steerAngleScrollBar = 0;

        public bool isAccelForward, isAccelBack;

        private bool isTCPset =  false;
        private string filepath = @"C:\Users\gabriel.steinwander\Desktop\data.csv";

        public List<ROBOT_vector> coordinates = new List<ROBOT_vector>() {  };
        private int lastLine = 0;
        #endregion properties sim

        public async Task<string> RecieveData()
        {
            if (isTCPset)
            {
                string server = "172.16.221.250";
                int port = 6;

                try
                {
                    // Erstelle einen neuen TCP-Client
                    using (var client = new TcpClient(server, port))
                    {
                        // Erhalte den Stream zum Lesen und Schreiben von Daten
                        NetworkStream stream = client.GetStream();

                        byte[] buffer = new byte[256]; // Puffer für empfangene Daten
                        int numberOfBytesRead;

                        Console.WriteLine("Verbunden mit dem Server. Warte auf Daten...");

                        // Schleife, die Daten empfängt, solange die Verbindung offen ist

                        // Initialisiere den Puffer
                        StringBuilder responseData = new StringBuilder();

                        // Lies Daten vom Server
                        if ((numberOfBytesRead = await stream.ReadAsync(buffer, 0, buffer.Length)) != 0)
                        {
                            // Konvertiere die Daten von Bytes in einen String
                            responseData.Append(Encoding.ASCII.GetString(buffer, 0, numberOfBytesRead));
                            Console.WriteLine("Empfangen: {0}", responseData.ToString());
                            var splitData = responseData.ToString().Split('\n');
                            foreach (var item in splitData)
                            {
                                var moretemp = ParseReceiveData(item);
                                if (moretemp.Item1 > 0 && moretemp.Item2 > 0)
                                {
                                    this.latitude1 = moretemp.Item1;
                                    this.longitude1 = moretemp.Item2;
                                    var coord = new ROBOT_vector { X = moretemp.Item1, Y = moretemp.Item2 };
                                    coordinates.Add(coord);
                                    //coordinates.Add(new ROBOT_vector() { Y = 48.12, X = 15.12 });
                                    //coordinates.Add(new ROBOT_vector() { Y = 48.121, X = 15.121 });
                                    //coordinates.Add(new ROBOT_vector() { Y = 48.122, X = 15.122 });

                                    if (coordinates.Count >= 10)
                                    {
                                        coordinates.RemoveAt(0);
                                    }
                                    Console.WriteLine(moretemp);
                                }
                            }
                        }
                        if (!(coordinates.Count <= 2))
                        {
                            calculateAlpha();
                        }
                        // Schließe den Stream und den Client
                        stream.Close();
                    }

                }
                catch (ArgumentNullException e)
                {
                    Console.WriteLine("ArgumentNullException: {0}", e);
                }
                catch (SocketException e)
                {
                    Console.WriteLine("SocketException: {0}", e);
                }
                return "";
            }
            else
            {
                var lines = File.ReadAllLines(filepath);
                double lat;
                double lon;
                string line = lines[lastLine];
                
                var coord = line.Split(';');
                lon = double.Parse(coord[0]);
                lat = double.Parse(coord[1]);

                await Console.Out.WriteLineAsync($"{lat}     {lon}");
                   

                if (lat > 0 && lon > 0)
                {
                    this.latitude1 = lat;
                    this.longitude1 = lon;
                    var coordinate = new ROBOT_vector { X = lat, Y = lon };
                    coordinates.Add(coordinate);
                    if (coordinates.Count >= 10)
                    {
                        coordinates.RemoveAt(0);
                    }

                    if (!(coordinates.Count <= 2))
                    {
                        calculateAlpha();
                    }
                    
                }
                lastLine++;
                return "";

            }
        }
 

        

        public void calculateAlpha()
        {
            var currentPos = coordinates[coordinates.Count-1];
            var lastPos = coordinates[coordinates.Count - 2];
            Console.WriteLine($"Current Pos: {currentPos.X} {currentPos.Y}\n\nLast Pos: {lastPos.X} {currentPos.Y}");

            Console.WriteLine("Alpha: " + alphaCalculation.calculateAlpha(lastPos, currentPos) * 180/Math.PI);
        }

        public CSim(FormGPS _f)
        {
            

            mf = _f;
            latitude = Properties.Settings.Default.setGPS_SimLatitude;
            longitude = Properties.Settings.Default.setGPS_SimLongitude;


        }

        public async void DoSimTick(double _st)
        {
            steerAngle = _st;

            double diff = Math.Abs(steerAngle - steerangleAve);

            if ( diff > 11)
            {
                if (steerangleAve >= steerAngle)
                {
                    steerangleAve -= 6;
                }
                else steerangleAve += 6;
            }
            else if (diff > 5)
            {
                if (steerangleAve >= steerAngle)
                {
                    steerangleAve -= 2;
                }
                else steerangleAve += 2;
            }
            else if (diff > 1)
            {
                if (steerangleAve >= steerAngle)
                {
                    steerangleAve -= 0.5;
                }
                else steerangleAve += 0.5;
            } 
            else
            {
                steerangleAve = steerAngle;
            }

            mf.mc.actualSteerAngleDegrees = steerangleAve;

            double temp = stepDistance * Math.Tan(steerangleAve * 0.0165329252) / 2;
            headingTrue += temp;
            if (headingTrue > glm.twoPI) headingTrue -= glm.twoPI;
            if (headingTrue < 0) headingTrue += glm.twoPI;

            mf.pn.vtgSpeed = Math.Abs(Math.Round(4 * stepDistance * 10, 2));
            mf.pn.AverageTheSpeed();

            //Calculate the next Lat Long based on heading and distance
            CalculateNewPostionFromBearingDistance(glm.toRadians(latitude), glm.toRadians(longitude), headingTrue, stepDistance / 1000.0);

            mf.pn.ConvertWGS84ToLocal(latitude, longitude, out mf.pn.fix.northing, out mf.pn.fix.easting);

            mf.pn.headingTrue = mf.pn.headingTrueDual = glm.toDegrees(headingTrue);
            mf.ahrs.imuHeading = mf.pn.headingTrue;
            if (mf.ahrs.imuHeading > 360) mf.ahrs.imuHeading -= 360;

            Task<string> result = RecieveData();

            result.Wait(100);
            if(result.IsCompleted)
            {
                //aktualisieren
            }
            // richtig zuweisen
            mf.pn.ConvertWGS84ToLocal(latitude1, longitude1, out mf.pn.fix.northing, out mf.pn.fix.easting);
            mf.pn.latitude = latitude1;
            mf.pn.longitude = longitude1;

            mf.pn.hdop = 0.7;
            mf.pn.altitude = 732;
            mf.pn.satellitesTracked = 12;

            mf.sentenceCounter = 0;
            mf.UpdateFixPosition();

            if (isAccelForward)
            {
                isAccelBack = false;
                stepDistance += 0.02;
                if (stepDistance > 0.12) isAccelForward = false;
            }

            if (isAccelBack)
            {
                isAccelForward = false;
                stepDistance -= 0.01;
                if (stepDistance < -0.06) isAccelBack = false;
            }
        }

        public void CalculateNewPostionFromBearingDistance(double lat, double lng, double bearing, double distance)
        {
            double R = distance / 6371.0; // Earth Radius in Km

            double lat2 = Math.Asin((Math.Sin(lat) * Math.Cos(R)) + (Math.Cos(lat) * Math.Sin(R) * Math.Cos(bearing)));
            double lon2 = lng + Math.Atan2(Math.Sin(bearing) * Math.Sin(R) * Math.Cos(lat), Math.Cos(R) - (Math.Sin(lat) * Math.Sin(lat2)));

            latitude = glm.toDegrees(lat2);
            longitude = glm.toDegrees(lon2);
        }

        public static (double, double) ParseReceiveData(string nmeaData)
        {
            string[] nmeaDataArray = nmeaData.Split(',');
            double latitude = 0;
            double longitude = 0;
            string latitude_comma = "";
            string longitude_comma = "";

            if (nmeaDataArray[0] == "$GNGGA" &&
                nmeaDataArray.Length >= 7 &&
                !string.IsNullOrWhiteSpace(nmeaDataArray[2]) &&
                !string.IsNullOrWhiteSpace(nmeaDataArray[4]))
            {
                // Parse latitude and longitude using NmeaToDecimal function
                latitude = NmeaToDegrees_lat(nmeaDataArray[2], (nmeaDataArray[3] == "S") ? -1 : 1);
                longitude = NmeaToDegrees_lon(nmeaDataArray[4], (nmeaDataArray[5] == "W") ? -1 : 1);
                latitude_comma = NmeaToMinutes_lat(nmeaDataArray[2]).ToString().Replace(",", "").Replace(".","").Remove(0,1);
                longitude_comma = NmeaToMinutes_lon(nmeaDataArray[4]).ToString().Replace(",", "").Replace(".","").Remove(0,1);

                //double latitude1 = latitude + latitude_comma;
                //double longitude1 = longitude + longitude_comma;
                latitude = double.Parse(latitude + "." + latitude_comma);
                longitude = double.Parse(longitude + "." + longitude_comma);


                return (latitude, longitude);
            }
            else
            {
                //return "Invalid input data";
                return (0,0);
            }

        }

        public static int NmeaToDegrees_lat(string ll, int hemisph)
        {
            int degrees = int.Parse(ll.Substring(0, 2))* hemisph;

            return degrees;
        }
        public static int NmeaToDegrees_lon(string ll, int hemisph)
        {

            int degrees = int.Parse(ll.Substring(0,3)) * hemisph;

            return degrees;
        }

        public static double NmeaToMinutes_lat(string ll)
        {
            double minutes = double.Parse(ll.Substring(2)) / 60;
            return minutes;
        }
        public static double NmeaToMinutes_lon(string ll)
        {
            double minutes = double.Parse(ll.Substring(3)) / 60;
            return minutes;
        }

    }
}