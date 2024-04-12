using System;
using System.Collections.Generic;
using System.Linq;
using System.Net.Sockets;
using System.Net;
using System.Text;
using System.Threading.Tasks;
using System.Security.Cryptography.X509Certificates;
using System.Text.Json;

namespace AgOpenGPS.Classes
{
    internal class CAB_sendData_postProcess
    {

        private static CAB_sendData_postProcess instance;

        private int port;
        private double angle;
        private double step;

        private CAB_sendData_postProcess()
        {
            Port = 8585;
        }


        public static CAB_sendData_postProcess Instance
        {
            get
            {
                if (instance == null)
                {
                    instance = new CAB_sendData_postProcess();
                }
                return instance;
            }
        }

        public int Port { get; set; }
        public double Angle
        {
            get { return angle; }
            set
            {
                angle = value;
                

                //send_data(JsonSerializer.Serialize(data));
            }
        }
        public double Step
        {
            get { return step; }
            set
            {
                step = value;
                

                //send_data(JsonSerializer.Serialize(data));
            }
        }


        public void sendData()
        {
            var data = new json_angle_steer
            {
                Step = this.Step,
                Angle = this.Angle
            };
            send_data(JsonSerializer.Serialize(data));
        }

        private void send_data(string message)
        {
            try
            {
                Console.WriteLine("Data: " + JsonSerializer.Serialize(message));

                // Create a UDP socket.
                Socket udpSocket = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);

                // Enable broadcast.
                udpSocket.SetSocketOption(SocketOptionLevel.Socket, SocketOptionName.Broadcast, 1);

                // Convert the message into bytes.
                //byte[] data = Encoding.ASCII.GetBytes(message);
                byte[] data = Encoding.ASCII.GetBytes(message + "\0");
                //data.Append(Encoding.ASCII.GetBytes("\0"));
                


                // Broadcast address and port.
                IPEndPoint endPoint = new IPEndPoint(IPAddress.Broadcast, Port);

                // Send the data to the broadcast address.
                udpSocket.SendTo(data, endPoint);

                //Console.WriteLine("Message broadcasted.");

                // Close the socket.
                udpSocket.Close();
            }
            catch (Exception ex)
            {
                Console.WriteLine($"An error occurred: {ex.Message}");
            }
            
            
        }
    }

}