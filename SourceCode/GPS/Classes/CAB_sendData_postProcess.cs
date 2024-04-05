using System;
using System.Collections.Generic;
using System.Linq;
using System.Net.Sockets;
using System.Net;
using System.Text;
using System.Threading.Tasks;
using System.Security.Cryptography.X509Certificates;

namespace AgOpenGPS.Classes
{
    internal class CAB_sendData_postProcess
    {
        private int port;

        public CAB_sendData_postProcess()
        {
            Port = 8585;
            ;
        }

        public int Port { get; set; }

        public void encodeData(double angle)
        {
            send_data(angle.ToString());
        }

        private void send_data(string message)
        {
            try
            {
                // Create a UDP socket.
                Socket udpSocket = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);

                // Enable broadcast.
                udpSocket.SetSocketOption(SocketOptionLevel.Socket, SocketOptionName.Broadcast, 1);

                // Convert the message into bytes.
                byte[] data = Encoding.ASCII.GetBytes(message);

                // Broadcast address and port.
                IPEndPoint endPoint = new IPEndPoint(IPAddress.Broadcast, Port);

                // Send the data to the broadcast address.
                udpSocket.SendTo(data, endPoint);

                Console.WriteLine("Message broadcasted.");

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