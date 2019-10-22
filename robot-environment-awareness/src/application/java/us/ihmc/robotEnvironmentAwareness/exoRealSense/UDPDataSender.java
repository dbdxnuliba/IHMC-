package us.ihmc.robotEnvironmentAwareness.exoRealSense;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;

public class UDPDataSender
{
   //params
   final String IPAddressString = "192.168.0.11";   
   final int port = 6669;    
   
   //variables
   DatagramSocket socket; 
   InetAddress IPAddress;  
   
   public UDPDataSender() {
      try {
         socket = new DatagramSocket(port+1);   
         IPAddress = InetAddress.getByName(IPAddressString);
      }
      catch (Exception e) {
         e.printStackTrace();
      }
   }
   
   public void sendDistance(double distance) {
      try
      {
         String dataString = String.valueOf(distance); 
         byte[] sendData = dataString.getBytes();
         DatagramPacket sendPacket = new DatagramPacket(sendData, sendData.length, IPAddress, port);
         socket.send(sendPacket);   
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
      
      //System.out.println("sent: " + counter);
   }
}
