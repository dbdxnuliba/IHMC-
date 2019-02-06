package us.ihmc.robotDataLogger;

import java.io.File;
import java.io.IOException;
import java.net.InetAddress;
import java.net.UnknownHostException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import gnu.trove.list.array.TByteArrayList;
import us.ihmc.idl.serializers.extra.PropertiesSerializer;
import us.ihmc.log.LogTools;

public class LoggerConfigurationLoader
{
   public static final String location = System.getProperty("user.home") + File.separator + ".ihmc" + File.separator + "IHMCLoggerConfiguration.ini";

   private final boolean publicBroadcast;
   private final TByteArrayList cameras = new TByteArrayList();
   private final ArrayList<InetAddress> initialPeers = new ArrayList<>();

   public LoggerConfigurationLoader(boolean client) throws IOException
   {
      PropertiesSerializer<LoggerConfiguration> ser = new PropertiesSerializer<>(new LoggerConfigurationPubSubType());

      boolean publicBroadcast = false;
      String cameraString = null;
      String initialPeerString = null;
      
      File in = new File(location);
      if (in.exists())
      {
         LoggerConfiguration config = ser.deserialize(in);
         publicBroadcast = config.getPublicBroadcast();
         cameraString = config.getCamerasToCaptureAsString();
         initialPeerString = config.getInitialPeersAsString();
      }

      String publicFromCmd = System.getProperty("ihmc.publicBroadcast");
      String cameraFromCmd = System.getProperty("ihmc.camerasToCapture");
      String initialPeersCmd = System.getProperty("ihmc.initialPeers"); 

      if (publicFromCmd != null)
      {
         publicBroadcast = Boolean.parseBoolean(publicFromCmd);
      }
      if (cameraFromCmd != null)
      {
         cameraString = cameraFromCmd;
      }
      if (initialPeersCmd != null)
      {
         initialPeerString = initialPeersCmd;
      }
      

      if (!client && !publicBroadcast)
      {
         LogTools.warn("Public broadcasting of logger data is OFF. The logger will only connect to your local computer. To enable public broadcasting, add \"publicBroadcast=true\" to "
               + location + " or pass in -Dihmc.publicBroadcast=true");
      }

      this.publicBroadcast = publicBroadcast;

      if (cameraString != null && !cameraString.trim().isEmpty())
      {
         String[] split = cameraString.split(",");
         for (int i = 0; i < split.length; i++)
         {
            try
            {
               byte camera = Byte.parseByte(split[i].trim());
               if (camera >= 0 && camera <= 127)
               {
                  cameras.add(camera);
               }
               else
               {
                  throw new NumberFormatException();
               }
            }
            catch (NumberFormatException e)
            {
               throw new IOException("Invalid camera id " + split[i].trim() + ",  Please edit " + location
                     + " or pass in a camera list with -Dihmc.camerasToCapture=[cameras, comma seperated]");
            }
         }
      }
      
      
      if(initialPeerString != null && !initialPeerString.trim().isEmpty())
      {
         String[] split = initialPeerString.split(",");
         for (String addressStr : split)
         {
            try
            {
               InetAddress address = InetAddress.getByName(addressStr.trim());
               initialPeers.add(address);
            }
            catch(UnknownHostException e)
            {
               throw new IOException("Invalid address " + addressStr + ", please edit " + location + " or pass in a list of initial peers with -Dihmc.initalPeers=[peers, comman seperated]");
            }
         }
      }
      
   }

   public boolean getPublicBroadcast()
   {
      return publicBroadcast;
   }

   public TByteArrayList getCameras()
   {
      return cameras;
   }
   
   public List<InetAddress> getInitialPeers()
   {
      return Collections.unmodifiableList(initialPeers);
   }

   public static void main(String[] args) throws IOException
   {
      LoggerConfigurationLoader loader = new LoggerConfigurationLoader(true);
    
      System.out.println("--- Current Settings ---");
      System.out.println("Public Broadcast: " + loader.getPublicBroadcast());
      System.out.println("Cameras: " + loader.getCameras());
      System.out.println("Initial Peers: " + loader.getInitialPeers());
    
      System.out.println();
      System.out.println("--- Default configuration file ---");
      LoggerConfiguration config = new LoggerConfiguration();
      LoggerConfigurationPubSubType type = new LoggerConfigurationPubSubType();

      PropertiesSerializer<LoggerConfiguration> ser = new PropertiesSerializer<>(type);

      System.out.println(ser.serializeToString(config));
   }
}
