package us.ihmc.humanoidBehaviors.ui.slam;

import java.io.IOException;

import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.messager.Messager;
import us.ihmc.robotEnvironmentAwareness.communication.KryoMessager;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties;
import us.ihmc.ros2.Ros2Node;

public class RealTimePlanarRegionSLAMModule
{
   private final Messager messager;
   private final Ros2Node ros2Node;

   public RealTimePlanarRegionSLAMModule(Ros2Node ros2Node, SharedMemoryJavaFXMessager messager)
   {
      this.messager = messager;
      this.ros2Node = ros2Node;
   }

   public static RealTimePlanarRegionSLAMModule createIntraprocessModule(Ros2Node ros2Node, SharedMemoryJavaFXMessager messager) throws IOException
   {
      KryoMessager kryoMessager = KryoMessager.createIntraprocess(RealTimePlanarRegionSLAMAPI.API, NetworkPorts.REA_MODULE_UI_PORT,
                                                                  REACommunicationProperties.getPrivateNetClassList());
      kryoMessager.setAllowSelfSubmit(true);
      kryoMessager.startMessager();

      return new RealTimePlanarRegionSLAMModule(ros2Node, messager);
   }

   public void stop() throws Exception
   {
      messager.closeMessager();
      ros2Node.destroy();
   }

   public void start()
   {
      
   }
}
