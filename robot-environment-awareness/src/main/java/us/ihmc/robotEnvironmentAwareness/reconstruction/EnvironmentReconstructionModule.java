package us.ihmc.robotEnvironmentAwareness.reconstruction;

import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.log.LogTools;
import us.ihmc.ros2.Ros2Node;

public class EnvironmentReconstructionModule
{

   public EnvironmentReconstructionModule(Ros2Node ros2Node, SharedMemoryJavaFXMessager messager)
   {
      
   }
   
   public void stop() throws Exception
   {
//      reaMessager.closeMessager();
//      ros2Node.destroy();
//
//      if (scheduled != null)
//      {
//         scheduled.cancel(true);
//         scheduled = null;
//      }
//
//      if (executorService != null)
//      {
//         executorService.shutdownNow();
//         executorService = null;
//      }
   }
   
   public static EnvironmentReconstructionModule createIntraprocessModule(Ros2Node ros2Node, SharedMemoryJavaFXMessager messager)
   {
      return new EnvironmentReconstructionModule(ros2Node, messager);
   }
}
