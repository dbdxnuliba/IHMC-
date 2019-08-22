package us.ihmc.humanoidBehaviors.ui.slam;

import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.messager.Messager;
import us.ihmc.ros2.Ros2Node;

public class PlanarRegionSLAMResultViewer
{
   private final Messager messager;
   public PlanarRegionSLAMResultViewer(Ros2Node ros2Node, SharedMemoryJavaFXMessager messager)
   {
      this.messager = messager;
   }
}
