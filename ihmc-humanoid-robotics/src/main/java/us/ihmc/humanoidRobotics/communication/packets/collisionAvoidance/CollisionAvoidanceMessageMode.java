package us.ihmc.humanoidRobotics.communication.packets.collisionAvoidance;

import us.ihmc.communication.ros.generators.RosEnumValueDocumentation;

public enum CollisionAvoidanceMessageMode
{

   @RosEnumValueDocumentation(documentation = "This message will override the previous regions.")
   OVERRIDE,
   @RosEnumValueDocumentation(documentation = "The previous message will add the new regions to the old ones.")
   ADD;

   public static final CollisionAvoidanceMessageMode[] values = values();

   public byte toByte()
   {
      return (byte) ordinal();
   }

   public static CollisionAvoidanceMessageMode fromByte(byte enumAsByte)
   {
      return values[enumAsByte];
   }
}
