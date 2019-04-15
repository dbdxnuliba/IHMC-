package us.ihmc.robotEnvironmentAwareness.fusion;

import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;

public class LidarImageFusionAPI
{
   private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();
   
   public static final MessagerAPI API = apiFactory.getAPIAndCloseFactory();
}
