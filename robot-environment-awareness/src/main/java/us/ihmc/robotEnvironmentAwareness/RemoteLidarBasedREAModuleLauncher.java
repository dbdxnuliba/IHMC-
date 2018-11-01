package us.ihmc.robotEnvironmentAwareness;

import us.ihmc.robotEnvironmentAwareness.updaters.LIDARBasedREAModule;

public class RemoteLidarBasedREAModuleLauncher
{
   public static void main(String[] args) throws Exception
   {
      LIDARBasedREAModule remoteModule = LIDARBasedREAModule.createRemoteModule(null);
      remoteModule.start();
   }
}
