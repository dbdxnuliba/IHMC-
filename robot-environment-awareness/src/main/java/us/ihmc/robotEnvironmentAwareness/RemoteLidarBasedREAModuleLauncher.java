package us.ihmc.robotEnvironmentAwareness;

import com.sun.javafx.application.ParametersImpl;

import us.ihmc.log.LogTools;
import us.ihmc.robotEnvironmentAwareness.updaters.LIDARBasedREAModule;

public class RemoteLidarBasedREAModuleLauncher
{
   public static void main(String[] args) throws Exception
   {
      LogTools.info("To change the location of the config file: \"--configFileName=C:\\myFolder\\myConfigFileName.txt\"");

      ParametersImpl parameters = new ParametersImpl(args);
      String configFileName = parameters.getNamed().getOrDefault("configFileName", null);

      LIDARBasedREAModule remoteModule = LIDARBasedREAModule.createRemoteModule(configFileName);
      remoteModule.start();
   }
}
