package us.ihmc.robotEnvironmentAwareness;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.stage.Stage;
import us.ihmc.log.LogTools;
import us.ihmc.robotEnvironmentAwareness.ui.LIDARBasedEnvironmentAwarenessUI;
import us.ihmc.robotEnvironmentAwareness.updaters.LIDARBasedREAModule;

public class LidarBasedREAStandaloneLauncher extends Application
{
   private LIDARBasedEnvironmentAwarenessUI ui;
   private LIDARBasedREAModule module;

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      Parameters parameters = getParameters();
      String uiConfigFileName = parameters.getNamed().get("uiConfigFileName");
      String moduleConfigFileName = parameters.getNamed().get("moduleConfigFileName");

      ui = LIDARBasedEnvironmentAwarenessUI.creatIntraprocessUI(uiConfigFileName, primaryStage);
      module = LIDARBasedREAModule.createIntraprocessModule(moduleConfigFileName);

      ui.show();
      module.start();
   }

   @Override
   public void stop() throws Exception
   {
      ui.stop();
      module.stop();

      Platform.exit();
   }

   public static void main(String[] args)
   {
      LogTools.info("To change the location of the config files:\n\t\"--uiConfigFileName=C:\\myFolder\\myUIConfigFileName.txt\""
            + "\n\t\"--moduleConfigFileName=C:\\myFolder\\myModuleConfigFileName.txt\"");
      launch(args);
   }
}
