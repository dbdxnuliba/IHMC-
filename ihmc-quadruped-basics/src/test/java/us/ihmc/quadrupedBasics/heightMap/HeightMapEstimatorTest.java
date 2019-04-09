package us.ihmc.quadrupedBasics.heightMap;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;

public class HeightMapEstimatorTest
{
   // Whether to start the UI or not.
   private static boolean VISUALIZE = true;

   private static HeightMapEstimatorVisualizerApplication visualizerApplication = null;
   private static JavaFXMessager messager = null;



   @BeforeEach
   public void setup()
   {
      VISUALIZE = VISUALIZE && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();

      if (VISUALIZE)
      {
         visualizerApplication = new HeightMapEstimatorVisualizerApplication();
         visualizerApplication.startOnAThread();

         messager = visualizerApplication.getMessager();
      }
   }

   @AfterEach
   public void destroy() throws Exception
   {
      if (VISUALIZE)
      {
         visualizerApplication.stop();
         visualizerApplication = null;
         messager = null;
      }
   }

   @Test
   public void testThreePoints()
   {
      messager.submitMessage(HeightMapEstimatorMessagerAPI.PointToAddTopic, new Point3D(0.5, 0.5, -0.1));
      messager.submitMessage(HeightMapEstimatorMessagerAPI.PointToAddTopic, new Point3D(-0.5, 0.5, 0.1));
      messager.submitMessage(HeightMapEstimatorMessagerAPI.PointToAddTopic, new Point3D(-0.5, -0.5, 0.1));
   }
}
