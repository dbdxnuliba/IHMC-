package us.ihmc.robotics.controllers.stiction;

import org.junit.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.Random;

import static org.junit.Assert.assertEquals;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST})
public class ConstantStictionModelTest
{
   private static final int iters = 100;
   private static final double epsilon = 1e-8;

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetStictionMagnitude()
   {
      YoVariableRegistry registry = new YoVariableRegistry("test");
      YoDouble constantStictionProvider = new YoDouble("constantStictionProvider", registry);
      StictionModel stictionModel = new ConstantStictionModel(constantStictionProvider);

      assertEquals(0.0, stictionModel.getStictionMagnitude(), epsilon);

      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         double expectedMagnitude = RandomNumbers.nextDouble(random, 0.0, 1000.0);
         constantStictionProvider.set(expectedMagnitude);

         assertEquals(expectedMagnitude, stictionModel.getStictionMagnitude(), epsilon);
      }
   }
}
