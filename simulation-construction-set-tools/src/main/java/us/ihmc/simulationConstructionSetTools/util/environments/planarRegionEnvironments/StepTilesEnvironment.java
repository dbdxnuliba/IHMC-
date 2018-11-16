package us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments;

import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.appearance.YoAppearanceTexture;
import us.ihmc.simulationConstructionSetTools.util.environments.DefaultCommonAvatarEnvironment;
import us.ihmc.simulationConstructionSetTools.util.ground.CombinedTerrainObject3D;

public class StepTilesEnvironment extends PlanarRegionEnvironmentInterface
{

   public StepTilesEnvironment(double footWidth, double footLenght, double stepWidth, double stepLength, int numberOfSteps)
   {
      generator.translate(0.0, -0.15, 0.001);
      generator.addRectangle(footLenght,footWidth);
      generator.identity();
      generator.translate(0.0, 0.15, 0.001);
      generator.addRectangle(footLenght,footWidth);
      generator.identity();
      for (int i = 0; i < numberOfSteps; i++)
      {
         double tilePositionX = i * stepLength + stepLength;
         double tilePositionY = -stepWidth;
         stepWidth = -stepWidth;
         generator.translate(tilePositionX, tilePositionY, 0.001);
         generator.addRectangle(footLenght,footWidth);
         generator.identity();
      }
      addPlanarRegionsToTerrain(YoAppearance.Grey());
   }
}



