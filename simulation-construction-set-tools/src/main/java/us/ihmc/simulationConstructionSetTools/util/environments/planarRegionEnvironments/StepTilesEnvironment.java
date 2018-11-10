package us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.appearance.YoAppearanceMaterial;
import us.ihmc.graphicsDescription.appearance.YoAppearanceTexture;
import us.ihmc.simulationConstructionSetTools.util.environments.DefaultCommonAvatarEnvironment;
import us.ihmc.simulationConstructionSetTools.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

public class StepTilesEnvironment extends PlanarRegionEnvironmentInterface
{
   private final CombinedTerrainObject3D water;

   public StepTilesEnvironment(double footWidth, double footLenght, double stepWidth, double stepLength, int numberOfSteps)
   {
      water = DefaultCommonAvatarEnvironment.setUpWater("Water");
      YoAppearanceTexture texture = new YoAppearanceTexture("Textures/brick.png");


      water.addBox(-0.5*footLenght,0.5*footWidth-0.15,0.5*footLenght,-0.5*footWidth-0.15,0.001, texture);

      water.addBox(-0.5*footLenght,0.5*footWidth+0.15,0.5*footLenght,-0.5*footWidth+0.15,0.001, texture);
      for (int i = 0; i < numberOfSteps; i++)
      {
            double tilePositionX = i*stepLength+stepLength;
            double tilePositionY = -stepWidth;
            stepWidth=-stepWidth;
         water.addBox(-0.5*footLenght+tilePositionX, 0.5*footWidth+tilePositionY, 0.5*footLenght+tilePositionX, -0.5*footWidth+tilePositionY, 0.001, texture);

      }
      //addPlanarRegionsToTerrain(YoAppearance.Grey());
   }
   @Override
   public TerrainObject3D getTerrainObject3D()
   {
      return water;
   }
}

