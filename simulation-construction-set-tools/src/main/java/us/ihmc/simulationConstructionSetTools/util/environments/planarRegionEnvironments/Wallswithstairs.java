package us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments;

import us.ihmc.euclid.*;
import us.ihmc.graphicsDescription.appearance.YoAppearance;

public class Wallswithstairs extends PlanarRegionEnvironmentInterface
{

   public Wallswithstairs(double stepLength, double wallHeight, double stepUpHeight)
   {

      double wallOffSet = 0.8; //0.11 was comfotable to wall so choosing 0.08 as starting position
      generator.identity();
      generator.translate(0.0,wallOffSet,0.0);
      generator.addCubeReferencedAtBottomMiddle(stepLength, 0.1, wallHeight);

      generator.identity();
      generator.translate(0.0,0.0,wallHeight);
      generator.addCubeReferencedAtBottomMiddle(stepLength,2*wallOffSet,0.2);


      generator.identity();
      generator.translate(0.0, -wallOffSet,0.0);
      generator.addCubeReferencedAtBottomMiddle(stepLength, 0.1, wallHeight);

      addPlanarRegionsToTerrain(YoAppearance.Grey());

      //SingleStepEnvironment environment = new SingleStepEnvironment(stepHeight, 0.7);

      // first ground plane
      generator.identity();
      generator.addRectangle(2.0, 2.0);

      // step
      generator.translate(1.0 + 0.5 * stepLength, 0.0, stepUpHeight);
      generator.addRectangle(stepLength, 2.0);

      generator.identity();
      generator.translate(1.0, 0.0, 0.5 * stepUpHeight);
      generator.rotate(0.5 * Math.PI, Axis.Y);
      generator.addRectangle(stepUpHeight, 2.0);

      generator.identity();
      generator.translate(1.0 + stepLength, 0.0, 0.5 * stepUpHeight);
      generator.rotate(0.5 * Math.PI, Axis.Y);
      generator.addRectangle(stepUpHeight, 2.0);

      // second ground plane
      generator.identity();
      generator.translate(2.0 + stepLength, 0.0, 0.0);
      generator.addRectangle(2.0, 2.0);

      addPlanarRegionsToTerrain(YoAppearance.Grey());



   }

}
