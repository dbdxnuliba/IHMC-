package us.ihmc.footstepPlanning.flatGroundPlanning;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments.PlanarRegionEnvironmentInterface;

public class AnimationEnvironment extends PlanarRegionEnvironmentInterface
{
   public AnimationEnvironment()
   {
      generator.identity();
      generator.translate(1.0, 0.0, 0.0);
      generator.addRectangle(1.75, 1.0);

      generator.identity();
      generator.addRectangle(0.6, 1.0);

      generator.identity();
      generator.translate(0.0, -0.6, 0.0);
      generator.addCubeReferencedAtBottomMiddle(0.3, 0.2, 0.4);

      generator.identity();
      generator.translate(-0.05, 0.1, 0.0);
      generator.addCubeReferencedAtBottomMiddle(0.3, 0.5, 0.05);

      generator.identity();
      generator.translate(0.2, -0.2, 0.0);
      generator.rotateEuler(new Vector3D(0.0, -Math.toRadians(35), 0.0));
      generator.addCubeReferencedAtBottomMiddle(0.3, 0.5, 0.05);

      addPlanarRegionsToTerrain(YoAppearance.Grey());
   }
}
