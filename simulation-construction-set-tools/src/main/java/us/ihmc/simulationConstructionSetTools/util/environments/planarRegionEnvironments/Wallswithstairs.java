package us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments;

import boofcv.struct.flow.ImageFlow.*;
import us.ihmc.euclid.*;
import us.ihmc.euclid.transform.*;
import us.ihmc.euclid.tuple3D.*;
import us.ihmc.graphicsDescription.appearance.*;
import us.ihmc.simulationConstructionSetTools.robotController.*;
import us.ihmc.simulationConstructionSetTools.util.environments.*;
import us.ihmc.simulationConstructionSetTools.util.environments.environmentRobots.*;
import us.ihmc.simulationConstructionSetTools.util.ground.*;
import us.ihmc.simulationconstructionset.*;
//import us.ihmc.avatar.environments.*;
//import us.ihmc.simulationconstructionset.util.ground.*;
import us.ihmc.simulationconstructionset.util.ground.*;
import us.ihmc.simulationconstructionset.util.simulationRunner.*;
import us.ihmc.yoVariables.registry.*;

import java.util.*;

public class Wallswithstairs extends PlanarRegionEnvironmentInterface implements CommonAvatarEnvironmentInterface
{
   private final boolean Door_test = false;
   private final List<Robot> contactableRobots = new ArrayList<>();
   private final CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D(getClass().getSimpleName());
   private final ArrayList<ExternalForcePoint> contactPoints = new ArrayList<ExternalForcePoint>();
   //private YoVariableRegistry doorTestRegistry;

   public Wallswithstairs(double stepLength, double wallHeight, double stepUpHeight)
   {
      double wallInitialOffSet = 1.0 + 2.5* stepLength; // this means that the wall is ahead of the steps by one stepLength to allow you just suffcient space to use footstep planner tool
      double wallOffSet = 0.8; //0.11 was comfortable to wall so choosing 0.8 as starting position
      generator.identity();
      generator.translate(wallInitialOffSet,wallOffSet,0.0);  // left wall
      generator.addCubeReferencedAtBottomMiddle(stepLength, 0.1, wallHeight);

      generator.identity();
      generator.translate(wallInitialOffSet,0.0,wallHeight); // top roof
      generator.addCubeReferencedAtBottomMiddle(stepLength,2*wallOffSet +0.1,0.1);


      generator.identity();
      generator.translate(wallInitialOffSet, -wallOffSet,0.0); //right wall
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