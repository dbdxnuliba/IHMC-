package us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments;

import us.ihmc.euclid.*;
import us.ihmc.euclid.tuple3D.*;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.simulationConstructionSetTools.robotController.*;
import us.ihmc.simulationConstructionSetTools.util.environments.*;
import us.ihmc.simulationConstructionSetTools.util.environments.environmentRobots.*;
import us.ihmc.simulationConstructionSetTools.util.ground.*;
import us.ihmc.simulationconstructionset.*;
import us.ihmc.simulationconstructionset.util.ground.*;
import us.ihmc.simulationconstructionset.util.simulationRunner.*;
import us.ihmc.yoVariables.registry.*;

import java.util.*;

public class Wallswithstairs extends PlanarRegionEnvironmentInterface
{
   private final List<Robot> contactableRobots = new ArrayList<>();
   private final CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D(getClass().getSimpleName());
   private final ArrayList<ExternalForcePoint> contactPoints = new ArrayList<>();
   //private YoVariableRegistry doorTestRegistry;

   public Wallswithstairs(double stepLength, double wallHeight, double stepUpHeight)
   {
      double wallInitialOffSet = 1.0 + 2.5* stepLength; // this means that the wall is ahead of the steps by one stepLength to allow you just suffcient space to use footstep planner tool
      double wallOffSet = 0.8; //0.11 was comfortable to wall so choosing 0.08 as starting position
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
/*
      //combinedTerrainObject.addBox(-5.0,-30.0,5.0,-0.05,0.0,YoAppearance.DarkGray());
      //combinedTerrainObject.addBox(-1.2192, -0.025, 0, 0.025, wallHeight,YoAppearance.Red());
      //combinedTerrainObject.addBox(0.0 +ContactableDoorRobot.DEFAULT_DOOR_DIMENSIONS.getX(), -0.025, 1.2192 + ContactableDoorRobot.DEFAULT_DOOR_DIMENSIONS.getY(), 0.025, wallHeight, YoAppearance.Blue());


      //combinedTerrainObject3D.getLinkGraphics().identity();
      //combinedTerrainObject3D.getLinkGraphics().addModelFile("models/SCTestBed.obj");

      ContactableDoorRobot door = new ContactableDoorRobot("doorRobot", new Point3D());
      contactableRobots.add(door);
      door.createAvailableContactPoints(0,15,15,0.02,true);
*/

   }
/*
   @Override
   public TerrainObject3D getTerrainObject3D()
   {
      return combinedTerrainObject;
   }

   @Override
   public List<? extends Robot> getEnvironmentRobots()
   {
      return contactableRobots;
   }

   @Override
   public void createAndSetContactControllerToARobot()
   {
      ContactController contactController = new ContactController();
      contactController.setContactParameters(100000.0, 100.0, 0.5, 0.3);

      contactController.addContactPoints(contactPoints);

      for (Robot r : contactableRobots)
      {
         if (r instanceof Contactable)
            contactController.addContactable((Contactable) r);

      }
      if (contactableRobots.size() > 0)
         contactableRobots.get(0).setController(contactController);
   }

   @Override
   public void addContactPoints(List<? extends ExternalForcePoint> externalForcePoints)
   {
      this.contactPoints.addAll(externalForcePoints);
   }

   @Override
   public void addSelectableListenerToSelectables(SelectableObjectListener selectedListener)
   {
   }
*/

}