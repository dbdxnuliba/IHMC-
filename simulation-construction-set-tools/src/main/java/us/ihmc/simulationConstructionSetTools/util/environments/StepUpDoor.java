package us.ihmc.simulationConstructionSetTools.util.environments;

import us.ihmc.euclid.geometry.*;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.shape.*;
import us.ihmc.euclid.transform.*;
import us.ihmc.euclid.tuple3D.*;
import us.ihmc.graphicsDescription.appearance.*;
import us.ihmc.simulationConstructionSetTools.robotController.*;
import us.ihmc.simulationConstructionSetTools.util.environments.environmentRobots.*;
import us.ihmc.simulationConstructionSetTools.util.ground.*;
import us.ihmc.simulationconstructionset.*;
import us.ihmc.simulationconstructionset.util.ground.*;

import java.util.*;

public class StepUpDoor implements CommonAvatarEnvironmentInterface
{
   private final boolean ADD_FIDCUIAL_FLOATING_BOX = false;
   private final boolean ADD_DOOR = true;
   private FramePose3D doorframepose = new FramePose3D();

   private final List<Robot> contactableRobots = new ArrayList<>();
   private final CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D(getClass().getSimpleName());
   private final ArrayList<ExternalForcePoint> contactPoints = new ArrayList<ExternalForcePoint>();

   public StepUpDoor()
   {

   }

   public StepUpDoor(double stepLength, double wallHeight, double stepUpHeight)
   {
      double wallInitialOffSet = 1.0 + 2.5* stepLength; // this means that the wall is ahead of the steps by one stepLength to allow you just suffcient space to use footstep planner tool
      double wallOffSet = 0.8;

      double xstart = 0.5;
      double stairStart = 0.5 + xstart;
      double wallWidth = 0.2;
      double doorAngle = -1.5708; //90degrees in rads
      //DefaultCommonAvatarEnvironment environment = new DefaultCommonAvatarEnvironment();
      //combinedTerrainObject.addTerrainObject(environment.setUpGround("ground"));

      combinedTerrainObject.addTerrainObject(setUpGround("Ground"));
      //combinedTerrainObject.addTerrainObject(environment.getTerrainObject3D());
      combinedTerrainObject.addBox(stairStart , -1.0, stairStart+stepLength, 1.0, stepUpHeight, YoAppearance.Gray()); //the robot is the starting location

      //building walls
      AppearanceDefinition appearance = YoAppearance.Gray();
      appearance.setTransparency(0.25);
      combinedTerrainObject.addBox(wallInitialOffSet, wallOffSet, wallInitialOffSet+ stepLength, wallOffSet+wallWidth ,wallHeight, appearance); //left wall
      combinedTerrainObject.addBox(wallInitialOffSet, -wallOffSet, wallInitialOffSet+stepLength, -(wallOffSet+wallWidth),wallHeight, appearance); //right wall
      //combinedTerrainObject.addBox(wallInitialOffSet,wallOffSet,wallInitialOffSet+stepLength,-(wallOffSet+wallHeight), 0.3, appearance); //top wall
//      combinedTerrainObject.addBox(wallInitialOffSet,wallOffSet,wallInitialOffSet+stepLength,-(wallOffSet+wallWidth), wallHeight,wallHeight+0.3,appearance);

      Point3D doorPosition = new Point3D(wallInitialOffSet + stepLength + 1.5, 0.5, 0.0);
      Point3D fiducialPosition = new Point3D(wallInitialOffSet + stepLength + 1.25, 0.0, 1.25);

      doorframepose = new FramePose3D(new Pose3D(doorPosition.getX(),doorPosition.getY(),doorPosition.getZ(),doorAngle,0.0,0.0));
      //System.out.println(doorframepose.getPosition().getX());
      if(ADD_DOOR)
      {

         ContactableDoorRobot door = new ContactableDoorRobot("doorRobot", doorPosition);
         contactableRobots.add(door);
         door.createAvailableContactPoints(0, 15, 15, 0.02, true);

         door.setYaw(-doorAngle);
      }

      if(ADD_FIDCUIAL_FLOATING_BOX)
      {
         FloatingFiducialBoxRobot fiducialBoxRobot = new FloatingFiducialBoxRobot(Fiducial.FIDUCIAL450,"4");
         fiducialBoxRobot.setPosition(fiducialPosition);
         fiducialBoxRobot.setYawPitchRoll(0.0, Math.PI/2.0, 0.0);

         contactableRobots.add(fiducialBoxRobot);
      }
   }

   public CombinedTerrainObject3D setUpGround(String name)
   {
      CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D(name);

      //    URL fileURL = DRCDemo01NavigationEnvironment.class.getClassLoader().getResource("Textures/ground2.png");
      YoAppearanceTexture texture = new YoAppearanceTexture("Textures/ground2.png");

      RigidBodyTransform location = new RigidBodyTransform();
      location.setTranslation(new Vector3D(0, 0, -0.5));

      RotatableBoxTerrainObject newBox = new RotatableBoxTerrainObject(new Box3D(location, 45, 45, 1), texture);
      combinedTerrainObject.addTerrainObject(newBox);
      RotatableBoxTerrainObject newBox2 = new RotatableBoxTerrainObject(new Box3D(location, 200, 200, 0.75), YoAppearance.DarkGray());
      combinedTerrainObject.addTerrainObject(newBox2);

      return combinedTerrainObject;
   }

   public FramePose3D getDoorFramePose()
   {
      return doorframepose;
   }

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


}
