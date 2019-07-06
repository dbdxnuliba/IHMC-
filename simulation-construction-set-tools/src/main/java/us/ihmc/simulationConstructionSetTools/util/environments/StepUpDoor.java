package us.ihmc.simulationConstructionSetTools.util.environments;

import com.github.quickhull3d.*;
import javafx.geometry.*;
import us.ihmc.euclid.axisAngle.*;
import us.ihmc.euclid.geometry.*;
import us.ihmc.euclid.referenceFrame.*;
//import us.ihmc.euclid.shape.*;
import us.ihmc.euclid.shape.primitives.*;
import us.ihmc.euclid.transform.*;
import us.ihmc.euclid.tuple3D.*;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.appearance.*;
import us.ihmc.robotics.geometry.*;
import us.ihmc.simulationConstructionSetTools.robotController.*;
import us.ihmc.simulationConstructionSetTools.util.environments.environmentRobots.*;
import us.ihmc.simulationConstructionSetTools.util.ground.*;
import us.ihmc.simulationconstructionset.*;
import us.ihmc.simulationconstructionset.util.*;
import us.ihmc.simulationconstructionset.util.ground.*;
import us.ihmc.yoVariables.registry.*;

import java.util.*;

public class StepUpDoor extends DefaultCommonAvatarEnvironment implements CommonAvatarEnvironmentInterface
{
   private final boolean ADD_FIDCUIAL_FLOATING_BOX = true;
   private final boolean ADD_DOOR = false;

   private final boolean ADD_BOXES = false;
   private final boolean ADD_CYLINDER = false;
   private final boolean ADD_SPHERES = true;

   private FramePose3D doorframepose = new FramePose3D();
   private final int NUmberofBoxes = 1;
   private final int NumberofSpheres = 1;
   private final int NumberofCylinders = 1;

   private final List<Robot> contactableRobots = new ArrayList<>();  //list of robots
   private final CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D(getClass().getSimpleName());
   private final ArrayList<ExternalForcePoint> contactPoints = new ArrayList<ExternalForcePoint>(); //list of external contact points

//   double kXY = 1000.0;
//   double bXY = 100.0;
//   double kZ = 500.0;
//   double bZ = 50.0;
//   double alphaStick = 0.7;
//   double alphaSlip = 0.5;




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

      double forceVectorScale = 1.0 / 50.0;

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

      if(ADD_CYLINDER)
      {
         double cylinderOffset = 0.5;
         Vector3D cylinderPosition = new Vector3D(doorPosition.getX(),0.0,0.0); //initial cylinder position
         for (int i = 0; i < NumberofCylinders; i++)
         {

            RigidBodyTransform transform = new RigidBodyTransform();

            AxisAngle facingUpwards = new AxisAngle();
            transform.setTranslation(cylinderPosition);
            transform.setRotation(facingUpwards);

            double cylinderHeight = 1.0;
            double cylinderRadius = 0.20;
            ContactableStaticCylinderRobot contactableStaticCylinderRobot = new ContactableStaticCylinderRobot("CylinderRobot"  + i, transform, cylinderHeight, cylinderRadius, YoAppearance.Gray());
            contactableStaticCylinderRobot.createAvailableContactPoints(1, 10, forceVectorScale, true);
            contactableRobots.add(contactableStaticCylinderRobot);

            //cylinderPosition.set(, , cylinderPosition.getZ() + cylinderOffset );
            cylinderPosition.setX(cylinderPosition.getX() + cylinderOffset);
            setGroundConatactPoint(contactableStaticCylinderRobot);
         }
      }

      if(ADD_BOXES) //change the dimensions of the box
      {
         double initialBoxPosition = doorPosition.getX();
         for(int i =0; i< NUmberofBoxes; i++)
         {
            ContactableSelectableBoxRobot boxRobot = new ContactableSelectableBoxRobot("BoxRobot" + i);
            boxRobot.setPosition(initialBoxPosition,0.0,0.1);
            boxRobot.createAvailableContactPoints(1,10, 0.02, true);
//            boxRobot.setGroundContactModel();

            contactableRobots.add(boxRobot);
            initialBoxPosition += 1.2;

//            GroundContactModel groundContactModel = new LinearStickSlipGroundContactModel(boxRobot, boxRobot.getRobotsYoVariableRegistry());
//           GroundContactModel groundContactModel = new LinearGroundContactModel(boxRobot, boxRobot.getRobotsYoVariableRegistry());
//           groundContactModel.setGroundProfile3D(combinedTerrainObject.getTerrainObjects().get(0));
//           boxRobot.setGroundContactModel(groundContactModel);
            setGroundConatactPoint(boxRobot);
         }
      }

      if(ADD_SPHERES)
      {
         double initialSpherePosition = doorPosition.getX();
         for(int i = 0; i < NumberofSpheres; i++)
         {
            ContactableSphereRobot sphereRobot = new ContactableSphereRobot("sphere" + i); //not adding contact points

            sphereRobot.setMass(1.0);
            sphereRobot.setPosition(initialSpherePosition, 0.0, ContactableSphereRobot.getDefaultRadius() + 0.1);

            Point3D[] contactPointOffset = SpiralBasedAlgorithm.generatePointsOnSphere(ContactableSphereRobot.getDefaultRadius(),50); //lookup rolling sphere e.g
            for(int j = 0; j < contactPointOffset.length; j++)
            {
               Point3D contactPointoffset = contactPointOffset[i];
               GroundContactPoint groundContactPoint = new GroundContactPoint("ballGC" + j,new Vector3D(contactPointoffset), sphereRobot);
               sphereRobot.getRootJoints().get(0).addGroundContactPoint(groundContactPoint);
            }
            sphereRobot.createAvailableContactPoints(1,10,0.02,true);
            contactableRobots.add(sphereRobot);
            initialSpherePosition += 0.5;

            setGroundConatactPoint(sphereRobot);
         }
      }
   }


   public static CombinedTerrainObject3D setUpGround(String name)
   {
      CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D(name);

      //    URL fileURL = DRCDemo01NavigationEnvironment.class.getClassLoader().getResource("Textures/ground2.png");
      YoAppearanceTexture texture = new YoAppearanceTexture("Textures/ground2.png");

      RigidBodyTransform location = new RigidBodyTransform();
      location.setTranslation(new Vector3D(0, 0, -0.5)); //the ground is 50 cm thick!!!

      RotatableBoxTerrainObject newBox = new RotatableBoxTerrainObject(new Box3D(location, 45, 45, 1), texture); //50cm  on both the sides
      combinedTerrainObject.addTerrainObject(newBox);
      RotatableBoxTerrainObject newBox2 = new RotatableBoxTerrainObject(new Box3D(location, 200, 200, 0.75), YoAppearance.DarkGray()); //37.5cm up and down
      combinedTerrainObject.addTerrainObject(newBox2);

      return combinedTerrainObject;
   }
   public void setGroundConatactPoint(Robot robot)
   {
      if (ADD_SPHERES)
      {
         double kXY = 1000.0; //1422.0;
         double bXY = 100.0; //150.6;
         double kZ = 20.0; //50.0;
         double bZ = 50.0; //1000.0;

         GroundContactModel groundContactModel = new LinearGroundContactModel(robot, kXY, bXY, kZ, bZ, robot.getRobotsYoVariableRegistry()); //verify if the ground contact model you are using is accurate to your situation
         groundContactModel.setGroundProfile3D(combinedTerrainObject.getTerrainObjects().get(0));
         robot.setGroundContactModel(groundContactModel);
      }

      else
      {
         GroundContactModel groundContactModel = new LinearGroundContactModel(robot, robot.getRobotsYoVariableRegistry()); //verify if the ground contact model you are using is accurate to your situation
         groundContactModel.setGroundProfile3D(combinedTerrainObject.getTerrainObjects().get(0));
         robot.setGroundContactModel(groundContactModel);
      }
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
   public ArrayList<Robot> getEnvironmentRobots()
   {
      return (ArrayList<Robot>) contactableRobots;
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
