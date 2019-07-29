package us.ihmc.simulationConstructionSetTools.util.environments;

import com.github.quickhull3d.*;
import controller_msgs.msg.dds.*;
import javafx.geometry.*;
import javafx.geometry.Point2D;
import us.ihmc.communication.*;
import us.ihmc.euclid.axisAngle.*;
import us.ihmc.euclid.geometry.*;
import us.ihmc.euclid.referenceFrame.*;
//import us.ihmc.euclid.shape.*;
import us.ihmc.euclid.shape.primitives.*;
import us.ihmc.euclid.transform.*;
import us.ihmc.euclid.tuple2D.*;
import us.ihmc.euclid.tuple3D.*;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.appearance.*;
import us.ihmc.graphicsDescription.yoGraphics.*;
import us.ihmc.humanoidRobotics.communication.packets.*;
import us.ihmc.jMonkeyEngineToolkit.jme.util.*;
import us.ihmc.robotics.geometry.*;
import us.ihmc.simulationConstructionSetTools.robotController.*;
import us.ihmc.simulationConstructionSetTools.util.environments.environmentRobots.*;
import us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments.*;
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
   private final double stepLength;
   private final double wallHeight;
   private final double stepUpHeight;
   private final double initialSpherePosition;


   private final boolean ADD_BOXES = false;
   private final boolean ADD_CYLINDER = false;
   private final boolean ADD_SPHERES = true;
   private final boolean ADD_ROLLING_SPHERE = false;
   private final boolean ADD_GOAL_POST = true;
   private final boolean ENABLE_GOAL_POST_ORIENTATION = true;

   private FramePose3D doorframepose = new FramePose3D();
   private final int NUmberofBoxes = 1;
   private final int NumberofSpheres = 1;
   private final int NumberofCylinders = 1;

   private final List<Robot> contactableRobots = new ArrayList<>();  //list of robots
   private final CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D(getClass().getSimpleName());
   private final ArrayList<ExternalForcePoint> contactPoints = new ArrayList<ExternalForcePoint>(); //list of external contact points
   private ContactableSphereRobot sphereRobot;
   private ContactableRollingSphereRobot rollingSphere;
   private Point3D GoalPostPosition;
   private static Point3D fiducialPosition;
   private static Point3D doorPosition;

//   private final Point3D positionOfGoalPost = new Point3D(6.72,-1.78,0.0); //goalpostposition
   private final Vector2D position = new Vector2D(10, -0.5);
   private final Orientation2D orientation2D = new Orientation2D(0.0);

   private final Pose2D poseOfGoalPost = new Pose2D(position,orientation2D);

   double wallOffSet = 0.8;

   double xstart = 0.5;
   double stairStart = 0.5 + xstart;
   double wallWidth = 0.2;
   double doorAngle = -1.5708;

   double forceVectorScale = 1.0 / 50.0;


//   private ReferenceFrame ballFrame = new ReferenceFrame("doorFrame", ReferenceFrame.getWorldFrame())
//   {
//      @Override
//      protected void updateTransformToParent(RigidBodyTransform transformToParent)
//      {
////         sphereRobot.getBodyTransformToWorld(transformToParent);
//         transformToParent.set(ReferenceFrame.getWorldFrame().getTransformToWorldFrame());
//      }
//   };




   public StepUpDoor(double stepLength, double wallHeight, double stepUpHeight)//, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      double wallInitialOffSet = 1.0 + 2.5* stepLength; // this means that the wall is ahead of the steps by one stepLength to allow you just suffcient space to use footstep planner tool

      doorPosition = new Point3D(wallInitialOffSet + stepLength + 1.5, 0.5, 0.0);
      fiducialPosition = new Point3D(wallInitialOffSet + stepLength + 1.25, 0.0, 1.25);

      initialSpherePosition = doorPosition.getX() + 3.0;

//      IHMCROS2Publisher<DoorLocationPacket> publisher = ROS2Tools.createPublisher(ROS2N ros2Node, DoorLocationPacket.class,
//                                                                                  IHMCHumanoidBehaviorManager.getSubscriberTopicNameGenerator(coactiveBehaviorsNetworkManager.getRobotName()));


      this.stepLength = stepLength;
      this.wallHeight = wallHeight;
      this.stepUpHeight = stepUpHeight;

      combinedTerrainObject.addTerrainObject(setUpGround("Ground"));

      combinedTerrainObject.addBox(stairStart , -1.0, stairStart+stepLength, 1.0, stepUpHeight, YoAppearance.Gray()); //the robot is the starting location

      //building walls
      AppearanceDefinition appearance = YoAppearance.Gray();
      appearance.setTransparency(0.25);
      combinedTerrainObject.addBox(wallInitialOffSet, wallOffSet, wallInitialOffSet+ stepLength, wallOffSet+wallWidth ,wallHeight, appearance); //left wall
      combinedTerrainObject.addBox(wallInitialOffSet, -wallOffSet, wallInitialOffSet+stepLength, -(wallOffSet+wallWidth),wallHeight, appearance); //right wall


      doorframepose = new FramePose3D(new Pose3D(doorPosition.getX(),doorPosition.getY(),doorPosition.getZ(),doorAngle,0.0,0.0));

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
            cylinderPosition.setX(cylinderPosition.getX() + cylinderOffset);
            setGroundConatactModel(contactableStaticCylinderRobot);
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
            contactableRobots.add(boxRobot);
            initialBoxPosition += 1.2;
            setGroundConatactModel(boxRobot);
         }
      }

//      ballFrame.update();
//      System.out.println(ballFrame.getTransformToDesiredFrame(ReferenceFrame.getWorldFrame()));
//      DoorLocationPacket doorLocationPacket = HumanoidMessageTools.createDoorLocationPacket();
//      //HumanoidMessageTools.createDoorLocationPacket(ballFrame.getTransformToDesiredFrame(ReferenceFrame.getWorldFrame()));
//      publisher.publish(doorLocationPacket);


      if(ADD_SPHERES)
      {
         ContactableSphereRobot.setROLLING(true);

         for(int i = 0; i < NumberofSpheres; i++)
         {
            sphereRobot = new ContactableSphereRobot("sphere" + i);
            sphereRobot.setMass(1.0);
            sphereRobot.setPosition(initialSpherePosition, 0.0, ContactableSphereRobot.getDefaultRadius() + 0.01);
            Point3D[] contactPointOffset;
            if (sphereRobot.getROLLING() != true);
            {
               if(ContactableSphereRobot.getDefaultRadius() > 0.5)
               {
                  contactPointOffset = SpiralBasedAlgorithm.generatePointsOnSphere(ContactableSphereRobot.getDefaultRadius(), 50);
               }
               else if (ContactableSphereRobot.getDefaultRadius() >= 0.25 && ContactableSphereRobot.getDefaultRadius() <= 0.5)
               {
                  contactPointOffset = SpiralBasedAlgorithm.generatePointsOnSphere(ContactableSphereRobot.getDefaultRadius(), 10);
               }
               else
               {
                  contactPointOffset = SpiralBasedAlgorithm.generatePointsOnSphere(ContactableSphereRobot.getDefaultRadius(), 10);
               }

               for (int j = 0; j < contactPointOffset.length; j++)
               {
                  Point3D contactPointoffset = contactPointOffset[i];
                  GroundContactPoint groundContactPoint = new GroundContactPoint("ballGC" + j, new Vector3D(contactPointoffset), sphereRobot);
                  sphereRobot.getRootJoints().get(0).addGroundContactPoint(groundContactPoint);
               }
            }
            //the last two variable are just cosmetic things like ratio to visualize, and true to visulize the force vector
            sphereRobot.createAvailableContactPoints(1, 10, 0.02, true);
            contactableRobots.add(sphereRobot);
            setGroundConatactModel(sphereRobot);
         }
      }

      if(ADD_ROLLING_SPHERE)
      {
         rollingSphere = new ContactableRollingSphereRobot("rollingSphere");
         rollingSphere.setMass(1.0);
         ContactableRollingSphereRobot.setDefaultRadius(ContactableSphereRobot.getDefaultRadius());
         rollingSphere.setPosition(initialSpherePosition, 0.0, ContactableRollingSphereRobot.getDefaultRadius() + 0.01);
         rollingSphere.createAvailableContactPoints(1,10,0.02,true);
         contactableRobots.add(rollingSphere);
         rollingSphere.setGravity(0.0);
         setGroundConatactModel(rollingSphere);
      }

      if(ADD_GOAL_POST)
      {
         //z,x,y is the offset from middle
         //eventually change this to a form of constructor that given one single point it can build all that stuff
         //say you x,y,z from the user which is the midpoint of top rod'sprojection on ground
         Vector3D support1;
         Vector3D support2;
         Vector3D support3;
         Vector3D support4;
         Vector3D topRod;
         double heightOffTheGround = 1.0; //lsin(theta)
         final double desiredBendAngleofSlantSupports = (Math.PI)/4; //theta (keep it as 45 degrees!! else the formulas gets messed up)
         double rodLength = heightOffTheGround/Math.sin(desiredBendAngleofSlantSupports); //l value
         double supportOffsetFromCenterofTopRod = 0.5;
         double initialoffset = 10;
         if(!ENABLE_GOAL_POST_ORIENTATION)
         {

            support1 = new Vector3D(initialoffset + (rodLength*Math.cos(desiredBendAngleofSlantSupports))/2.0,supportOffsetFromCenterofTopRod,heightOffTheGround/2.0);  //slant support
            support2 = new Vector3D(initialoffset + (rodLength*Math.cos(desiredBendAngleofSlantSupports))/2.0,-supportOffsetFromCenterofTopRod,0.5); //slant support
            topRod = new Vector3D(initialoffset ,0.0,heightOffTheGround); //top rod
            support3 = new Vector3D(initialoffset ,supportOffsetFromCenterofTopRod,heightOffTheGround/2.0); //vertical support
            support4 = new Vector3D(initialoffset ,-supportOffsetFromCenterofTopRod,heightOffTheGround/2.0); //vertical support
         }
         else
         {
            topRod = new Vector3D(poseOfGoalPost.getX(),poseOfGoalPost.getY(),heightOffTheGround);//the z-value of 1.0 being the height(length is the height value in constructor) is fixed. can make it variable afterwards too
            support3 = new Vector3D(poseOfGoalPost.getX(),poseOfGoalPost.getY() + supportOffsetFromCenterofTopRod,heightOffTheGround/2.0);
            support4 = new Vector3D(poseOfGoalPost.getX(),poseOfGoalPost.getY() - supportOffsetFromCenterofTopRod,heightOffTheGround/2.0);
            support1 = new Vector3D(poseOfGoalPost.getX()+(rodLength*Math.cos(desiredBendAngleofSlantSupports))/2.0,poseOfGoalPost.getY() + supportOffsetFromCenterofTopRod,heightOffTheGround/2.0);
            support2 = new Vector3D(poseOfGoalPost.getX()+(rodLength*Math.cos(desiredBendAngleofSlantSupports))/2.0,poseOfGoalPost.getY() - supportOffsetFromCenterofTopRod,heightOffTheGround/2.0);
         }
         GoalPostPosition = new Point3D(topRod.getX(),topRod.getY(),0.0);

         YoAppearance app = new YoAppearance();
         final double pitchDownDegreesForSlantRods = 135;
         final double pitchDownDegreesForTopRods = 90;
//         double yawDegree = poseOfGoalPost.getOrientation().getYaw();
         CylinderTerrainObject goalPost1_1 = new CylinderTerrainObject(support1,pitchDownDegreesForSlantRods,0.0,rodLength,0.02,app.Black());
         CylinderTerrainObject goalPost1_2 = new CylinderTerrainObject(support2,pitchDownDegreesForSlantRods,0.0, rodLength,0.02,app.Black());
         CylinderTerrainObject goalPost1_3 = new CylinderTerrainObject(topRod,pitchDownDegreesForTopRods,-90.0,1.0,0.02,app.Black());
         CylinderTerrainObject goalPost1_4 = new CylinderTerrainObject(support3,0.0,0.0,1.0,0.02,app.Black());
         CylinderTerrainObject goalPost1_5 = new CylinderTerrainObject(support4,0.0,0.0,1.0,0.02,app.Black());


         combinedTerrainObject.addTerrainObject(goalPost1_1);
         combinedTerrainObject.addTerrainObject(goalPost1_2);
         combinedTerrainObject.addTerrainObject(goalPost1_3);
         combinedTerrainObject.addTerrainObject(goalPost1_4);
         combinedTerrainObject.addTerrainObject(goalPost1_5);
      }
   }

   public static Point3D getFiducialPosition()
   {
      return fiducialPosition;
   }

   public Point3D getBallLocation()
   {
      return new Point3D(getInitialSpherePosition()-ContactableSphereRobot.getDefaultRadius(),0.0,ContactableSphereRobot.getDefaultRadius());
   }

   public Point3D getGoalPostPosition()
   {
      return GoalPostPosition;
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

   public ContactableSphereRobot getSphereRobot()
   {
      return sphereRobot;
   }

   public void setGroundConatactModel(Robot robot)
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

      else if(ADD_ROLLING_SPHERE)
      {
         double kXY = 1000.0; //1422.0;
         double bXY = 100.0; //150.6;
         double kZ = 20.0; //50.0;
         double bZ = 50.0; //1000.0;

         GroundContactModel groundContactModel = new LinearGroundContactModel(robot, kXY, bXY, kZ, bZ, robot.getRobotsYoVariableRegistry());
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

   public double getInitialSpherePosition()
   {
      return initialSpherePosition;
   }

   public Point2D getInitialSpherePos()
   {
      return new Point2D(initialSpherePosition,0.0);
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
//   public static void main(String[] args)
//   {
//      StepUpDoor abc = new StepUpDoor(0.0,0.0,0.0);
//   }

}


