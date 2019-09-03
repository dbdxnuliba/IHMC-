package us.ihmc.simulationConstructionSetTools.util.environments;

import java.util.List;

import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.simulationConstructionSetTools.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.ground.RotatableBoxTerrainObject;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

public class GroundAsABoxEnvironment implements CommonAvatarEnvironmentInterface
{
   private final CombinedTerrainObject3D flatGround;

   private double groundLength = 4.0;
   private double groundWidth = 4.0;
   private double groundThickness = 0.05;
   
   public GroundAsABoxEnvironment()
   {
      this(false, 0.0);
   }
   
   public GroundAsABoxEnvironment(boolean addWall)
   {
      this(addWall, 0.0);
   }
   
   public GroundAsABoxEnvironment(boolean addWall, double groundAngle)
   {
      CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D("GroundAsABox");

      RigidBodyTransform location = new RigidBodyTransform();
      location.setTranslation(new Vector3D(0, 0, -0.5 * groundThickness));
      location.appendPitchRotation(groundAngle);
      RotatableBoxTerrainObject newBox = new RotatableBoxTerrainObject(new Box3D(location, groundLength, groundWidth, groundThickness), YoAppearance.Green());
      combinedTerrainObject.addTerrainObject(newBox);

      flatGround = combinedTerrainObject;
      
      if(addWall)
         addWall();
   }

   public void addWall()
   {
      double offsetX = -0.75;
      double offsetY = 0.0;
      double xRotation = 0.0;
      double yRotation = Math.PI / 8.0;
      flatGround.addTerrainObject(createWall(groundLength, groundWidth, groundThickness, offsetX, offsetY, xRotation, yRotation));

      offsetX = 0.75;
      offsetY = 0.0;
      xRotation = 0.0;
      yRotation = -Math.PI / 8.0;
      flatGround.addTerrainObject(createWall(groundLength, groundWidth, groundThickness, offsetX, offsetY, xRotation, yRotation));

      offsetX = 0.0;
      offsetY = -0.75;
      xRotation = -Math.PI / 8.0;
      yRotation = 0.0;
      flatGround.addTerrainObject(createWall(groundLength, groundWidth, groundThickness, offsetX, offsetY, xRotation, yRotation));

      offsetX = 0.0;
      offsetY = 0.75;
      xRotation = Math.PI / 8.0;
      yRotation = 0.0;
      flatGround.addTerrainObject(createWall(groundLength, groundWidth, groundThickness, offsetX, offsetY, xRotation, yRotation));
   }

   private RotatableBoxTerrainObject createWall(double floorLength, double floorWidth, double floorThickness, double offsetX, double offsetY, double xRotation,
                                             double yRotation)
   {
      RigidBodyTransform location = new RigidBodyTransform();
      location.appendTranslation(offsetX, offsetY, -floorThickness);
      location.appendRollRotation(xRotation);
      location.appendPitchRotation(yRotation);
      
      RotatableBoxTerrainObject wall = new RotatableBoxTerrainObject(new Box3D(location, groundLength, groundWidth, groundThickness), YoAppearance.Green());
      return wall;
   }

   @Override
   public TerrainObject3D getTerrainObject3D()
   {
      return flatGround;
   }

   @Override
   public List<Robot> getEnvironmentRobots()
   {
      return null;
   }

   @Override
   public void createAndSetContactControllerToARobot()
   {
   }

   @Override
   public void addContactPoints(List<? extends ExternalForcePoint> externalForcePoints)
   {
   }

   @Override
   public void addSelectableListenerToSelectables(SelectableObjectListener selectedListener)
   {
   }
}