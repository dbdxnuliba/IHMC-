package us.ihmc.simulationConstructionSetTools.util.environments.environmentRobots;

import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.shape.primitives.Sphere3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.*;
import us.ihmc.robotics.geometry.RotationalInertiaCalculator;
import us.ihmc.simulationconstructionset.*;
import us.ihmc.yoVariables.registry.*;
import us.ihmc.yoVariables.variable.*;

import java.util.*;

public class ContactableSphereRobot extends ContactableRobot
{
   private static final double DEFAULT_RADIUS = 0.30;
   private static final double DEFAULT_MASS = 10.0;
   private static boolean ROLLING = false;
   private final FloatingJoint floatingJoint;
   private final Sphere3D originalSphere3d, currentSphere3d;
   private YoGraphicsListRegistry yoGraphicsListRegistry1 = new YoGraphicsListRegistry();
   private final YoVariableRegistry registry = new YoVariableRegistry("RollingSphere");
   private YoGraphicsList yoGraphicsList = new YoGraphicsList("init");
   private YoGraphicPosition yoGraphicPosition;// = new YoGraphicPosition("init", new YoFramePoint3D(0.0,0.0,0.0,referenceFrame), 0.01, YoAppearance.Red());
   private YoGraphicVector yoGraphicVector;// =  new YoGraphicVector("initf", new YoFramePoint3D(0.0,0.0,0.0,referenceFrame), new YoFrameVector3D(0.0,0.0,0.0,referenceFrame), 1.0/50.0);

   private Link sphereLink;

   public ContactableSphereRobot()
   {
      this("ContactableSphereRobot");
   }

   public ContactableSphereRobot(String name)
   {
      this(name, DEFAULT_RADIUS,DEFAULT_MASS,YoAppearance.EarthTexture());
   }

   public ContactableSphereRobot(String name, double radius, double mass, AppearanceDefinition color)
   {
      super(name);

      floatingJoint = new FloatingJoint("base", new Vector3D(0.0, 0.0, 0.0), this);

      sphereLink = ball(radius, mass, color);
      floatingJoint.setLink(sphereLink);
      this.addRootJoint(floatingJoint);

      originalSphere3d = new Sphere3D(radius);
      currentSphere3d = new Sphere3D(radius);

      if(ROLLING)
      {
         int N = 8; //8;

         for (int i = 0; i < N; i++)
         {
            double latitude = -Math.PI / 2.0 + (i * Math.PI) / N;

            int nForThisLatitude = (int) ((Math.cos(latitude) * N) + 0.5);

            for (int j = 0; j < nForThisLatitude; j++)
            {
               double longitude = (j * 2.0 * Math.PI) / nForThisLatitude;

               double z = DEFAULT_RADIUS * Math.sin(latitude);
               double x = DEFAULT_RADIUS * Math.cos(latitude) * Math.cos(longitude);
               double y = DEFAULT_RADIUS * Math.cos(latitude) * Math.sin(longitude);

               String gcName = "gc" + i + "_" + j;
               GroundContactPoint gc = new GroundContactPoint(gcName, new Vector3D(x, y, z), this);
               floatingJoint.addGroundContactPoint(gc);

               yoGraphicPosition= new YoGraphicPosition(gcName + "Position", gc.getYoPosition(), 0.01, YoAppearance.Red());
               yoGraphicsListRegistry1.registerYoGraphic("FallingSphereGCPoints", yoGraphicPosition);

               yoGraphicVector = new YoGraphicVector(gcName + "Force", gc.getYoPosition(), gc.getYoForce(), 1.0/50.0);
               yoGraphicsListRegistry1.registerYoGraphic("FallingSphereForces", yoGraphicVector);
            }
         }
         this.getRobotsYoVariableRegistry().addChild(registry);
         this.addYoGraphicsListRegistry(yoGraphicsListRegistry1);
      }
   }

   public static void setROLLING(boolean ROLLING)
   {
      ContactableSphereRobot.ROLLING = ROLLING;
   }

   public boolean getROLLING()
   {
      return ContactableSphereRobot.ROLLING;
   }

   public YoGraphicsListRegistry getYoGraphicsListRegistry()
   {
      return yoGraphicsListRegistry1;
   }

   public YoGraphicsList getYoGraphicsList()
   {
      return yoGraphicsList;
   }

   private Link ball(double radius, double mass, AppearanceDefinition color)
   {
      Link ret = new Link("ball");

      ret.setMass(mass);

      ret.setMomentOfInertia(RotationalInertiaCalculator.getRotationalInertiaMatrixOfSolidEllipsoid(mass, radius, radius, radius));

      ret.setComOffset(0.0, 0.0, 0.0);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addSphere(radius, color);
      ret.setLinkGraphics(linkGraphics);

      return ret;
   }

   public static double getDefaultRadius()
   {
      return DEFAULT_RADIUS;
   }

   @Override
   public synchronized boolean isPointOnOrInside(Point3D pointInWorldToCheck)
   {
      return currentSphere3d.isPointInside(pointInWorldToCheck);
   }

   @Override
   public boolean isClose(Point3D pointInWorldToCheck)
   {
      return isPointOnOrInside(pointInWorldToCheck);
   }

   @Override
   public synchronized void closestIntersectionAndNormalAt(Point3D intersectionToPack, Vector3D normalToPack, Point3D pointInWorldToCheck)
   {
      currentSphere3d.evaluatePoint3DCollision(pointInWorldToCheck, intersectionToPack, normalToPack);
   }

   @Override
   public void setMass(double mass)
   {
      sphereLink.setMass(mass);
   }

   @Override
   public void setMomentOfInertia(double Ixx, double Iyy, double Izz)
   {
      sphereLink.setMomentOfInertia(Ixx, Iyy, Izz);
   }

   @Override
   public FloatingJoint getFloatingJoint()
   {
      return floatingJoint;
   }


   @Override
   public void update()
   {
      super.update();
      updateCurrentSphere3d();
   }

   private final RigidBodyTransform temporaryTransform3D = new RigidBodyTransform();

   private synchronized void updateCurrentSphere3d()
   {
      floatingJoint.getTransformToWorld(temporaryTransform3D);
      currentSphere3d.set(originalSphere3d);
      currentSphere3d.applyTransform(temporaryTransform3D);
   }
}
