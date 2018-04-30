package us.ihmc.manipulation.planning.rrt.exploringSpatial;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.rotationConversion.RotationMatrixConversion;
import us.ihmc.euclid.rotationConversion.RotationVectorConversion;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;

public class SE3InterpolationTest
{
   public SE3InterpolationTest()
   {
      // INPUT configuration space name.
      RigidBodyTransform origin = new RigidBodyTransform();
      double yaw = Math.PI * 30 / 180;
      double pitch = Math.PI * 20 / 180;
      double roll = Math.PI * 10 / 100;

      double alpha = 0.5;

      RigidBodyTransform full = new RigidBodyTransform(origin);
      full.appendYawRotation(yaw);
      full.appendPitchRotation(pitch);
      full.appendRollRotation(roll);

      // Case 1 : when interpolate yaw pitch independently.
      RigidBodyTransform halfCase1 = new RigidBodyTransform(origin);
      halfCase1.appendYawRotation(yaw * alpha);
      halfCase1.appendPitchRotation(pitch * alpha);
      halfCase1.appendRollRotation(roll * alpha);

      System.out.println("Case 1");
      System.out.println("full distance " + getOrientationDistance(origin, full));
      System.out.println("half distance " + getOrientationDistance(origin, halfCase1));
      System.out.println("compare " + (getOrientationDistance(origin, full) * alpha - getOrientationDistance(origin, halfCase1)));
      // **** not bad

      // Case 2 : when interpolate rotational vector space.
      Vector3D rvFull = new Vector3D();
      RotationVectorConversion.convertMatrixToRotationVector(full.getRotationMatrix(), rvFull);
      System.out.println("Case 2");
      System.out.println("rvFull " + rvFull);

      Vector3D rvHalf = new Vector3D(rvFull.getX() * alpha, rvFull.getY() * alpha, rvFull.getZ() * alpha);
      System.out.println("rvHalf " + rvHalf);
      
      RotationMatrix halfCase2RM = new RotationMatrix();
      RotationMatrixConversion.convertRotationVectorToMatrix(rvHalf, halfCase2RM);
      
      RigidBodyTransform halfCase2 = new RigidBodyTransform(origin);
      halfCase2.setRotation(halfCase2RM);
      
      System.out.println("full distance " + getOrientationDistance(origin, full));
      System.out.println("half distance " + getOrientationDistance(origin, halfCase2));
      System.out.println("compare " + (getOrientationDistance(origin, full) * alpha - getOrientationDistance(origin, halfCase2)));
      // **** perfect
      
      // Visualize SE3 into yaw pitch roll.
      // interpolate orientation by rotational vector space.
      // for SE3 as well.
   }

   // Compare orientation distance is proper or not.
   public static void main(String[] args)
   {
      new SE3InterpolationTest();
   }

   private double getOrientationDistance(RigidBodyTransform one, RigidBodyTransform two)
   {
      Quaternion qtOne = new Quaternion(one.getRotationMatrix());
      Quaternion qtTwo = new Quaternion(two.getRotationMatrix());

      return qtOne.distance(qtTwo);
   }
}
