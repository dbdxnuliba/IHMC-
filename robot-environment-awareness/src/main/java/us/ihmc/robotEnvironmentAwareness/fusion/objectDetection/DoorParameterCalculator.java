package us.ihmc.robotEnvironmentAwareness.fusion.objectDetection;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import controller_msgs.msg.dds.DoorParameterPacket;
import sensor_msgs.msg.dds.RegionOfInterest;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.linearAlgebra.PrincipalComponentAnalysis3D;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.Ros2Node;

public class DoorParameterCalculator extends AbstractObjectParameterCalculator<DoorParameterPacket>
{
   private static final int numberOfTimesToRANSAC = 100;
   private static final double thresholdOfInlier = 0.05;
   private final List<Point3DBasics> pointsOnPlane = new ArrayList<>();
   private final List<Point3DBasics> tempOutlierPoints = new ArrayList<>();

   private final PrincipalComponentAnalysis3D pca = new PrincipalComponentAnalysis3D();

   private static final int numberOfSearchingRectangle = 10;
   private static final SideDependentList<Point3D> bottomVertices = new SideDependentList<>();
   private static final SideDependentList<Point3D> topSideVertices = new SideDependentList<>();

   public DoorParameterCalculator(Ros2Node ros2Node, Class<DoorParameterPacket> packetType)
   {
      super(ros2Node, packetType);
      for (RobotSide robotSide : RobotSide.values)
      {
         bottomVertices.put(robotSide, new Point3D());
         topSideVertices.put(robotSide, new Point3D());
      }
   }

   /**
    * ROI(Door), ROI(Door Handle)
    */
   @Override
   public void calculate(RegionOfInterest... additionalROIs)
   {
      ransac();
      findPrincipalComponent();
      findRectangle();
      assignHingedPoint(additionalROIs[0]);
   }

   private void ransac()
   {
      Random random = new Random(0612L);
      int numberOfPointInROI = pointCloudToCalculate.size();

      int maximumNumberOfInlier = 0;
      PlaneEquation bestPlane = new PlaneEquation();

      for (int i = 0; i < numberOfTimesToRANSAC; i++)
      {
         int index1 = -1, index2 = -1, index3 = -1;
         index1 = random.nextInt(numberOfPointInROI);
         while (index1 == index2 || index2 == index3 || index3 == index1)
         {
            index2 = random.nextInt(numberOfPointInROI);
            index3 = random.nextInt(numberOfPointInROI);
         }

         PlaneEquation selectedPlane = new PlaneEquation(pointCloudToCalculate.get(index1), pointCloudToCalculate.get(index2),
                                                         pointCloudToCalculate.get(index3));

         int numberOfInlier = 0;
         for (int j = 0; j < numberOfPointInROI; j++)
         {
            Point3DBasics point = pointCloudToCalculate.get(j);
            double distance = selectedPlane.distance(point);
            if (distance <= thresholdOfInlier)
               numberOfInlier++;
         }

         if (numberOfInlier > maximumNumberOfInlier)
         {
            bestPlane.set(selectedPlane);
            maximumNumberOfInlier = numberOfInlier;
         }
      }

      pointsOnPlane.clear();
      for (int i = 0; i < numberOfPointInROI; i++)
      {
         Point3DBasics point = pointCloudToCalculate.get(i);
         double distance = bestPlane.distance(point);
         if (distance <= thresholdOfInlier)
            pointsOnPlane.add(point);
      }

      LogTools.info("Best Plane is " + bestPlane.getInformation());
      LogTools.info("Number of points to be fitted is " + pointsOnPlane.size());

      // TODO: remove
      tempOutlierPoints.clear();
      for (int i = 0; i < numberOfPointInROI; i++)
      {
         Point3DBasics point = pointCloudToCalculate.get(i);
         double distance = bestPlane.distance(point);
         if (distance > thresholdOfInlier)
            tempOutlierPoints.add(point);
      }

      Vector3D total = new Vector3D();
      for (Point3DBasics outPoint : tempOutlierPoints)
      {
         total.add(outPoint);
      }
      total.setX(total.getX() / tempOutlierPoints.size());
      total.setY(total.getY() / tempOutlierPoints.size());
      total.setZ(total.getZ() / tempOutlierPoints.size());

      LogTools.info("Number of points out of door is " + tempOutlierPoints.size() + ", X: " + total.getX() + ", Y: " + total.getY() + ", Z: " + total.getZ());
   }

   private void findPrincipalComponent()
   {
      pca.clear();
      pca.addAllDataPoints(pointsOnPlane);
      pca.compute();
   }

   private void findRectangle()
   {
      Point3D initialGuessCenter = new Point3D();
      pca.getMean(initialGuessCenter);

      RotationMatrix clusterdRotationMatrix = new RotationMatrix();
      pca.getPrincipalFrameRotationMatrix(clusterdRotationMatrix);

      //TODO: set center.
      FiniteRectangleCalculator finiteRectangleCalculator = new FiniteRectangleCalculator();
      double minimumArea = Double.POSITIVE_INFINITY;
      for (int i = 0; i < numberOfSearchingRectangle; i++)
      {
         double rotatingAngle = i * Math.PI / 2.0 / (numberOfSearchingRectangle - 1);
         clusterdRotationMatrix.appendYawRotation(rotatingAngle);

         //TODO: set center position.
         finiteRectangleCalculator.compute(pointsOnPlane, clusterdRotationMatrix);
         double area = finiteRectangleCalculator.area();
         if (area < minimumArea)
         {
            //TODO: get vertices.
            LogTools.info("minimum area is " + minimumArea);
         }
      }
   }

   private void assignHingedPoint(RegionOfInterest handleROI)
   {
      // TODO : select hinged vertex.
      if (handleROI == null)
      {
         // TODO : assume the handle located on left side of the image.
      }
      else
      {

      }
   }

   /**
    * Standard plane equation form.
    * a*x + b*y + c*z + d = 0.
    */
   private class PlaneEquation
   {
      private final Vector3D normalVector = new Vector3D();
      private double constantD = 0.0;

      private PlaneEquation()
      {

      }

      /**
       * a*x + b*y + c*z + d = 0.
       * @param normal: a, b, c
       * @param constant: d
       */
      private PlaneEquation(Vector3DBasics normal, double constant)
      {
         normalVector.set(normal);
         constantD = constant;
      }

      private PlaneEquation(Vector3DBasics normal, Tuple3DBasics pointOnPlane)
      {
         normalVector.set(normal);
         constantD = -normalVector.getX() * pointOnPlane.getX() - normalVector.getY() * pointOnPlane.getY() - normalVector.getZ() * pointOnPlane.getZ();
      }

      private PlaneEquation(Tuple3DBasics point1, Tuple3DBasics point2, Tuple3DBasics point3)
      {
         Vector3D vector1To2 = new Vector3D(point2);
         vector1To2.sub(point1);
         Vector3D vector1To3 = new Vector3D(point3);
         vector1To3.sub(point1);

         normalVector.cross(vector1To2, vector1To3);
         constantD = -normalVector.getX() * point1.getX() - normalVector.getY() * point1.getY() - normalVector.getZ() * point1.getZ();
      }

      private void set(PlaneEquation other)
      {
         normalVector.set(other.normalVector);
         constantD = other.constantD;
      }

      private double distance(Tuple3DBasics point)
      {
         return Math.abs(normalVector.getX() * point.getX() + normalVector.getY() * point.getY() + normalVector.getZ() * point.getZ() + constantD)
               / Math.sqrt(normalVector.lengthSquared());
      }

      private String getInformation()
      {
         return "A: " + normalVector.getX() + ", B: " + normalVector.getY() + ", C: " + normalVector.getZ() + ", D: " + constantD;
      }
   }

   private class FiniteRectangleCalculator
   {
      private final List<Point2DBasics> convertedPoints = new ArrayList<>();
      //TODO: fields for edge.

      private FiniteRectangleCalculator()
      {

      }

      private void compute(List<Point3DBasics> points, RotationMatrix clusteredRotationMatrix)
      {
         convertedPoints.clear();
         for (Point3DBasics point : points)
         {
            Point2D convertedPoint = new Point2D();
            // TODO: convert 3d to 2d.
            convertedPoints.add(convertedPoint);
         }

         // TODO: find edge.
      }

      private double area()
      {
         return 0.0;
      }

      // TODO: getter for vertex.
   }
}