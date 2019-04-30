package us.ihmc.robotEnvironmentAwareness.fusion.objectDetection;

import java.util.ArrayList;
import java.util.List;

import controller_msgs.msg.dds.DoorParameterPacket;
import sensor_msgs.msg.dds.RegionOfInterest;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.Ros2Node;

public class DoorParameterCalculator extends AbstractObjectParameterCalculator<DoorParameterPacket>
{
   private static final int numberOfTimesToRANSAC = 100;
   private static final double thresholdOfInlier = 0.05;
   private final List<Point3D> pointsOnPlane = new ArrayList<>();

   private final Vector3D principalXAxis = new Vector3D();
   private final Vector3D principalYAxis = new Vector3D();
   private final Vector3D principalZAxis = new Vector3D();
   
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
      for (int i = 0; i < numberOfTimesToRANSAC; i++)
      {

      }
      pointsOnPlane.clear();
   }

   private void findPrincipalComponent()
   {
      
   }
   
   private void findRectangle()
   {
      for (int i = 0; i < numberOfSearchingRectangle; i++)
      {

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
}