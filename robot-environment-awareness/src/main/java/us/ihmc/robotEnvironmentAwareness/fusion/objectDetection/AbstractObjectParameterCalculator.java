package us.ihmc.robotEnvironmentAwareness.fusion.objectDetection;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import boofcv.struct.calib.IntrinsicParameters;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import sensor_msgs.msg.dds.RegionOfInterest;
import us.ihmc.commons.MathTools;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.idl.IDLSequence.Float;
import us.ihmc.robotEnvironmentAwareness.fusion.projection.PointCloudProjectionHelper;
import us.ihmc.ros2.Ros2Node;

public abstract class AbstractObjectParameterCalculator<T extends Packet<?>>
{
   //private static final IntrinsicParameters intrinsicParameters = PointCloudProjectionHelper.multisenseIntrinsicParameters;
   private static final IntrinsicParameters intrinsicParameters = new IntrinsicParameters();
   protected final List<Point3DBasics> pointCloudToCalculate;

   private final IHMCROS2Publisher<T> packetPublisher;
   protected final AtomicReference<T> newPacket = new AtomicReference<>(null);

   public AbstractObjectParameterCalculator(Ros2Node ros2Node, Class<T> packetType)
   {
      pointCloudToCalculate = new ArrayList<Point3DBasics>();
      packetPublisher = ROS2Tools.createPublisher(ros2Node, packetType, ROS2Tools.getDefaultTopicNameGenerator());
      
      intrinsicParameters.setFx(601.5020141601562);
      intrinsicParameters.setFy(602.0339965820312);
      intrinsicParameters.setCx(520.92041015625);
      intrinsicParameters.setCy(273.5399169921875);
      
      System.out.println(intrinsicParameters.getCx());
      System.out.println(ROS2Tools.getDefaultTopicNameGenerator().generateTopicName(packetType));
   }

   public void getPointCloudInROI(StereoVisionPointCloudMessage pointCloudMessage, RegionOfInterest roi)
   {
      pointCloudToCalculate.clear();
      Float messageData = pointCloudMessage.getPointCloud();
      int numberOfPoints = messageData.size() / 3;
      for (int i = 0; i < numberOfPoints; i++)
      {
         Point3D point = new Point3D(messageData.get(3 * i + 0), messageData.get(3 * i + 1), messageData.get(3 * i + 2));
         Point2D projectedPixel = new Point2D();
         PointCloudProjectionHelper.projectMultisensePointCloudOnImage(point, projectedPixel, intrinsicParameters);

         //System.out.println(""+projectedPixel.getX()+" "+projectedPixel.getY()+" "+roi.getXOffset()+" "+roi.getYOffset());
         if (MathTools.intervalContains(projectedPixel.getX(), roi.getXOffset(), roi.getXOffset() + roi.getWidth()))
         {
            if (MathTools.intervalContains(projectedPixel.getY(), roi.getYOffset(), roi.getYOffset() + roi.getHeight()))
            {
               pointCloudToCalculate.add(point);
            }
         }
      }
      System.out.println("total number of points in roi " + pointCloudToCalculate.size());
   }

   public void publish()
   {
      packetPublisher.publish(newPacket.get());
   }

   public abstract void calculateAndPackResult();
}
