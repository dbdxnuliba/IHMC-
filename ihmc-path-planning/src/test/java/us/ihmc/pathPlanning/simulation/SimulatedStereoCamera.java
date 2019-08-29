package us.ihmc.pathPlanning.simulation;

import boofcv.struct.calib.IntrinsicParameters;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.javaFXVisualizers.IdMappedColorFunction;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.yoVariables.providers.DoubleProvider;

import java.awt.*;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.*;
import java.util.concurrent.atomic.AtomicLong;

public class SimulatedStereoCamera implements Runnable
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final ReferenceFrame cameraSensorFrame;

   private PlanarRegionsList planarRegionsList;

   private final int widthPixels;
   private final int heightPixels;
   private final double minimumDistance;
   private final double widthFieldOfView;
   private final double heightFieldOfView;

   private final IHMCROS2Publisher<StereoVisionPointCloudMessage> stereoVisionPublisher;
   private double startTime = Double.NaN;

   private boolean start = false;
   private final ScheduledExecutorService executorService = Executors.newScheduledThreadPool(1, ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));
   private final ScheduledFuture<?> scheduledFuture;

   public SimulatedStereoCamera(Ros2Node ros2Node, ReferenceFrame cameraSensorFrame, RGBDSensorDescription sensorDescription,
                                double updateFrequencyHz)
   {
      this(ros2Node, cameraSensorFrame, sensorDescription.getPixelsWide(), sensorDescription.getPixelsHigh(),
           sensorDescription.getWidthFieldOfView(), sensorDescription.getHeightFieldOfView(), sensorDescription.getMinimumRange(), updateFrequencyHz);
   }

   /**
    * The camera sensor frame is defined with a positive x forward and z up, as if you were looking at the picture.
    */
   public SimulatedStereoCamera(Ros2Node ros2Node, ReferenceFrame cameraSensorFrame, int widthPixels, int heightPixels,
                                double widthFieldOfView, double heightFieldOfView, double minimumDistance, double updateFrequencyHz)
   {
      this.widthPixels = widthPixels;
      this.heightPixels = heightPixels;
      this.cameraSensorFrame = cameraSensorFrame;
      this.widthFieldOfView = widthFieldOfView;
      this.heightFieldOfView = heightFieldOfView;
      this.minimumDistance = minimumDistance;

      stereoVisionPublisher = ROS2Tools.createPublisher(ros2Node, StereoVisionPointCloudMessage.class, ROS2Tools.getDefaultTopicNameGenerator());

      scheduledFuture = executorService.scheduleAtFixedRate(this, 0, (long) (1.0 / updateFrequencyHz), TimeUnit.SECONDS);
   }

   public void setPlanarRegionsEnvironment(PlanarRegionsList planarRegionsList)
   {
      this.planarRegionsList = planarRegionsList;
   }

   public void start()
   {
      start = true;
   }

   public void stop()
   {
      start = false;
   }

   public void destroy()
   {
      scheduledFuture.cancel(true);
   }

   @Override
   public void run()
   {
      if (Double.isNaN(startTime))
         startTime = Conversions.nanosecondsToSeconds(System.nanoTime());

      if (!start)
         return;

      double widthIncrement = widthFieldOfView / widthPixels;
      double heightIncrement = heightFieldOfView / heightPixels;

      FrameVector3D rayDirection = new FrameVector3D(cameraSensorFrame);
      FramePoint3D rayOrigin = new FramePoint3D(cameraSensorFrame);
      rayOrigin.changeFrame(worldFrame);

      List<Point3DReadOnly> pointCloud = new ArrayList<>();
      List<Color> colors = new ArrayList<>();

      for (int widthIdx = 0; widthIdx < widthPixels; widthIdx++)
      {
         for (int heightIdx = 0; heightIdx < heightPixels; heightIdx++)
         {
            double widthAngle = 0.5 * widthFieldOfView - widthIdx * widthIncrement;
            double heightAngle = 0.5 * heightFieldOfView - heightIdx * heightIncrement;
            rayDirection.setToZero(cameraSensorFrame);
            rayDirection.set(Math.cos(widthAngle) * Math.cos(heightAngle), Math.cos(heightAngle) * Math.sin(widthAngle), Math.sin(heightAngle));
            rayDirection.changeFrame(worldFrame);

            Pair<Point3D, PlanarRegion> intersectionPair = PlanarRegionTools.intersectRegionsWithRay(planarRegionsList, rayOrigin, rayDirection);
            FramePoint3D intersectionWidthWorld = new FramePoint3D(worldFrame, intersectionPair.getLeft());
            intersectionWidthWorld.changeFrame(cameraSensorFrame);
            if (intersectionPair.getLeft().distanceFromOrigin() < minimumDistance)
               continue;

            pointCloud.add(intersectionWidthWorld);
            javafx.scene.paint.Color color = IdMappedColorFunction.INSTANCE.apply(intersectionPair.getRight().getRegionId());
            colors.add(new Color((int) (255.0 * color.getRed()), (int) (255.0 * color.getGreen()), (int) (255.0 * color.getBlue()), (int) (255.0 *color.getOpacity())));
         }
      }

      int numberOfPoints = pointCloud.size();
      long timestamp = (long) (Conversions.nanosecondsToSeconds(System.nanoTime()) - startTime);
      float[] pointCloudBuffer = new float[3 * numberOfPoints];
      int[] colorsInteger = new int[numberOfPoints];

      for (int i = 0; i < numberOfPoints; i++)
      {
         Point3DReadOnly scanPoint = pointCloud.get(i);

         pointCloudBuffer[3 * i + 0] = (float) scanPoint.getX();
         pointCloudBuffer[3 * i + 1] = (float) scanPoint.getY();
         pointCloudBuffer[3 * i + 2] = (float) scanPoint.getZ();

         colorsInteger[i] = colors.get(i).getRGB();
      }

      StereoVisionPointCloudMessage stereoVisionMessage = MessageTools.createStereoVisionPointCloudMessage(timestamp, pointCloudBuffer, colorsInteger);

      stereoVisionPublisher.publish(stereoVisionMessage);
   }
}
