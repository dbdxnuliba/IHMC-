package us.ihmc.ihmcPerception.depthData;

import java.net.URISyntaxException;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Color3f;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.ros.apps.RosPointCloudFilterRepublisher;

/*
 * See @RosPointCloudFilterRepublisher
 */
public class DepthDataFilterRosDemo extends RosPointCloudFilterRepublisher
{
   private DepthDataFilter depthDataFilter;
   private RigidBodyTransform sensorTransform = new RigidBodyTransform(new AxisAngle4d(), new Vector3d(0.0, 0.0, 1.0));


   public DepthDataFilterRosDemo()
   {
      ReferenceFrame headFrame = ReferenceFrame.getWorldFrame();
      this.depthDataFilter = new DepthDataFilter(headFrame);

   }


   @Override
   protected boolean includePoint(Point3d point, Color3f color)
   {
      return includePoint(point, (color.x + color.y + color.z) / 3.0f);
   }

   @Override
   protected boolean includePoint(Point3d point, float intensity)
   {
      return depthDataFilter.addPoint(point, sensorTransform);
   }


   public static void main(String[] arg) throws URISyntaxException
   {
      DepthDataFilterRosDemo republisher = new DepthDataFilterRosDemo();
      new Thread(republisher).start();
   }
}
