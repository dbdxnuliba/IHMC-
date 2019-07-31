package us.ihmc.robotEnvironmentAwareness.fusion.tools;

import boofcv.struct.calib.IntrinsicParameters;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public class PointCloudProjectionHelper
{
   private static final int defaultOffsetU = 11;
   private static final int defaultOffsetV = 0;

   public static final IntrinsicParameters multisenseOnCartIntrinsicParameters = new IntrinsicParameters();
   static
   {
      // cart
          multisenseOnCartIntrinsicParameters.setFx(566.8350830078125);
          multisenseOnCartIntrinsicParameters.setFy(566.8350830078125);
          multisenseOnCartIntrinsicParameters.setCx(505.5);
          multisenseOnCartIntrinsicParameters.setCy(260.5);
      // atlas
//      multisenseOnCartIntrinsicParameters.setFx(555.999267578125);
//      multisenseOnCartIntrinsicParameters.setFy(555.999267578125);
//      multisenseOnCartIntrinsicParameters.setCx(512.0);
//      multisenseOnCartIntrinsicParameters.setCy(269.5);
   }

   /**
    * This method is to pack a pixel value (u, v) that is a point projected onto image.
    * The author recommends to use parameter set of 'P' which placed in `CameraInfo` of Multisense.
    */
   public static void projectMultisensePointCloudOnImage(Point3DReadOnly pointToProject, Point2DBasics pixelToPack, IntrinsicParameters param)
   {
      int[] pixelArray = new int[2];
      pixelArray = projectMultisensePointCloudOnImage(pointToProject, param);

      pixelToPack.set(pixelArray[0], pixelArray[1]);
   }

   public static void projectMultisensePointCloudOnImage(Point3DReadOnly pointToProject, Point2DBasics pixel)
   {
      projectMultisensePointCloudOnImage(pointToProject, pixel, multisenseOnCartIntrinsicParameters, defaultOffsetU, defaultOffsetV);
   }

   public static void projectMultisensePointCloudOnImage(Point3DReadOnly pointToProject, Point2DBasics pixelToPack, int offsetU, int offsetV)
   {
      projectMultisensePointCloudOnImage(pointToProject, pixelToPack, multisenseOnCartIntrinsicParameters, offsetU, offsetV);
   }

   public static void projectMultisensePointCloudOnImage(Point3DReadOnly pointToProject, Point2DBasics pixelToPack, IntrinsicParameters param, int offsetU, int offsetV)
   {
      projectMultisensePointCloudOnImage(pointToProject, pixelToPack, param);
      pixelToPack.add(offsetU, offsetV);
   }

   /**
    * Point cloud projection from camera transform which is not same with world frame.
    */
   public static int[] projectMultisensePointCloudOnImage(Point3DReadOnly pointToProject, IntrinsicParameters param, Point3DReadOnly cameraPosition,
                                                          QuaternionReadOnly cameraOrientation)
   {
      Point3D pointToCamera = new Point3D(pointToProject);
      RigidBodyTransform transformWorldToCamera = new RigidBodyTransform(cameraOrientation, cameraPosition);
      pointToCamera.applyInverseTransform(transformWorldToCamera);
      return projectMultisensePointCloudOnImage(pointToCamera, param);
   }

   public static int[] projectMultisensePointCloudOnImage(Point3DReadOnly point, IntrinsicParameters param)
   {
      double fx = param.getFx();
      double fy = param.getFy();
      double cx = param.getCx();
      double cy = param.getCy();

      double cameraX = -point.getY();
      double cameraY = -point.getZ();
      double cameraZ = point.getX();

      double normX = cameraX / cameraZ;
      double normY = cameraY / cameraZ;

      int u = (int) (fx * normX + cx);
      int v = (int) (fy * normY + cy);

      int[] pixel = new int[2];
      pixel[0] = u;
      pixel[1] = v;

      return pixel;
   }
}