package us.ihmc.robotEnvironmentAwareness.fusion.objectDetection;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ThreadLocalRandom;

import controller_msgs.msg.dds.DoorParameterPacket;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.ros2.Ros2Node;
import java.lang.Math; 

public class DoorParameterCalculator extends AbstractObjectParameterCalculator<DoorParameterPacket>
{
   public List<Point3DBasics> pointCloudFitted;
   public int size = pointCloudToCalculate.size();
   public double epsilon = .05; // point cloud within this distance from the plane will be collected
   public DoorParameterCalculator(Ros2Node ros2Node, Class<DoorParameterPacket> packetType)
   {
      super(ros2Node, packetType);
   }

   @Override
   public void calculateAndPackResult()
   {
      pointCloudFitted = new ArrayList<Point3DBasics>();
      double[] equationOfBestPlane = getBestPlane();
      // TODO : pack result on newPacket.
      double xtotal = 0;
      double ytotal = 0;
      double ztotal = 0;
      System.out.println("freeze");
      for (int i = 0; i < size; i++)
      {
         if (getPointPlaneDistance(equationOfBestPlane, pointCloudToCalculate.get(i).getX(), pointCloudToCalculate.get(i).getY(), pointCloudToCalculate.get(i).getZ()) < epsilon) {
            pointCloudFitted.add(pointCloudToCalculate.get(i));
            xtotal += pointCloudToCalculate.get(i).getX();
            ytotal += pointCloudToCalculate.get(i).getY();
            ztotal += pointCloudToCalculate.get(i).getZ();
         }
      }
      int newSize = pointCloudFitted.size();
      System.out.println("freeze "+newSize);
      
      System.out.println("door center " +xtotal/newSize +" "+ ytotal/newSize +" "+ztotal/newSize);
   }
      
   public double[] getBestPlane()
   {
      double xtotal = 0 , ytotal = 0, ztotal = 0;
      double[] xRandomPoints = new double[3];
      double[] yRandomPoints = new double[3];
      double[] zRandomPoints = new double[3];
      int preInliers = 0;
      int curInliers = 0;
      double[] bestPlane = new double[4];
      
//      w = .9 - estimated # of outliers / total number of cloud points      
//      n = 3 - min # of points needed to for the model, 
//      p = .99 - probability that at least one of the sets of random samples does not include an outlier
//      k = log(1-p)/log(1-w**n) ~ 4 - minimum # of iterations required to get desired accuracy (p)
      
      for (int k = 0; k < 15; k++)
      {
         for (int n = 0; n < 3; k++)
         {
            int randomNum = ThreadLocalRandom.current().nextInt(0, size + 1);
            xRandomPoints[n] = pointCloudToCalculate.get(randomNum).getX();
            yRandomPoints[n] = pointCloudToCalculate.get(randomNum).getY();
            zRandomPoints[n] = pointCloudToCalculate.get(randomNum).getZ();
         }
         double[] equationOfPlane = getEquationOfPlane(xRandomPoints[0], yRandomPoints[0], zRandomPoints[0], xRandomPoints[1], yRandomPoints[1], zRandomPoints[1],xRandomPoints[2], yRandomPoints[2], zRandomPoints[2] );
         for (int j = 0; j < size; j++)
         {
            if (getPointPlaneDistance(equationOfPlane, pointCloudToCalculate.get(j).getX(), pointCloudToCalculate.get(j).getY(), pointCloudToCalculate.get(j).getZ()) < epsilon)
            {
               curInliers++;
            }
         }
         if (curInliers > preInliers)
         {
            preInliers = curInliers;
            bestPlane = equationOfPlane;
         }
      }
      return bestPlane;
   }
   
   public double[] getEquationOfPlane(double x1, double y1, double z1, double x2, double y2, double z2, double x3, double y3, double z3)
   {
      double[] v1 = {x3 - x1, y3 - y1, z3 - z1};
      double[] v2 = {x2 - x1, y2 - y1, z2 - z1};
      double[] cross_v1_v2 = {v1[1] * v2[2] - v1[2] * v2[1],
                     v1[2] * v2[0] - v1[0] * v2[2],
                     v1[0] * v2[1] - v1[1] * v2[0]};
      double a = cross_v1_v2[0];
      double b = cross_v1_v2[1];
      double c = cross_v1_v2[2];
      double d = a*x1 + b*y1 + c*z1;
      double[] result = {a, b, c, d};
      return result; // ax + by + cz = d
   }
   
   public double getPointPlaneDistance(double[] equationOfPlane, double x, double y, double z)
   {
      return Math.abs(equationOfPlane[0]*x + equationOfPlane[1]*y + equationOfPlane[2]*z - equationOfPlane[3])/Math.sqrt(Math.pow(equationOfPlane[0], 2)+Math.pow(equationOfPlane[1], 2)+Math.pow(equationOfPlane[2], 2));
   }
}