package us.ihmc.robotics.math.corruptors;

import static us.ihmc.robotics.math.frames.YoFrameVariableNameTools.createXName;
import static us.ihmc.robotics.math.frames.YoFrameVariableNameTools.createYName;
import static us.ihmc.robotics.math.frames.YoFrameVariableNameTools.createZName;

import java.util.Random;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.math.filters.ProcessingYoVariable;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFrameTuple3D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

public class GaussianCorruptorYoFrameVector3D extends YoFrameVector3D implements ProcessingYoVariable
{
   private final GaussianCorruptorYoDouble x, y, z;

   private GaussianCorruptorYoFrameVector3D(GaussianCorruptorYoDouble x, GaussianCorruptorYoDouble y, GaussianCorruptorYoDouble z,
                                            ReferenceFrame referenceFrame)
   {
      super(x, y, z, referenceFrame);

      this.x = x;
      this.y = y;
      this.z = z;
   }

   public static GaussianCorruptorYoFrameVector3D createGaussianCorruptorYoFrameVector3D(String namePrefix, String nameSuffix, YoVariableRegistry registry,
                                                                                         Random random, DoubleProvider standardDeviation,
                                                                                         ReferenceFrame referenceFrame)
   {
      GaussianCorruptorYoDouble x = new GaussianCorruptorYoDouble(createXName(namePrefix, nameSuffix), registry, random, standardDeviation);
      GaussianCorruptorYoDouble y = new GaussianCorruptorYoDouble(createYName(namePrefix, nameSuffix), registry, random, standardDeviation);
      GaussianCorruptorYoDouble z = new GaussianCorruptorYoDouble(createZName(namePrefix, nameSuffix), registry, random, standardDeviation);

      return new GaussianCorruptorYoFrameVector3D(x, y, z, referenceFrame);
   }

   public static GaussianCorruptorYoFrameVector3D createGaussianCorruptorYoFrameVector3D(String namePrefix, String nameSuffix, YoVariableRegistry registry,
                                                                                         Random random, DoubleProvider standardDeviation, YoFrameTuple3D input)
   {
      GaussianCorruptorYoDouble x, y, z;
      x = new GaussianCorruptorYoDouble(createXName(namePrefix, nameSuffix), registry, random, standardDeviation, input.getYoX());
      y = new GaussianCorruptorYoDouble(createYName(namePrefix, nameSuffix), registry, random, standardDeviation, input.getYoY());
      z = new GaussianCorruptorYoDouble(createZName(namePrefix, nameSuffix), registry, random, standardDeviation, input.getYoZ());

      return new GaussianCorruptorYoFrameVector3D(x, y, z, input.getReferenceFrame());
   }

   public static GaussianCorruptorYoFrameVector3D createGaussianCorruptorYoFrameVector3D(String namePrefix, String nameSuffix, YoVariableRegistry registry,
                                                                                         Random random, Vector3DReadOnly standardDeviation,
                                                                                         ReferenceFrame referenceFrame)
   {
      GaussianCorruptorYoDouble x = new GaussianCorruptorYoDouble(createXName(namePrefix, nameSuffix), registry, random, () -> standardDeviation.getX());
      GaussianCorruptorYoDouble y = new GaussianCorruptorYoDouble(createYName(namePrefix, nameSuffix), registry, random, () -> standardDeviation.getY());
      GaussianCorruptorYoDouble z = new GaussianCorruptorYoDouble(createZName(namePrefix, nameSuffix), registry, random, () -> standardDeviation.getZ());

      return new GaussianCorruptorYoFrameVector3D(x, y, z, referenceFrame);
   }

   public static GaussianCorruptorYoFrameVector3D createGaussianCorruptorYoFrameVector3D(String namePrefix, String nameSuffix, YoVariableRegistry registry,
                                                                                         Random random, Vector3DReadOnly standardDeviation, YoFrameTuple3D input)
   {
      GaussianCorruptorYoDouble x, y, z;
      x = new GaussianCorruptorYoDouble(createXName(namePrefix, nameSuffix), registry, random, () -> standardDeviation.getX(), input.getYoX());
      y = new GaussianCorruptorYoDouble(createYName(namePrefix, nameSuffix), registry, random, () -> standardDeviation.getY(), input.getYoY());
      z = new GaussianCorruptorYoDouble(createZName(namePrefix, nameSuffix), registry, random, () -> standardDeviation.getZ(), input.getYoZ());

      return new GaussianCorruptorYoFrameVector3D(x, y, z, input.getReferenceFrame());
   }

   @Override
   public void update()
   {
      x.update();
      y.update();
      z.update();
   }

   public void update(double xInput, double yInput, double zInput)
   {
      x.update(xInput);
      y.update(yInput);
      z.update(zInput);
   }

   public void update(Tuple3DReadOnly input)
   {
      x.update(input.getX());
      y.update(input.getY());
      z.update(input.getZ());
   }

   public void update(FrameTuple3DReadOnly inpput)
   {
      checkReferenceFrameMatch(inpput);
      x.update(inpput.getX());
      y.update(inpput.getY());
      z.update(inpput.getZ());
   }
}
