package us.ihmc.robotEnvironmentAwareness.fusion.parameters;

public class StereoREAParallelParameters
{
   public static final boolean addPointsToRawPCAInParallel = false;
   public static final boolean addPointsToPCAWhenExtendingInParallel = true;
   public static final boolean updateRawSuperPixelNormalsInParallel = true;

   public static final boolean useParallelStreamsForFiltering = true;

   public static final boolean updateSparsityInParallel = true;
}
