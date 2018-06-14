package us.ihmc.manipulation.planning.gradientDescent;

public interface SingleQueryFunction
{
   public abstract double getQuery(double... values);
}
