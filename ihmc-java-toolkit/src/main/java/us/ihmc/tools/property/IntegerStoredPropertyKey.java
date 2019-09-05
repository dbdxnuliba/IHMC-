package us.ihmc.tools.property;

public class IntegerStoredPropertyKey extends StoredPropertyKey<Integer>
{
   public IntegerStoredPropertyKey(int id, String titleCasedName)
   {
      super(Integer.class, id, titleCasedName);
   }

   public IntegerStoredPropertyKey(int id, String titleCasedName, int defaultValue)
   {
      super(Integer.class, id, titleCasedName, defaultValue);
   }
}
