public class MVector {
  double x, y;
  
  public MVector(double xx, double yy) { //Constructor
    x = xx;
    y = yy;
  }

  //For display purposes  
  public String toString() {
    return String.format("(%.10f, %.10f)", x, y);
  }
  
  //Magnitude of a vector
  public double mag() {
    return Math.hypot(x, y);
  }

  //Direction of vector reckoned anticlockwise from +x direction
  public double ang() {
    return Math.atan2(y, x);
  }

  // Static vector algebra methods ...
  
  public static final MVector ZERO = new MVector(0.0, 0.0);
  
  //Additive inverse of a vector
  public static MVector neg(MVector a) {
    return MVector.minus(ZERO, a);
  }
  
  //Add two vectors
  public static MVector plus(MVector a, MVector b) {
    return new MVector(a.x + b.x, a.y + b.y);
  }
  
  //Subsrtract two vectors: also the displacement from b to a (ba with over-arrow)
  public static MVector minus(MVector a, MVector b) {
    return new MVector(a.x - b.x, a.y - b.y);
  }
  
  //scalar-multiply a vector
  public static MVector scMul(double m, MVector a) {
    return new MVector(m * a.x, m * a.y);
  }
  
  //Distance beween points (tips of vecotrs)
  //-same as MVector.minus(a,b).mag() but without construction so faster
  public static double dist(MVector a, MVector b) {
    return Math.hypot(a.x - b.x, a.y - b.y);
  }
  
  //Polar angle of a relative to b
  //-same as MVector.minus(a,b).ang() but without construction so faster
  public static double ang(MVector a, MVector b) {
    return Math.atan2(a.y - b.y, a.x - b.x);
  }

}
