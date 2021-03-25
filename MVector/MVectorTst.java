public class MVectorTst {
  public static void main(String[] args) {
    double ax = Double.parseDouble(args[0]),
           ay = Double.parseDouble(args[1]),
           bx = Double.parseDouble(args[2]),
           by = Double.parseDouble(args[3]);
           
    MVector a = new MVector(ax, ay), b = new MVector(bx, by);
    System.out.printf("a = %s, b= %s\n", a, b);
    System.out.printf("|a| = %.10f, a.ang = %.10f (%.10f deg)\n",
                  a.mag(), a.ang(), a.ang()/Math.PI*180);
    System.out.printf("|b| = %.10f, b.arg = %.10f (%.10f deg)\n",
                  b.mag(), b.ang(), b.ang()/Math.PI*180);

    MVector ma = MVector.neg(a), mb = MVector.neg(b);
    System.out.printf("|-a| = %.10f, (-a).ang = %.10f (%.10f deg)\n",
                  ma.mag(), ma.ang(), ma.ang()/Math.PI*180);
    System.out.printf("|-b| = %.10f, (-b).arg = %.10f (%.10f deg)\n",
                  mb.mag(), mb.ang(), mb.ang()/Math.PI*180);

    System.out.println();
    System.out.printf("3.5 b = %s\n", MVector.scMul(3.5, b));
    System.out.printf("a + b = %s\n", MVector.plus(a, b));
    System.out.printf("a - b = %s\n", MVector.minus(a, b));
    
    System.out.println();
    System.out.printf("dist(a,b) = %s\n", MVector.dist(a, b));
    double theta = MVector.ang(a, b);
    System.out.printf("ang(a, b) = %.10f(%.10f deg)\n", theta, theta/Math.PI*180);
    theta = MVector.ang(b, a);
    System.out.printf("ang(b, a) = %.10f(%.10f deg)\n", theta, theta/Math.PI*180);
    theta = MVector.ang(ma, mb);
    System.out.printf("ang(-a, -b) = %.10f(%.10f deg)\n", theta, theta/Math.PI*180);
    theta = MVector.ang(b, ma);
    System.out.printf("ang(b, -a) = %.10f(%.10f deg)\n", theta, theta/Math.PI*180);
  }
}
