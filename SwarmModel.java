import java.util.*; //List, ArrayList, Random
import java.io.*;
import org.json.*; //Json support: org.json.jar needs to be in classpath
import java.nio.file.*; //Path, Files

public class SwarmModel {

  /* Swarm state  ********************************************************/
  int      swmSz;        //swarm size: all above arrays have this length
  double[] posX,  posY,  // agents' position
           cohX,  cohY,  // cpts of cohesion vectors
           repX,  repY,  // cpts of repulsion vectors
           dirX,  dirY,  // cpts of direction vectors
           advX,  advY,  // cpts of adversarial vectors
           gapX,  gapY,  // components of gap reduction vector
           resX,  resY;  // cpts of of resultant vectors

  int[]    prm,          // 0 or 1; 1 => agent known to be on perimeter of swarm
           cohN,  repN;  // num of cohesion, repulsion neighbours

  double getX(int i) { return posX[i]; }
  double getY(int i) { return posY[i]; }

  /* Other constants  ****************************************************/
  int      LINEAR = 0, QUAD = 1, EXPTL = 2; //repulsion calculation modes
  final double snapRdg = Math.pow(10.0, 9); //in applyStep() snap x, y posns to 9 DP precision


  /* Swarm parameters ****************************************************/
  double[][] rb = {{2.0,2.0},{2.0,2.0}},      //repulsion radii
             kc = {{0.15,0.15},{0.15,0.15}},  //cohesion weights
             kr = {{50.0,50.0},{50.0,50.0}};  //repulsion weights
  double     cb  = 3.0,                       //cohesion range (radius)
             goalX = 0.0, goalY = 0.0;
  double[]   kd  = {0.0,0.0},                 //direction weight
             ka  = {0.0,0.0},                 //adversarial weight 
             ra  = {Math.PI/2,Math.PI/2};     //adversarial angles 
  double     kg  = 0.0,           //gap reduction wt, default 0.0
             expRt = 0.2,         //exponential rate (if rep mode == EXPTL)
             speed = 0.05,        //model time-step size
             stabFac = 0.0,       //minimum magnitude for RES to be applied
             gain = Double.NaN;   //for linear (not normalised) model
         
  int     repMode = LINEAR;      //default repulsion calculation mode

  boolean   gapFillRflx = false; //true => (reflex angle => gap to be filled)



  /* Working data structures ***********************************************/
  double[][]  xDiff, yDiff,   // x- and y-displacements of one agent from another
              dists, angles;  // distance, polar angle of an agent relative to another
  boolean[][] nbrs;           // nbrs[i][j] <=> i is within coh range of j
  boolean[][] repels;         // repels[i][j] <=> i is repelled by j

  /* initialisation ********************************************************/
  private void setParams(Map<String, String> params) {
    for (String ks: params.keySet()) {
      if (ks.equals("rb"))  get2DArray(params.get(ks), rb);
      if (ks.equals("kc"))  get2DArray(params.get(ks), kc);
      if (ks.equals("kr"))  get2DArray(params.get(ks), kr);
      if (ks.equals("cb"))  cb = Double.parseDouble(params.get(ks));
      if (ks.equals("kd"))  get1DArray(params.get(ks), kd); 
      if (ks.equals("ka"))  get1DArray(params.get(ks), ka); 
      if (ks.equals("ra"))  get1DArray(params.get(ks), ra); 
      if (ks.equals("kg"))  kg = Double.parseDouble(params.get(ks));
      if (ks.equals("scaling")) {
        if (params.get(ks).substring(0,4).equals("expo"))
          repMode = EXPTL;
        else if (params.get(ks).substring(0,4).equals("quad"))
          repMode = QUAD;
        //else LINEAR by default
      }
      if (ks.equals("exp_rate")) expRt  = Double.parseDouble(params.get(ks));
      if (ks.equals("speed"))    speed  = Double.parseDouble(params.get(ks));
      if (ks.length() >= 4 && ks.substring(0,4).equals("stab"))
        stabFac = Double.parseDouble(params.get(ks));
      if (ks.equals("rgf"))
        gapFillRflx = Boolean.parseBoolean(params.get(ks));
      if (ks.equals("gain")) {
        try {
          gain = Double.parseDouble(params.get(ks));
        } catch (NumberFormatException ex) {
          gain = Double.NaN;
        }
      }
      if (ks.toLowerCase().equals("goal")) {
        double[] goal = new double[2];
        get1DArray(params.get(ks), goal); 
        goalX = goal[0]; goalY = goal[1];
      }
    } //ks
    System.out.printf("rb = %s\n", dispArray(rb, true)); // with delimters.
    System.out.printf("kc = %s\n", dispArray(kc, true)); //  See persistence sec below
    System.out.printf("kr = %s\n", dispArray(kr, true));
    System.out.printf("cb = %.10f\n", cb);  
    System.out.printf("kd = %s, goal = (%.10f,%.10f)\n", dispArray(kd, true), goalX, goalY);  
    System.out.printf("ka = %s\n", dispArray(ka, true));  
    System.out.printf("ra = %s\n", dispArray(ra, true));  
    System.out.printf("kg = %.10f, gap fill reflx = %b\n", kg, gapFillRflx);  
    System.out.printf("rep sclg mode = %d, expRt = %.10f\n", repMode, expRt);
    System.out.printf("speed = %.10f, stb fct = %.10f\n", speed, stabFac);
    System.out.printf("gain = %.10f\n", gain);
    System.out.printf("goal = %.10f, %.10f\n", goalX, goalY);
  } // setParams
  
  /** Helper for setParams() - get a double[n] from a text string of n doubles */
  private void get1DArray(String source, double[] target) {
    Scanner sc = new Scanner(source);
    for (int i = 0; i < target.length; i++)
      target[i] = sc.nextDouble();     
  }
  
  /** Helper for setParams() - get a double[m][n] from a text string of m*n doubles */
  private void get2DArray(String source, double[][] target) {
    Scanner sc = new Scanner(source);
    for (int i = 0; i < target.length; i++)
      for (int j = 0; j < target[i].length; j++)
        target[i][j] = sc.nextDouble();
  }
  
  /**
   Initialize state and working data structures: swarm of agents at (xs[], ys[]).
   Cohesion, repulsion, gap, direction, adversarial and resultant vector are intially zero. 
   Initially, agents are presumed  NOT on perimeter.
   Arrays are created and sized for interagent displacments, distances, polar
    angles; these updated (another function) before use.
  */
  private void  initWorkingData(double[] xs, double[] ys) {
    assert xs.length == ys.length; //num x-coords == num y-coords!
    swmSz = xs.length;
    
    // swarm state ...
    posX = new double[swmSz];  posY = new double[swmSz]; 
    cohX = new double[swmSz];  cohY = new double[swmSz]; 
    repX = new double[swmSz];  repY = new double[swmSz]; 
    dirX = new double[swmSz];  dirY = new double[swmSz]; 
    advX = new double[swmSz];  advY = new double[swmSz]; 
    gapX = new double[swmSz];  gapY = new double[swmSz]; 
    resX = new double[swmSz];  resY = new double[swmSz]; 

    prm = new int[swmSz];
    cohN = new int[swmSz];     repN = new int[swmSz];

    for (int i = 0; i < swmSz; i++) {
      posX[i] = xs[i];    posY[i] = ys[i];
      cohX[i] = 0.0;      cohY[i] = 0.0; 
      repX[i] = 0.0;      repY[i] = 0.0; 
      dirX[i] = 0.0;      dirY[i] = 0.0; 
      advX[i] = 0.0;      advY[i] = 0.0; 
      gapX[i] = 0.0;      gapY[i] = 0.0; 
      resX[i] = 0.0;      resY[i] = 0.0; 

      prm[i] = 0;   cohN[i]  = 0;   repN[i] = 0;
    }
    //Inter-agent data (will be updated before use):
    xDiff = new double[swmSz][swmSz];  yDiff  = new double[swmSz][swmSz]; //displacements 
    dists = new double[swmSz][swmSz];  angles = new double[swmSz][swmSz]; //dists, angles
    nbrs  =  new boolean[swmSz][swmSz]; //[i][j] -> i attracted by j (cohesion)
    repels = new boolean[swmSz][swmSz]; //[i][j] -> i is repelled by j
  } //initWorkingData
  
  /** Constructor for swarm at xs[], ys[] ... */
  public SwarmModel(double[] xs, double[] ys, Map<String, String> prms) {
    setParams(prms);    // All parameters apart from state[][] now presumed initialised
    initWorkingData(xs, ys);
  } //constructor

  /**
   Construct swarm of (swmSz) agents in random positions in rectangle
     (-grd+loc, -grd+loc)--(grd+loc, grd+loc). Agent 0 is at (loc, loc).
  */
  public SwarmModel(int swmSz, double grd, double loc, Map<String, String> prms) {
    this(System.nanoTime(), swmSz, grd, loc, prms);
  }
  
  public SwarmModel(long seed, int sz, double grd, double loc,
                                       Map<String, String> prms) {
    setParams(prms);    // All params apart from state[][] presumed initialised
    swmSz = sz;
    double[] xs = new double[swmSz], ys = new double[swmSz];
    Random prng = new Random(seed);
    xs[0] = loc;  ys[0] = loc; 
    for (int i = 1; i < swmSz; i++) {
      xs[i] = (prng.nextDouble()*2 - 1)*grd + loc;
      ys[i] = (prng.nextDouble()*2 - 1)*grd + loc;
    }
    initWorkingData(xs, ys);
  } //constructor


  /* Update model state *******************************************************/

  /** maintain arrays of interagent displacements distances, polar angles, coh, rep data */
  void updtWorkingData() {
    double theta;
    // Interagent displacements, distances, angles, eff coh radii, nbrs
    for (int i = 0; i < swmSz; i++) {
      xDiff[i][i]  = 0.0;   yDiff[i][i]  = 0.0;
      dists[i][i]  = 0.0;   angles[i][i]  = 0.0;
      nbrs[i][i] = false;   repels[i][i] = false;
      
      for (int j = 0; j < i; j++) {
        xDiff[i][j]  = posX[i] - posX[j];
        xDiff[j][i]  = -xDiff[i][j];
        yDiff[i][j]  = posY[i] - posY[j];
        yDiff[j][i]  = -yDiff[i][j];
        dists[i][j]  = Math.hypot(xDiff[i][j], yDiff[i][j]);
        dists[j][i]  = dists[i][j]; //
        theta = Math.atan2(yDiff[i][j], xDiff[i][j]);
        angles[j][i] = theta; //in [-pi,pi]. x,yDiff point bckwd: hence sw'd i,j 
        angles[i][j] = theta>0.0? theta - Math.PI: theta + Math.PI;
      } //j
    } //i
    for (int i = 0; i < swmSz; i++) {
      cohN[i] = 0;
      for (int j = 0; j < swmSz; j++) {
        if (j == i) continue;
        nbrs[i][j] = (dists[i][j] <= cb);
        if (nbrs[i][j])  cohN[i]++;
      }
    } //i

    // Perimeter status and repellors
    updateprm();
    for (int i = 0; i < swmSz; i++) {
      repels[i][i] = false;
      for (int j = 0; j < swmSz; j++) {
        if (j == i) continue;
        repels[i][j] = (dists[i][j] <= rb[prm[i]][prm[j]]); 
      }
    }
  } //updtWorkingData()


  /** Called by updtWorkingData()
   *  Update perimeter status prm[] of all agents in swarm.
   *  Also update gap closing vectors gapX[], gapY[]. 
   *  Assumes xDiff, yDiff, dists, angles, and cohN[] are up to date. */
  void updateprm() {
    for (int i = 0; i < swmSz; i++) {
      prm[i] = 0;
      gapX[i] = 0.0; gapY[i] = 0.0; 
      if (cohN[i] < 3) {
        prm[i] = 1;   //under 3 nbrs => perimeter
        continue;         //next i
      }
      int[] iNbrs = new int[cohN[i]];  //coh nbrs of agent i
      int k = 0;
      for (int j=0; j<swmSz; j++) {
        if (j != i && nbrs[j][i]) {
          iNbrs[k] = j;
          k++;
        }
      }
      
      sortNbrs(i, iNbrs);  // sort i's nbrs j by increasing angles[i][j]
      for (int j = 0; j < iNbrs.length; j++) {
        k = (j+1) % iNbrs.length;
        if (!nbrs[iNbrs[k]][iNbrs[j]]) { 
          prm[i] = 1;  //two consec nbrs out of coh range => prm[i]
          gapX[i] += kg * (0.5*(posX[iNbrs[k]] + posX[iNbrs[j]]) - posX[i]);
          gapY[i] += kg * (0.5*(posY[iNbrs[k]] + posY[iNbrs[j]]) - posY[i]);
         break; // abandon j-loop
        }
        // else ...
        double delta = angles[i][iNbrs[k]] - angles[i][iNbrs[j]];
        if (delta < 0) delta += Math.PI * 2;
        if (delta > Math.PI) { //two consec nbrs make a reflex angle
          prm[i] = 1;
          if (gapFillRflx) {
            gapX[i] += kg * (0.5*(posX[iNbrs[k]] + posX[iNbrs[j]]) - posX[i]);
            gapY[i] += kg * (0.5*(posY[iNbrs[k]] + posY[iNbrs[j]]) - posY[i]);
          }
          break;
        }
      } // end for j
    } // end for i
  } // updateprm()

  /** Helper: sort neighbours of agent i 
   * i and the array are presumed to be indexes of agents. 
   * Array a[j] is sorted into increasing order by angles[i][a[j]] */
  private void sortNbrs(int i, int[] a ) {
		int jmin;
		for (int j = 0; j < a.length; j++) { //selection sort algorithm
			jmin = j;
			for (int k = j; k < a.length; k++) {
				if (angles[i][a[k]] < angles[i][a[jmin]])
					jmin = k;
			} 
			int tmp = a[j]; a[j] = a[jmin]; a[jmin] = tmp;
		}
  } //sort()
  
  
  /** Compute COH components assuming working data is up to date */
  void computeCOH() {
    for (int i = 0; i < swmSz; i++) {
      cohX[i] = 0.0; cohY[i] = 0.0;
      for (int j = 0; j < swmSz; j++)
        if (nbrs[j][i]) {
          cohX[i] += (xDiff[j][i] * kc[prm[i]][prm[j]]);
          cohY[i] += (yDiff[j][i] * kc[prm[i]][prm[j]]);
        }
      if (cohN[i] > 0) {    //in David's jl, this is postponed to compute_step()
        cohX[i] /= cohN[i];  cohY[i] /= cohN[i];
      }
    }
  } //computeCOH()
  
  /** Compute COH components assuming working data inc repels[] is up to date 
   *  LINEAR mode */ 
  void computeREP_lin() {
    for (int i = 0; i < swmSz; i++) {
      repN[i] = 0; repX[i] = 0.0; repY[i] = 0.0; 
      for (int j = 0; j < swmSz; j++) {
        if (!repels[i][j])  continue;
        repN[i] += 1;
        repX[i] += (1.0 - (rb[prm[i]][prm[j]]/dists[j][i]))*xDiff[j][i]*kr[prm[i]][prm[j]];
        repY[i] += (1.0 - (rb[prm[i]][prm[j]]/dists[j][i]))*yDiff[j][i]*kr[prm[i]][prm[j]];
      } //j
      if (repN[i] >= 1) {   //in David's jl, this is postponed to compute_step()
        repX[i] /= repN[i]; repY[i] /= repN[i];
      }
    } //i
  } //computeREP_lin()

  /*  QUAD mode */
  void computeREP_quad() {
    double dd;
    for (int i = 0; i < swmSz; i++) {
      repN[i] = 0; repX[i] = 0.0; repY[i] = 0.0; 
      for (int j = 0; j < swmSz; j++) {
        if (!repels[i][j]) 
          continue;
        repN[i] += 1;
        dd = dists[j][i];
        repX[i] -= rb[prm[i]][prm[j]]/dd/dd * xDiff[j][i]/dd * kr[prm[i]][prm[j]];
        repY[i] -= rb[prm[i]][prm[j]]/dd/dd * yDiff[j][i]/dd * kr[prm[i]][prm[j]];
      } //j
      if (repN[i] >= 1) {   //in David's jl, this is postponed to compute_step()
        repX[i] /= repN[i]; repY[i] /= repN[i];
      }
    } //i
  } //computeREP_quad()

  /*  EXPONENTIAL mode */
  void computeREP_exp() {
    double dd;
    for (int i = 0; i < swmSz; i++) {
      repN[i] = 0; repX[i] = 0.0; repY[i] = 0.0; 
      for (int j = 0; j < swmSz; j++) {
        if (!repels[i][j]) 
          continue;
        repN[i] += 1;
        dd = dists[j][i];
        repX[i] -= rb[prm[i]][prm[j]]*Math.exp(-dd*expRt) * xDiff[j][i]/dd * kr[prm[i]][prm[j]];
        repY[i] -= rb[prm[i]][prm[j]]*Math.exp(-dd*expRt) * yDiff[j][i]/dd * kr[prm[i]][prm[j]];
      } //j
      if (repN[i] >= 1) {   //in David's jl, this is postponed to compute_step()
        repX[i] /= repN[i]; repY[i] /= repN[i];
      }
    } //i
  } //computeREP_exp()
  
  void computeDIR() {
    for (int i = 0; i < swmSz; i++) {
      dirX[i] = kd[prm[i]]*(goalX - posX[i]);
      dirY[i] = kd[prm[i]]*(goalY - posY[i]);
    }
  }
  
  /**
   * The DIR vector may be zero, and is always when the ka parameter is 0;
   * Normalising a zero DIR makesnosense so just set ADV to 0 in this case.
   */
  void computeADV() {
    for (int i = 0; i < swmSz; i++) {
      double magDir = Math.hypot(dirX[i], dirY[i]), nDirX, nDirY, α;
      if (magDir == 0.0) {
        advX[i] = 0.0; advY[i] = 0.0;       
      } else {
        nDirX  = dirX[i]/magDir;  nDirY  = dirY[i]/magDir;
        α = ra[prm[i]];
        advX[i] = ka[prm[i]] * (Math.cos(α)*nDirX - Math.sin(α)*nDirY);
        advY[i] = ka[prm[i]] * (Math.sin(α)*nDirX + Math.cos(α)*nDirY);
      }
    }
  }


  /**  Compute one step in the evolution of swarm.
   * Param speed is the number of simulation distance units per simulation step.
   */
  void computeStep(double speed) {
    updtWorkingData(); // xDiff, yDiff, dists, angles, , nbrs, perim, repels, cohN
    computeCOH();      // Updates cohX, cohY using weights

    if (repMode == LINEAR)  // repX, repY
      computeREP_lin();
    else if (repMode == QUAD)
      computeREP_quad();
    else if (repMode == EXPTL)
      computeREP_exp();
    
    computeDIR();
    computeADV();

    // compute resultant
    for (int i = 0; i < swmSz; i++) {
      resX[i] = cohX[i] + repX[i] + gapX[i] + dirX[i] + advX[i];
      resY[i] = cohY[i] + repY[i] + gapY[i] + dirY[i] + advY[i];

      if (Double.isNaN(gain)) { //normalise resultant
        double mag = Math.hypot(resX[i], resY[i]);
        if (mag > stabFac * speed) {
          resX[i] *= speed/mag; resY[i] *= speed/mag;
        } else {
          resX[i] *= 0;  resY[i] *= 0; 
        }
      }
      else {  //scale resultant by gain
        resX[i] *= gain; resY[i] *= gain;      
      }  
    } //i
  } //computeStep


  void applyStep() {
    for (int i = 0; i < swmSz; i++) {
      posX[i] = Math.rint((posX[i] + resX[i]) * snapRdg)/snapRdg;
      posY[i] = Math.rint((posY[i] + resY[i]) * snapRdg)/snapRdg;
    }
  } //applyStep
  
  
  /* Persistence methods *****************************************************/
  void saveState(String path) throws IOException {
    PrintWriter ptwr = new PrintWriter(new FileWriter(path));
    for (int i = 0; i < swmSz; i++)
      ptwr.println(stateStg(i)); //better without legend? Or optional?
    ptwr.close();
  }
  
  /* Helper to saveState() ***************/
  public String stateStg(int agt) {
    StringBuilder sb = new StringBuilder();
    sb.append(String.format("POS=(%.10f,%.10f); ", posX[agt], posY[agt]));
    sb.append(String.format("COH=(%.10f,%.10f); ", cohX[agt], cohY[agt]));
    sb.append(String.format("REP=(%.10f,%.10f); ", repX[agt], repY[agt]));
    sb.append(String.format("DIR=(%.10f,%.10f); ", dirX[agt], dirY[agt]));
    sb.append(String.format("ADV=(%.10f,%.10f); ", advX[agt], advY[agt]));
    sb.append(String.format("GAP=(%.10f,%.10f); ", gapX[agt], gapY[agt]));
    sb.append(String.format("RES=(%.10f,%.10f); ", resX[agt], resY[agt]));
    sb.append(String.format("%d nbrs, %s rplrs; ", cohN[agt], repN[agt]));
    sb.append(String.format("PRM=%d  ",prm[agt]));
    return sb.toString();
  }


  void saveSwarm(String path) throws IOException {
    PrintWriter ptwr = new PrintWriter(new FileWriter(path));
    ptwr.println(String.format("kc: %s", dispArray(kc, false)));
    ptwr.println(String.format("kr: %s", dispArray(kr, false)));
    ptwr.println(String.format("rb: %s", dispArray(rb, false)));
    ptwr.println(String.format("cb: %.10f", cb));
    ptwr.println(String.format("kd: %s", dispArray(kd, false)));
    ptwr.println(String.format("ka: %s", dispArray(ka, false)));
    ptwr.println(String.format("ra: %s", dispArray(ra, false)));
    ptwr.println(String.format("kg: %.10f", kg));
    ptwr.println(String.format("exp_rate: %.10f", expRt));
    ptwr.println(String.format("scaling: %s",
        repMode==2? "expo": (repMode==1?"quad":"linear")));
    ptwr.println(String.format("speed: %.10f", speed));
    ptwr.println(String.format("stab: %.10f", stabFac));
    ptwr.println(String.format("gain: %.10f", gain));
    ptwr.println(String.format("goal: %.10f %.10f", goalX, goalY));
    ptwr.println("# POS_X, POS_Y --");
    for (int i = 0; i < swmSz; i++)
      ptwr.println(String.format("%.15f  %.15f", posX[i], posY[i]));
    ptwr.close();
  }

  /** Helper - make save/display string for a double[] */
  public static String dispArray(double[] xx, boolean delim) {
    StringBuilder sb = new StringBuilder();
    if (delim) sb.append("[");
    for (int i = 0; i < xx.length; i++)
      sb.append(String.format("%.10f ", xx[i]));
    sb.deleteCharAt(sb.length()-1);
    if (delim) sb.append("]");
    return sb.toString();
  }

  /** Helper - make save/display string for a double[][] */
  public static String dispArray(double[][] xx, boolean delim) {
    StringBuilder sb = new StringBuilder();
    if (delim) sb.append("[");
    for (int i = 0; i < xx.length; i++) {
      for (int j = 0; j < xx[i].length; j++)
        sb.append(String.format("%.10f ", xx[i][j]));
      if (delim && i < xx.length - 1)
        sb.append("/ ");
    }
    if (delim) sb.append("]");
    return sb.toString();
  }
  

  /** Unpack model data from a flat config file & build model */
  public static SwarmModel loadSwarmFlat(String path)
                    throws IOException, NumberFormatException {
    Map<String, String> params = new HashMap<String, String>();
    List<String> xLst = new ArrayList<String>(),
                 yLst = new ArrayList<String>();
    BufferedReader fr = new BufferedReader(new FileReader(path));
    String[] tokens;
    String line = fr.readLine();
    while (line != null && line.charAt(0) != '#') { //params section
      tokens = line.split("\\s*:\\s*");
      params.put(tokens[0], tokens[1]);
      System.out.println(tokens[0]+" - "+tokens[1]);
      line = fr.readLine();
    }
    if (line.charAt(0) == '#')
      line = fr.readLine();
    System.out.println("parameters read");
    while (line != null) {                    //swarm coords section
      tokens = line.split("\\s+");
      if (tokens.length >= 2) {
        xLst.add(tokens[0]);  yLst.add(tokens[1]);
      }
      line = fr.readLine();
    }
    fr.close();
    System.out.println("file read\n");

    assert(xLst.size() == yLst.size());
    double[] xs = new double[xLst.size()],
             ys = new double[yLst.size()];
    for (String s: xLst) xs[xLst.indexOf(s)] = Double.parseDouble(s);
    for (String s: yLst) ys[yLst.indexOf(s)] = Double.parseDouble(s);
    
    return new SwarmModel(xs, ys, params); 
  } //end loadSwarmFlat(..)


  /** Unpack model data from a Json file & build model */
  public static SwarmModel loadSwarmJson(String path) 
           throws JSONException, IOException, NumberFormatException {
    JSONObject json = new JSONObject(Files.readString(Path.of(path)));
    
    JSONObject jprms = json.getJSONObject("params");
    Map<String, String> params = new HashMap<String, String>();
    Iterator itn = jprms.keys();    //Type-safety bug in org.json.jar!
                                    // RHS returns an Iterator rather than an Iterator<String>
    while (itn.hasNext()) {
      String key = (String)itn.next(); //.... hence this kludge
      String val = dropPunctn(jprms.getString(key));
      params.put(key, val);
    }      

    //Get params goalX, goalY from dests:coords if any are specified:
    try {
      JSONArray dsts = json.getJSONObject("destinations").getJSONArray("coords");
      if (dsts.getJSONArray(0).length() > 0 && dsts.getJSONArray(1).length() > 0) {
        params.put("goal", dsts.getJSONArray(0).getString(0) + " " + dsts.getJSONArray(1).getString(0));
        //System.out.printf("goal: %s %s\n", dsts.getJSONArray(0).getString(0), dsts.getJSONArray(1).getString(0));
      }
    } catch (JSONException ex) {
      System.out.printf(
        "%s: Missing or invalid destination spec; using defaults\n", ex);
    }
    
    JSONArray crds = json.getJSONObject("agents").getJSONArray("coords"),
              xCds = crds.getJSONArray(0), 
              yCds = crds.getJSONArray(1);
    double[] xs = new double[xCds.length()], ys = new double[yCds.length()];
    assert (xs.length == ys.length);
    for (int i=0 ; i<xs.length; i++) {
      xs[i] = xCds.getDouble(i);
      ys[i] = yCds.getDouble(i);
    }

    return new SwarmModel(xs, ys, params); 
  } //end loadSwarmJson(..)

  /** Helper for unpacking JSON: strip '[', ']' and replace ',' with space */  
  static String dropPunctn(String s) {
    StringBuilder sb = new StringBuilder(s);
    int i = 0;
    while (i < sb.length()) {
      char c = sb.charAt(i); 
      if (c == '[' || c == ']')
        sb.deleteCharAt(i);
      else {
        if (c == ',') sb.setCharAt(i, ' ');
        i++;
      }
    }
    return sb.toString();
  }

} // end class


/** NOTE on goals (destinations)
  * Goal parameter currently treated as a single 2D point, represented internally as
  * goalX, goalY loaded and saved (flat) as a double[2].
  *  Json currently puts this data in "destinations": {"coords": [ [x], [y], [z] ]}
  *  - this may change.
  *
  * There used to be goal coordinates in the agent state but we have so far always 
  * had a single goal for all agents. The model as described in the perimeter control
  * paper implicitly treats a goal in this fashion also, as does the json format.
  */

