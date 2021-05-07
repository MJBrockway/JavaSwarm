import java.util.*; //List, ArrayList, Random
import java.io.*;
import org.json.*; //Json support: org.json.jar needs to be in classpath
import java.nio.file.*; //Path, Files

public class SwarmModel {
  // Array accessor constants
  final int POS_X  = 0,   POS_Y  = 1,   // agents' position
            COH_X  = 2,   COH_Y  = 3,   // cpts of cohesion vectors
            REP_X  = 4,   REP_Y  = 5,   // cpts of repulsion vectors
            DIR_X  = 6,   DIR_Y  = 7,   // cpts of direction vectors
            RES_X  = 8,   RES_Y  = 9,   // cpts of of resultant vectors
            GOAL_X = 10,  GOAL_Y = 11,  // cpts of  goals
            PRM    = 12,                // agent known to be on perimeter of swarm
            GAP_X  = 13,  GAP_Y  = 14,  // components of gap reduction vector
            COH_N  = 15,  REP_N  = 16,  // num of cohesion, repulsion neighbours

            N_ROWS = 17,    // number of rows in array that models swarm state
  // Other constants          
            LINEAR = 0, QUAD = 1, EXPTL = 2; //repulsion calcuation modes

  // Swarm parameters
  double[][] rb = {{1.0,1.0},{1.0,1.0}}, //repulsion radii
             kc = {{1.0,1.0},{1.0,1.0}}, //cohesion weights
             kr = {{1.0,1.0},{1.0,1.0}}; //repulsion weights
  double  cb  = 4.0,        //master cohesion range (radius)
          ob  = 0.0,        //master obstacle aviodance range 
          kd  = 0.0,        //master  direction weight
          ko  = 0.0,        //master  obstacle aviodance weight [not implemented here]
          kg  = 0.0,        // .aster gap reduction wt, default 0.0
          expRt = 0.2,      //exponential rate (if rep mode == EXPTL)
          speed = 0.05,     //model time-step size
          stabFac = 0.0,    //minimum magnitude for RES to be applied
          goalX = 0.0,
          goalY = 0.0;      //goal coordinates
         
  int     repMode = LINEAR;    //default repulsion calculation mode
  boolean perimDrctd = false,  //true => only perimeter agents direct swarm twd goal
          gapFillRflx = false; //true => consider a reflex angle a gap to be filled
  final double snapRdg = Math.pow(10.0, 9); //in applyStep() snap x, y posns to 9 DP precision


  double[][] state;   //state of swarm
  int swmSz;

  double getX(int i) { return state[POS_X][i]; }
  double getY(int i) { return state[POS_Y][i]; }

  //Working structures --
  double[][]  xDiff, yDiff,   // x- and y-displacements of one agent from another
              dists, angles;  // distance, polar angle of an agent relative to another
  boolean[][] nbrs;           // nbrs[i][j] <=> i is within coh range of j
  boolean[][] repels;         // repels[i][j] <=> i is repelled by j
  int[] prm;                 // prm[i] == 1 <=> i on perimeter, ow 0    
        
  public String stateStg(int agtNo) {
    StringBuilder sb = new StringBuilder();
    for (int r = 0; r < N_ROWS; r++)
      sb.append(String.format("%.10f  ", state[r][agtNo]));
    return sb.toString();
  }

  private void setParams(Map<String, String> params) {
    for (String k: params.keySet()) {
      if (k.equals("rb")) getDoubleArray(params.get(k), rb);
      if (k.equals("kc")) getDoubleArray(params.get(k), kc);
      if (k.equals("kr")) getDoubleArray(params.get(k), kr);
      if (k.equals("cb"))  cb = Double.parseDouble(params.get(k));
      if (k.equals("ob"))  ob = Double.parseDouble(params.get(k));
      if (k.equals("kd"))  kd = Double.parseDouble(params.get(k));
      if (k.equals("ko"))  ko = Double.parseDouble(params.get(k));
      if (k.equals("kg"))  kg = Double.parseDouble(params.get(k));
      if (k.equals("scaling")) {
        if (params.get(k).substring(0,4).equals("expo"))
          repMode = EXPTL;
        else if (params.get(k).substring(0,4).equals("quad"))
          repMode = QUAD;
        //else LINEAR by default
      }
      if (k.equals("exp_rate")) expRt = Double.parseDouble(params.get(k));
      if (k.equals("speed"))    speed   = Double.parseDouble(params.get(k));
      if (k.length() >= 4 && k.substring(0,4).equals("stab"))
        stabFac = Double.parseDouble(params.get(k));
      if (k.equals("perim_coord"))
        perimDrctd = Boolean.parseBoolean(params.get(k));
      if (k.equals("rgf"))
         gapFillRflx = Boolean.parseBoolean(params.get(k));
      if (k.toLowerCase().equals("goalx")) goalX = Double.parseDouble(params.get(k));
      if (k.toLowerCase().equals("goaly")) goalY = Double.parseDouble(params.get(k));
    } //k
    System.out.printf("rb = %s\n", dispArray(rb));
    System.out.printf("kc = %s\n", dispArray(kc));
    System.out.printf("kr = %s\n", dispArray(kr));
    System.out.printf("cb = %.10f\n", cb);  
    System.out.printf("kg = %.10f, gap fill reflx = %b\n", kg, gapFillRflx);  
    System.out.printf("goal = (%.10f,%.10f), kd = %.10f\n", goalX, goalY, kd);  
    System.out.printf("ob = %.10f, ko = %.10f\n", ob, ko);  
    System.out.printf("expRt = %.10f, sclg mode = %d\n", expRt, repMode);
    System.out.printf("speed = %.10f, stb fct = %.10f\n", speed, stabFac);
    System.out.printf("perimeter directed = %b\n", perimDrctd);
  } // setParams
  
  /** Helper for setParams() - get a double[m][n] from a text string of m*n doubles */
  private void getDoubleArray(String source, double[][] target) {
    Scanner sc = new Scanner(source);
    for (int i = 0; i < target.length; i++)
      for (int j = 0; j < target[i].length; j++)
        target[i][j] = sc.nextDouble();     
  }
  
  /** Helper - build display string for a double[][] */
  public static String dispArray(double[][] xx) {
    String s = "";
    for (int i = 0; i < xx.length; i++)
      for (int j = 0; j < xx[i].length; j++)
        s += String.format("%.15f  ", xx[i][j]);
    return s;
  }

  
  /**
   Initialize state and working data structures: swarm of agents at (xs[], ys[]).
   Cohesion, repulsion, direction and resultant vector are intially zero. 
   Goal is at point (goal, goal).
   Cohesion and repulsion field radii are set to parameters cf, rf respectively. 
   Cohesion, repulsion, direction weightings are set to kc, kr, kd respectively.
   Initially, agents are presumed  NOT on perimeter.
   Arrays are created and sized for interagent displacments, distances, polar
    angles; these are updated befor use.
  */
  private void  initWorkingData(double[] xs, double[] ys) {
    assert xs.length == ys.length; //num x-coords == num y-coords!
    swmSz = xs.length;
    // swarm state ...
    state = new double[N_ROWS][swmSz];     
    prm = new int[swmSz];
    for (int i = 0; i < swmSz; i++) {
      state[POS_X][i] = xs[i];  state[POS_Y][i] = ys[i];
      state[COH_X][i]  = 0.0;   state[COH_Y][i] = 0.0; 
      state[REP_X][i]  = 0.0;   state[REP_Y][i] = 0.0; 
      state[DIR_X][i]  = 0.0;   state[DIR_Y][i] = 0.0; 
      state[RES_X][i]  = 0.0;   state[RES_Y][i] = 0.0; 
      state[GOAL_X][i] = goalX; state[GOAL_Y][i] = goalY;
      state[PRM][i] = 0;
      state[GAP_X][i]  = 0.0;   state[GAP_Y][i] = 0.0; 
      state[COH_N][i]  = 0;   state[REP_N][i] = 0;
      prm[i] = 0;
    }
    //Inter-agent data (will be updated before use): 
    xDiff = new double[swmSz][swmSz];  yDiff  = new double[swmSz][swmSz]; //displacements 
    dists = new double[swmSz][swmSz];  angles = new double[swmSz][swmSz]; //dists, angles
    nbrs  =  new boolean[swmSz][swmSz]; //[i][j] -> i,j in cohesion range
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

  /** 
   * maintain arrays of interagent displacements distances, polar angles, 
   * cohesion neighbours data (ecf, state[COH_N])
   */
  void updtWorkingData() {
    double theta;
    // Interagent displacements, distances, angles, eff coh radii, nbrs
    for (int i = 0; i < swmSz; i++) {
      xDiff[i][i]  = 0.0;   yDiff[i][i]  = 0.0;
      dists[i][i]  = 0.0;   angles[i][i]  = 0.0;
      nbrs[i][i] = false;   repels[i][i] = false;
      
      for (int j = 0; j < i; j++) {
        xDiff[i][j]  = state[POS_X][i] - state[POS_X][j];
        xDiff[j][i]  = -xDiff[i][j];
        yDiff[i][j]  = state[POS_Y][i] - state[POS_Y][j];
        yDiff[j][i]  = -yDiff[i][j];
        dists[i][j]  = Math.hypot(xDiff[i][j], yDiff[i][j]);
        dists[j][i]  = dists[i][j]; //
        theta = Math.atan2(yDiff[i][j], xDiff[i][j]);
        angles[j][i] = theta; //in [-pi,pi]. x,yDiff point bckwd: hence sw'd i,j 
        angles[i][j] = theta>0.0? theta - Math.PI: theta + Math.PI;
      } //j
    } //i
    for (int i = 0; i < swmSz; i++) {
      state[COH_N][i] = 0;
      for (int j = 0; j < swmSz; j++) {
        nbrs[i][j] = (j != i && dists[i][j] <= cb); 
        if (nbrs[i][j]) 
          state[COH_N][i]++;
        repels[i][j] = (j != i && dists[i][j] <= rb[prm[i]][prm[j]]); 
      }
    } //i

    // Perimeter status
    updateprm();
  } //updtWorkingData()


  /** Called by updtWorkingData()
   *  Update perimeter status state[PRM] of all agents in swarm.
   *  Also update gapclosing vectors state[GAP_X][], state[GAP_Y  ][]. 
   *  Assumes xDiff, yDiff, dists, angles, and COH_N are up to date.
   */
  void updateprm() {
    for (int i = 0; i < swmSz; i++) {
      prm[i] = 0;
      state[GAP_X][i] = 0.0; state[GAP_Y][i] = 0.0;
      int k = (int)state[COH_N][i];
      if (k < 3) {
        prm[i] = 1;   //under 3 nbrs => perimeter
        continue;         //next i
      }
      int[] iNbrs = new int[k];  // //coh nbrs of agent i
      k = 0;
      for (int j=0; j<swmSz; j++) {
        if (j != i && nbrs[j][i]) {
          iNbrs[k] = j; k++;
        }
      }
      
      sortNbrs(i, iNbrs);  // sort i's nbrs j by increasing angles[i][j]
      for (int j = 0; j < iNbrs.length; j++) {
        k = (j+1) % iNbrs.length;
        if (!nbrs[iNbrs[k]][iNbrs[j]]) { 
          prm[i] = 1;  //two consec nbrs out of coh range => prm[i]
          state[GAP_X][i] = kg *
            (0.5*(state[POS_X][iNbrs[k]] + state[POS_X][iNbrs[j]]) - state[POS_X][i]);
          state[GAP_Y][i] = kg *
            (0.5*(state[POS_Y][iNbrs[k]] + state[POS_Y][iNbrs[j]]) - state[POS_Y][i]);
         break; // abandon j-loop
        }       // else ...
        double delta = angles[i][iNbrs[k]] - angles[i][iNbrs[j]];
        if (delta < 0) delta += Math.PI * 2;
        if (delta > Math.PI) { //two consec nbrs make a reflex angle
          prm[i] = 1;
          if (gapFillRflx) {
            state[GAP_X][i] = kg *
              (0.5*(state[POS_X][iNbrs[k]] + state[POS_X][iNbrs[j]]) - state[POS_X][i]);
            state[GAP_Y][i] = kg *
              (0.5*(state[POS_Y][iNbrs[k]] + state[POS_Y][iNbrs[j]]) - state[POS_Y][i]);
          }
          break;
        }
      } // end for j
    } // end for i
    for (int i = 0; i < swmSz; i++)
      state[PRM][i] = (double)prm[i];
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
      state[COH_X][i] = 0.0; state[COH_Y][i] = 0.0;
      for (int j = 0; j < swmSz; j++)
        if (nbrs[j][i]) {
          state[COH_X][i] += (xDiff[j][i] * kc[prm[i]][prm[j]]);
          state[COH_Y][i] += (yDiff[j][i] * kc[prm[i]][prm[j]]);
        }
      if (state[COH_N][i] > 0) {
        state[COH_X][i] /= state[COH_N][i];
        state[COH_Y][i] /= state[COH_N][i];
      }
    }
  } //computeCOH()
  
  /** Compute COH components assuming working data is up to date
   *  LINEAR mode */
  void computeREP_lin() {
    for (int i = 0; i < swmSz; i++) {
      state[REP_N][i] = 0; state[REP_X][i] = 0.0; state[REP_Y][i] = 0.0; 
      for (int j = 0; j < swmSz; j++) {
        if (!repels[i][j]) 
          continue;
        state[REP_N][i] += 1;
        state[REP_X][i] += (1 - (rb[prm[i]][prm[j]]/dists[j][i]))*xDiff[j][i]*kr[prm[i]][prm[j]];
        state[REP_Y][i] += (1 - (rb[prm[i]][prm[j]]/dists[j][i]))*yDiff[j][i]*kr[prm[i]][prm[j]];
      } //j
      if (state[REP_N][i] >= 1) {
        state[REP_X][i] /= state[REP_N][i]; state[REP_Y][i] /= state[REP_N][i];
      }
    } //i
  } //computeREP_lin()

  /** Compute COH components assuming working data is up to date
   *  QUAD mode */
  void computeREP_quad() {
    double dd;
    for (int i = 0; i < swmSz; i++) {
      state[REP_N][i] = 0; state[REP_X][i] = 0.0; state[REP_Y][i] = 0.0; 
      for (int j = 0; j < swmSz; j++) {
        if (!repels[i][j]) 
          continue;
        state[REP_N][i] += 1;
        dd = dists[j][i];
        state[REP_X][i] -= rb[prm[i]][prm[j]]/dd/dd * xDiff[j][i]/dd * kr[prm[i]][prm[j]];
        state[REP_Y][i] -= rb[prm[i]][prm[j]]/dd/dd * yDiff[j][i]/dd * kr[prm[i]][prm[j]];
      } //j
      if (state[REP_N][i] >= 1) {
        state[REP_X][i] /= state[REP_N][i]; state[REP_Y][i] /= state[REP_N][i];
      }
    } //i
  } //computeREP_quad()

  /** Compute COH components assuming working data is up to date
   *  EXPONENTIAL mode */
  void computeREP_exp() {
    double dd;
    for (int i = 0; i < swmSz; i++) {
      state[REP_N][i] = 0; state[REP_X][i] = 0.0; state[REP_Y][i] = 0.0; 
      for (int j = 0; j < swmSz; j++) {
        if (!repels[i][j]) 
          continue;
        state[REP_N][i] += 1;
        dd = dists[j][i];
        state[REP_X][i] -= rb[prm[i]][prm[j]]*Math.exp(-dd*expRt) * xDiff[j][i]/dd * kr[prm[i]][prm[j]];
        state[REP_Y][i] -= rb[prm[i]][prm[j]]*Math.exp(-dd*expRt) * yDiff[j][i]/dd * kr[prm[i]][prm[j]];
      } //j
      if (state[REP_N][i] >= 1) {
        state[REP_X][i] /= state[REP_N][i]; state[REP_Y][i] /= state[REP_N][i];
      }
    } //i
  } //computeREP_exp()
  

  /**  Compute one step in the evolution of swarm. 
   * Param speed is the number of simulation distance units per simulation step.
   */
  void computeStep(double speed) {
    updtWorkingData(); // xDiff, yDiff, dists, angles, , nbrs, perim,
    computeCOH();      // Updates state[COH_X], state[COH_Y] including weights

    if (repMode == LINEAR)  // state[REP_X], state[REP_Y] including weights
      computeREP_lin();
    else if (repMode == QUAD)
      computeREP_quad();
    else if (repMode == EXPTL)
      computeREP_exp();
    
    for (int i = 0; i < swmSz; i++) {
      state[DIR_X][i] = kd*(state[GOAL_X][i] - state[POS_X][i]);
      state[DIR_Y][i] = kd*(state[GOAL_Y][i] - state[POS_Y][i]);

      // Resultant of the cohesion, repulsion and direction vectors:
      if (!perimDrctd || prm[i] == 1) {
        state[RES_X][i]
          = state[COH_X][i] + state[GAP_X][i] + state[REP_X][i] + state[DIR_X][i];
        state[RES_Y][i]
          = state[COH_Y][i] + state[GAP_Y][i] + state[REP_Y][i] + state[DIR_Y][i];
      } else {
        state[RES_X][i] = state[COH_X][i] + state[GAP_X][i] + state[REP_X][i];
        state[RES_Y][i] = state[COH_Y][i] + state[GAP_Y][i] + state[REP_Y][i];
      }

      double magRES = Math.hypot(state[RES_X][i], state[RES_Y][i]);
      if (magRES > stabFac * speed) {
        state[RES_X][i] *= speed/magRES;
        state[RES_Y][i] *= speed/magRES; 
      } else {
        state[RES_X][i] *= 0;
        state[RES_Y][i] *= 0; 
      }
    } //i
  } //computeStep


  void applyStep() {
    for (int i = 0; i < swmSz; i++) {
      state[POS_X][i] += state[RES_X][i];
      state[POS_Y][i] += state[RES_Y][i];
      state[POS_X][i] = Math.rint(state[POS_X][i] * snapRdg)/snapRdg;
      state[POS_Y][i] = Math.rint(state[POS_Y][i] * snapRdg)/snapRdg;
    }
  } //applyStep
  
  
  void saveState(String path) throws IOException {
    PrintWriter ptwr = new PrintWriter(new FileWriter(path));
    for (int i = 0; i < swmSz; i++) {
      for (int k = 0; k < N_ROWS; k++) 
        ptwr.printf("%.15f  ", state[k][i]);
      ptwr.println();
    }
    ptwr.close();
  }
  

  void saveSwarm(String path) throws IOException {
    PrintWriter ptwr = new PrintWriter(new FileWriter(path));
    ptwr.println(String.format("kc: %s", dispArray(kc)));
    ptwr.println(String.format("kr: %s", dispArray(kr)));
    ptwr.println(String.format("rb: %s", dispArray(rb)));
    ptwr.println(String.format("cb: %.15f", cb));
    ptwr.println(String.format("ob: %.15f", ob));
    ptwr.println(String.format("kd: %.15f", kd));
    ptwr.println(String.format("ko: %.15f", ko));
    ptwr.println(String.format("kg: %.15f", kg));
    ptwr.println(String.format("exp:_rate %.15f", expRt));
    ptwr.println(String.format("scaling: %s",
        repMode==2? "expo": (repMode==1?"quad":"linear")));
    ptwr.println(String.format("speed: %.15f", speed));
    ptwr.println(String.format("stab: %.15f", stabFac));
    ptwr.println(String.format("goalX: %.15f", goalX));
    ptwr.println(String.format("goalY: %.15f", goalY));
    ptwr.println(String.format("perim_coord: %b", perimDrctd));
    ptwr.println("# POS_X, POS_Y --");
    for (int i = 0; i < swmSz; i++)
      ptwr.println(String.format("%.15f  %.15f", state[POS_X][i], state[POS_Y][i]));
    ptwr.close();
  }

  /** Unpack model data from a config file & build model */
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
    while (line != null) {                    //swarm coords section
      tokens = line.split("\\s+");
      if (tokens.length >= 2) {
        xLst.add(tokens[0]);  yLst.add(tokens[1]);
      }
      line = fr.readLine();
    }
    fr.close();
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
    Iterator<String> itn = jprms.keys();    //Type-safety bug in org.json.jar!
                    // RHS returns an Iterator rather than an Iterator<String>
    while (itn.hasNext()) {
      String key = itn.next();
      String val = dropPunctn(jprms.getString(key));
      params.put(key, val);
    }      

    //Get params goalX, goalY from dests:coords if any are specified:
    try {
      JSONArray dsts = json.getJSONObject("destinations").getJSONArray("coords");
      if (dsts.getJSONArray(0).length() > 0 && dsts.getJSONArray(1).length() > 0) {
        params.put("goalX", dsts.getJSONArray(0).getString(0));
        params.put("goalY", dsts.getJSONArray(1).getString(0));
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

