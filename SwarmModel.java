import java.util.*; //List, ArrayList, Random
import java.io.*;
//import static java.util.Map.entry;

public class SwarmModel {
  // Array accessor constants
  final int POS_X  = 0,   POS_Y  = 1,   // agents' position
            COH_X  = 2,   COH_Y  = 3,   // cpts of cohesion vectors
            REP_X  = 4,   REP_Y  = 5,   // cpts of repulsion vectors
            DIR_X  = 6,   DIR_Y  = 7,   // cpts of direction vectors
            RES_X  = 8,   RES_Y  = 9,   // cpts of of resultant vectors
            GOAL_X = 10,  GOAL_Y = 11,  // cpts of  goals
            CF     = 12,  RF     = 13,  // cohesion, repulsion field radii
            KC = 14, KR = 15, KD = 16,  // weights for combining coh/rep/dir vectors
            PRM    = 17,                // agent known to be on perimeter of swarm
            COH_N  = 18,  REP_N  = 19,  // num of cohesion, repulsion neighbours

            N_ROWS = 20,    // number of rows in array that models swarm state
            
            LINEAR = 0, QUAD = 1, EXPTL = 2; //repulsion calcuation modeS

  double  cb = 4.0,         //master cohesion range (radius)
          rb = 3.0,         //master repulsion range
          ob = 3.0,         //master obstacle aviodance range 
          kc = 1.0,         //master  cohesion weight
          kr = 1.0,         //master  repulsion weight
          kd = 0.0,         //master  direction weight
          ko = 0.0,         //master  obstacle aviodance weight [not implemented here]
          pc = 1.0,         //cohesion weight multiplier for perimeter agents (>= 1)
          pr = 1.0,         //repulsion weight multiplier for perimeter agents (<= 1)
          expRt = 0.2,      //exponential rate (if rep mode == EXPTL)
          speed = 0.05,     //model time-step size
          stabFac = 0.0,    //minimum magnitude for RES to be applied
          goalX = 0.0,
          goalY = 0.0;      //goal coordinates
         
  int     repMode = LINEAR; //default repulsion calculation mode
  boolean perimDrctd = false;       //true => only perimeter agents direct swarm twd goal
  final double snapRdg = Math.pow(10.0, 9); //in applyStep() snap x, y posns to 9 DP precision


  double[][] state;   //state of swarm
  int size;

  double getX(int i) { return state[POS_X][i]; }
  double getY(int i) { return state[POS_Y][i]; }

  //Working structures --
  double[][]  xDiff, yDiff,   // x- and y-displacements of one agent from another
              dists, angles,  // distance, polar angle of an agent relative to another
              ecb,            // effective cohesion range of i wrt j
              erf, ekc, ekr;  // effective repulsion range, cohesion, repulsion weights
  boolean[][] nbrs;           // nbrs[i][j] <=> i is with effective coh range of j
  boolean[] onPerim;          // onPerim[i] <=> i on perimeter    
        

  private void setParams(Map<String, String> params) {
    for (String k: params.keySet()) {
      if (k.equals("cb")) cb = Double.parseDouble(params.get(k));
      if (k.equals("rb")) rb = Double.parseDouble(params.get(k));
      if (k.equals("ob")) ob = Double.parseDouble(params.get(k));
      if (k.equals("kc")) kc = Double.parseDouble(params.get(k));
      if (k.equals("kr")) kr = Double.parseDouble(params.get(k));
      if (k.equals("kd")) kd = Double.parseDouble(params.get(k));
      if (k.equals("ko")) ko = Double.parseDouble(params.get(k));
      if (k.equals("pc")) pc = Double.parseDouble(params.get(k));
      if (k.equals("pr")) pr = Double.parseDouble(params.get(k));
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
      if (k.equals("perim_coord"))   perimDrctd = Boolean.parseBoolean(params.get(k));
      if (k.equals("goalX")) goalX = Double.parseDouble(params.get(k));
      if (k.equals("goalY")) goalY = Double.parseDouble(params.get(k));
    } //k
  } // setParams
  
  /**
   Initialize state and working data structures: swarm of agents at (xs[], ys[]).
   Cohesion, repulsion, direction and resultant vector are intially zero. 
   Goal is at point (goal, goal).
   Cohesion and repulsion field radii are set to parameters cf, rf respectively. 
   Cohesion, repulsion, direction weightings are set to kc, kr, kd respectively.
   Initially, agents are presumed  NOT on perimeter.
   Arrays are created and sized for interagent displacments, distances, polar
    angles, effective cohesion range; these are updated for being used.
  */
  private void  initWorkingData(double[] xs, double[] ys) {
    assert xs.length == ys.length; //num x-coords == num y-coords!
    size = xs.length;
    // swarm state ...
    state = new double[N_ROWS][size];     
    onPerim = new boolean[size];
    for (int i = 0; i < size; i++) {
      state[POS_X][i] = xs[i];  state[POS_Y][i] = ys[i];
      state[COH_X][i]  = 0.0;   state[COH_Y][i] = 0.0; 
      state[REP_X][i]  = 0.0;   state[REP_Y][i] = 0.0; 
      state[DIR_X][i]  = 0.0;   state[DIR_Y][i] = 0.0; 
      state[RES_X][i]  = 0.0;   state[RES_Y][i] = 0.0; 
      state[GOAL_X][i] = goalX; state[GOAL_Y][i] = goalY;
      state[CF][i]     = cb;    state[RF][i]     = rb;
      state[KC][i] = kc; state[KR][i] = kr; state[KD][i] = kd; 
      state[PRM][i] = 0;
      state[COH_N][i]  = 0;   state[REP_N][i] = 0;
      onPerim[i] = false;
    }
    //Inter-agent data (will be updated before use): 
    xDiff = new double[size][size];  yDiff  = new double[size][size]; //displacements 
    dists = new double[size][size];  angles = new double[size][size]; //dists, angles
    ecb   =  new double[size][size]; erf   =  new double[size][size]; //eff COH,REP radii
    ekc   =  new double[size][size]; ekr   =  new double[size][size]; //eff COH,REP wts
    nbrs  =  new boolean[size][size];
  } //initWorkingData
  
  /** Constructor for swarm at xs[], ys[] ... */
  public SwarmModel(double[] xs, double[] ys, Map<String, String> prms) {
    setParams(prms);    // All parameters apart from state[][] now presumed initialised
    initWorkingData(xs, ys);
  } //constructor

  /**
   Construct swarm of (size) agents in random positions in rectangle
     (-grd+loc, -grd+loc)--(grd+loc, grd+loc). Agent 0 is at (loc, loc).
  */
  public SwarmModel(int size, double grd, double loc, Map<String, String> prms) {
    this(System.nanoTime(), size, grd, loc, prms);
  }
  
  public SwarmModel(long seed, int size, double grd, double loc,
                                       Map<String, String> prms) {
    setParams(prms);    // All params apart from state[][] presumed initialised
    this.size = size;
    double[] xs = new double[size], ys = new double[size];
    Random prng = new Random(seed);
    xs[0] = loc;  ys[0] = loc; 
    for (int i = 1; i < size; i++) {
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
    //(1) interagent displacements, distances, angles, eff coh radii, nbrs
    for (int i = 0; i < size; i++) {
      xDiff[i][i]  = 0.0;   yDiff[i][i]  = 0.0;
      dists[i][i]  = 0.0;   angles[i][i]  = 0.0;
      ecb[i][i] = state[CF][i];
      nbrs[i][i] = false;
      
      for (int j = 0; j < i; j++) {
        xDiff[i][j]  = state[POS_X][i] - state[POS_X][j];
        xDiff[j][i]  = -xDiff[i][j];
        yDiff[i][j]  = state[POS_Y][i] - state[POS_Y][j];
        yDiff[j][i]  = -yDiff[i][j];
        dists[i][j]  = Math.hypot(xDiff[i][j], yDiff[i][j]);
        dists[j][i]  = dists[i][j]; //NB x,yDiff are pntng bckwd: hence sw'd i,j
        theta = Math.atan2(yDiff[i][j], xDiff[i][j]);
        angles[j][i] = theta; // In [-pi, pi].  
        angles[i][j] = theta>0.0? theta - Math.PI: theta + Math.PI;
        ecb[i][j] = state[CF][i];
        ecb[j][i] = ecb[i][j];
      } //j
    } //i
    for (int i = 0; i < size; i++) {
      state[COH_N][i] = 0;
      for (int j = 0; j < size; j++) {
        nbrs[i][j] = (j != i && dists[i][j] <= ecb[i][j]); 
        if (nbrs[i][j]) 
          state[COH_N][i]++;
      }
    } //i

    //(2) Perimeter status
    updateOnPerim();

    //(3) effective rep field radii, COH and REP weights
    for (int i = 0; i < size; i++) {
      erf[i][i] = 0; ekc[i][i] = 0; ekr[i][i] = 0;
      for (int j = 0; j <= i; j++) {
        if (onPerim[i] && onPerim[j]) {
          erf[i][j] = state[RF][i] * pr;
          erf[j][i] = state[RF][j] * pr;
          ekc[i][j] = state[KC][i] * pc;
          ekc[j][i] = state[KC][j] * pc;
        } else {
          erf[i][j] = state[RF][i];
          erf[j][i] = state[RF][j];
          ekc[i][j] = state[KC][i];
          ekc[j][i] = state[KC][j];
        }
        ekr[i][j] = state[KR][i];
        ekr[j][i] = state[KR][j];
      } //j
    } //i
  } //updtWorkingData()


  /** Called by updtWorkingData()
   *  Update perimeter status state[PRM] of all agents in swarm.
   *  Assumes xDiff, yDiff, dists, angles, and COH_N are up to date.
   */
  void updateOnPerim() {
    for (int i = 0; i < size; i++) {
      onPerim[i] = false;
      int k = (int)state[COH_N][i];
      if (k < 3) {
        onPerim[i] = true;   //under 3 nbrs => perimeter
        continue;            //next i
      }
      int[] iNbrs = new int[k];  // //coh nbrs of agent i
      k = 0;
      for (int j=0; j<size; j++) {
        if (j != i && nbrs[j][i]) {
          iNbrs[k] = j; k++;
        }
      }
      
      sort(i, iNbrs);  // sort i's nbrs j by increasing angles[i][j]
      for (int j = 0; j < iNbrs.length; j++) {
        k = (j+1) % iNbrs.length;
        if (!nbrs[iNbrs[k]][iNbrs[j]]) { 
          onPerim[i] = true;  //two consec nbrs out of coh range => onPerim[i]
          break; // abandon j-loop
        }
        double delta = angles[i][iNbrs[k]] - angles[i][iNbrs[j]];
        if (delta < 0) delta += Math.PI * 2;
        if (delta > Math.PI) { //two consec nbrs make a reflex angle
          onPerim[i] = true;
          break;
        }
      } // end for j
    } // end for i
    for (int i = 0; i < size; i++)
      state[PRM][i] = onPerim[i]? 1.0: 0.0;
  } // updateOnPerim()

  /** Helper: sort neighbours of agent i 
   * i and the array are presumed to be indexes of agents. 
   * Array a[j] is sorted into increasing order by angles[i][a[j]] */
  private void sort(int i, int[] a ) {
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
    for (int i = 0; i < size; i++) {
      state[COH_X][i] = 0.0; state[COH_Y][i] = 0.0;
      for (int j = 0; j < size; j++)
        if (j != i && dists[j][i] <= ecb[j][i]) {
          state[COH_X][i] += (xDiff[j][i] * ekc[j][i]);
          state[COH_Y][i] += (yDiff[j][i] * ekc[j][i]);
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
    for (int i = 0; i < size; i++) {
      state[REP_N][i] = 0; state[REP_X][i] = 0.0; state[REP_Y][i] = 0.0; 
      for (int j = 0; j < size; j++) {
        if (j == i || dists[j][i] > erf[i][j]) 
          continue;
        state[REP_N][i] += 1;
        state[REP_X][i] += (1 - (erf[i][j]/dists[j][i]))*xDiff[j][i]*ekr[j][i];
        state[REP_Y][i] += (1 - (erf[i][j]/dists[j][i]))*yDiff[j][i]*ekr[j][i];
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
    for (int i = 0; i < size; i++) {
      state[REP_N][i] = 0; state[REP_X][i] = 0.0; state[REP_Y][i] = 0.0; 
      for (int j = 0; j < size; j++) {
        if (j == i || dists[j][i] > erf[i][j]) 
          continue;
        state[REP_N][i] += 1;
        dd = dists[j][i];
        state[REP_X][i] -= erf[i][j]/dd/dd * xDiff[j][i]/dd * ekr[j][i];
        state[REP_Y][i] -= erf[i][j]/dd/dd * yDiff[j][i]/dd * ekr[j][i];
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
    for (int i = 0; i < size; i++) {
      state[REP_N][i] = 0; state[REP_X][i] = 0.0; state[REP_Y][i] = 0.0; 
      for (int j = 0; j < size; j++) {
        if (j == i || dists[j][i] > erf[i][j]) 
          continue;
        state[REP_N][i] += 1;
        dd = dists[j][i];
        state[REP_X][i] -= erf[i][j]*Math.exp(-dd*expRt) * xDiff[j][i]/dd * ekr[j][i];
        state[REP_Y][i] -= erf[i][j]*Math.exp(-dd*expRt) * yDiff[j][i]/dd * ekr[j][i];
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
    updtWorkingData(); // xDiff, yDiff, dists, angles, , nbrs, perim, ecb, erf, ekc, ekr
    computeCOH();      // Updates state[COH_X], state[COH_Y] including weights

    if (repMode == LINEAR)  // state[REP_X], state[REP_Y] including weights
      computeREP_lin();
    else if (repMode == QUAD)
      computeREP_quad();
    else if (repMode == EXPTL)
      computeREP_exp();
    
    for (int i = 0; i < size; i++) {
      state[DIR_X][i] = state[KD][i]*(state[GOAL_X][i] - state[POS_X][i]);
      state[DIR_Y][i] = state[KD][i]*(state[GOAL_Y][i] - state[POS_Y][i]);

      // Resultant of the cohesion, repulsion and direction vectors:
      if (!perimDrctd || onPerim[i]) {
        state[RES_X][i] = state[COH_X][i] + state[REP_X][i] + state[DIR_X][i];
        state[RES_Y][i] = state[COH_Y][i] + state[REP_Y][i] + state[DIR_Y][i];
      } else {
        state[RES_X][i] = state[COH_X][i] + state[REP_X][i];
        state[RES_Y][i] = state[COH_Y][i] + state[REP_Y][i];
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
    for (int i = 0; i < size; i++) {
      state[POS_X][i] += state[RES_X][i];
      state[POS_Y][i] += state[RES_Y][i];
      state[POS_X][i] = Math.rint(state[POS_X][i] * snapRdg)/snapRdg;
      state[POS_Y][i] = Math.rint(state[POS_Y][i] * snapRdg)/snapRdg;
    }
  } //applyStep
  
  
  void saveState(String path) throws IOException {
    PrintWriter ptwr = new PrintWriter(new FileWriter(path));
    for (int i = 0; i < size; i++) {
      for (int k = 0; k < N_ROWS; k++) 
        ptwr.printf("%.15f  ", state[k][i]);
      ptwr.println();
    }
    ptwr.close();
  }
  

  void saveSwarm(String path) throws IOException {
    PrintWriter ptwr = new PrintWriter(new FileWriter(path));
    ptwr.println(String.format("cb %.15f", cb));
    ptwr.println(String.format("rb %.15f", rb));
    ptwr.println(String.format("ob %.15f", ob));
    ptwr.println(String.format("kc %.15f", kc));
    ptwr.println(String.format("kr %.15f", kr));
    ptwr.println(String.format("kd %.15f", kd));
    ptwr.println(String.format("ko %.15f", ko));
    ptwr.println(String.format("kc %.15f", kc));
    ptwr.println(String.format("pc %.15f", pc));
    ptwr.println(String.format("pr %.15f", pr));
    ptwr.println(String.format("exp_rate %.15f", expRt));
    ptwr.println(String.format("scaling %s",
        repMode==2? "expo": (repMode==1?"quad":"linear")));
    ptwr.println(String.format("speed %.15f", speed));
    ptwr.println(String.format("stab %.15f", stabFac));
    ptwr.println(String.format("goalX %.15f", goalX));
    ptwr.println(String.format("goalY %.15f", goalY));
    ptwr.println(String.format("perim_coord %b", perimDrctd));
    ptwr.println("# POS_X, POS_Y --");
    for (int i = 0; i < size; i++)
      ptwr.println(String.format("%.15f  %.15f", state[POS_X][i], state[POS_Y][i]));
    ptwr.close();
  }

  /** Unpack model data from a config file & build model */
  public static SwarmModel loadSwarm(String path)
                    throws IOException, NumberFormatException {
    Map<String, String> params = new HashMap<String, String>();
    List<String> xLst = new ArrayList<String>(),
                 yLst = new ArrayList<String>();
    BufferedReader fr = new BufferedReader(new FileReader(path));
    String[] tokens;
    String line = fr.readLine();
    while (line != null && line.charAt(0) != '#') { //params section
      tokens = line.split("\\s+");
      params.put(tokens[0], tokens[1]);
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
  }

} // end class

