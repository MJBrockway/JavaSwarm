import java.awt.*;
import java.awt.event.*;
import javax.swing.*;
import java.io.*;
import java.util.*;

/** SwarmView.java
  Graphical display of SwarmModel on grid GSIZE pixels square.
  ORG=GSIZE defines logical (model) origin:
    graphicX = ORG + modelX*scFact; modelX = (graphicX - ORG)/scFact
    graphicY = ORG - modelY*scFact; modelY = (ORG - graphicY)/scFact
  Mouse press displays on comsole model coordinates pointed at.
  Timer tick -> steps the model and repaints
  Control panel has run/pause/single-step; faster/slower timer,
    shorter/longer simulation step, zoom (sf *10, /10).
 */
public class SwarmView extends JPanel implements
            MouseListener, MouseMotionListener, ActionListener, FocusListener {
  public static Color agentClr = Color.black, prmClr = Color.red;
  public static final int GSIZE = 2000, ORG = GSIZE/2, tmrInt=64;
  double scFact = 20.0, sfMul = 5.0, stepSz;
  int stepNo = 1, runLim = 999999999;
  SwarmModel model;
  javax.swing.Timer timer;

  //Controls
  JPanel pnlCtrl;
  JButton btnRunPse, btnStep, btnSlwr, btnFstr, btnZoomIn, btnZoomOut, btnDmpSt, btnSvSwm;
  JCheckBox chkCohLns, chkFnGrd;
  JLabel lblCrds, lblStep, lblScFct, lblTmrInt;
  JTextField txtRunLim;

  /** Constructor */
  public SwarmView(SwarmModel m) {
    timer = new javax.swing.Timer(tmrInt, this);
    model = m;
    addMouseListener(this); addMouseMotionListener(this);

    pnlCtrl = new JPanel();
    pnlCtrl.setPreferredSize( new Dimension(130, 250));
    lblCrds = new JLabel("(-------, -------)"); pnlCtrl.add(lblCrds);
    btnStep = new JButton("Step");  pnlCtrl.add(btnStep);
    btnStep.addActionListener(this);
    lblStep = new JLabel(String.format("  %06d     ", stepNo)); pnlCtrl.add(lblStep);
    btnRunPse = new JButton("Run to");  pnlCtrl.add(btnRunPse);
    btnRunPse.addActionListener(this);
    txtRunLim = new JTextField(String.format("%d", runLim ), 9);  pnlCtrl.add(txtRunLim);
    txtRunLim.addFocusListener(this);
    txtRunLim.setHorizontalAlignment(JTextField.CENTER);
    btnFstr = new JButton("Faster");  pnlCtrl.add(btnFstr);
    btnFstr.addActionListener(this);
    lblTmrInt = new JLabel(String.format("%12d",tmrInt)); pnlCtrl.add(lblTmrInt);
    btnSlwr = new JButton("Slower");  pnlCtrl.add(btnSlwr);
    btnSlwr.addActionListener(this);
    btnZoomOut = new JButton("Zoom out");  pnlCtrl.add(btnZoomOut);
    btnZoomOut.addActionListener(this);
    lblScFct = new JLabel(String.format("%12.4f",scFact)); pnlCtrl.add(lblScFct);
    btnZoomIn = new JButton("Zoom in");  pnlCtrl.add(btnZoomIn);
    btnZoomIn.addActionListener(this);
    btnDmpSt = new JButton("Dump state");  pnlCtrl.add(btnDmpSt);
    btnDmpSt.addActionListener(this);
    btnSvSwm = new JButton("Save swarm");  pnlCtrl.add(btnSvSwm);
    btnSvSwm.addActionListener(this);

    chkCohLns = new JCheckBox("Show Coh", false);  pnlCtrl.add(chkCohLns);
    chkCohLns.addActionListener(this);
    chkFnGrd =  new JCheckBox("Fine grid", true);  pnlCtrl.add(chkFnGrd);
    chkFnGrd.addActionListener(this);
    stepSz = m.speed;
    model.computeStep(stepSz);
  }

  /** Needed to complement the scroll bars */
  public Dimension getPreferredSize() {
    return new Dimension(GSIZE, GSIZE);
  }

  public void paintComponent(Graphics g) {
    int gx, gy;
    super.paintComponent(g);
    if (chkFnGrd.isSelected()) {
      g.setColor(Color.gray.brighter()); //grid lines at 10-px intervals
      for (int i = 0; i<GSIZE; i+=10) {
        g.drawLine(0,i,GSIZE,i);
        g.drawLine(i,0,i,GSIZE);
      }
    }
    g.setColor(Color.green.darker());           //grid lines at 100-px intervals
    for (int i = 0; i<GSIZE; i+=100) {
      g.drawLine(0,i,GSIZE,i);
      g.drawLine(i,0,i,GSIZE);
    }
    g.setColor(Color.blue);            //axes centred at (0,0) in model coords.
    g.drawLine(0,ORG,GSIZE,ORG);
    g.drawLine(ORG,0,ORG,GSIZE);

    //Plot swarm
    for (int i=0; i<model.swmSz; i++) {
      gx =  (int)(model.getX(i)*scFact) + ORG - 2;
      gy = -(int)(model.getY(i)*scFact) + ORG - 2;
      g.setColor(model.prm[i]==1? prmClr: agentClr);
      g.drawOval(gx, gy, 5, 5);
    }
    if (chkCohLns.isSelected()) { 
      for (int  i=0; i<model.swmSz; i++) 
        for (int j=0; j<i; j++) 
          if (model.nbrs[i][j] || model.nbrs[j][i]) {
            g.setColor((model.prm[i]==1 && model.prm[j]==1)? Color.red:Color.gray);
            g.drawLine(
              (int)(model.getX(i)*scFact) + ORG, -(int)(model.getY(i)*scFact) + ORG,
              (int)(model.getX(j)*scFact) + ORG, -(int)(model.getY(j)*scFact) + ORG);
          }
    }
  }

  public void mouseEntered(MouseEvent e) {}
  public void mouseExited(MouseEvent e) {}
  public void mouseClicked(MouseEvent e) {}
  public void mouseReleased(MouseEvent e) {}
  public void mouseDragged(MouseEvent e) { }

  public void mouseMoved(MouseEvent e) {
    double x = (double)(e.getX()-ORG)/scFact,
           y = (double)(ORG-e.getY())/scFact; 
    lblCrds.setText(String.format("(%7.3f,%7.3f)", x, y));
    System.out.printf("(%7.3f,%7.3f)\r", x, y);
  }
  
  public void mousePressed(MouseEvent e) {
    if (timer.isRunning())
      return;
    
    for (int i = 0; i < model.swmSz; i++) 
      if (Math.hypot(model.getX(i) * scFact +ORG - e.getX(),
                    -model.getY(i) * scFact +ORG - e.getY()) < 5) {
        System.out.printf("Agent %d at (%f, %f) has %d neighbours:\n\t",
          i, model.getX(i), model.getY(i), (int)model.cohN[i]);
        for (int j = 0; j < model.swmSz; j++) 
          if (model.nbrs[i][j])
            System.out.printf(
              "%d:%f ∟ %.1f; ", j, model.dists[i][j], model.angles[i][j]*180/Math.PI);
        System.out.printf("\nRepellors for Agent %d:\n\t", i);
        for (int j = 0; j < model.swmSz; j++) 
          if (model.repels[i][j])
            System.out.printf(
              "%d:%f ∟ %.1f; ", j, model.dists[i][j], model.angles[i][j]*180/Math.PI);
        System.out.println();
        }     
  }

  public void focusGained(FocusEvent e) {}

  public void focusLost(FocusEvent e) {
    if (e.getSource() == txtRunLim) {
      try {
        runLim = Integer.parseInt(txtRunLim.getText());
      } catch (NumberFormatException x) {
        JOptionPane.showMessageDialog(this, "Enter a valid positive integer in the box");
        txtRunLim.setText(String.format("%d", runLim ));
        txtRunLim.requestFocus();
      }
    }
  }
  
  public void actionPerformed(ActionEvent e) {
    Object src = e.getSource();
    if (src == btnRunPse) {
      if (timer.isRunning()) {
        timer.stop();
        btnRunPse.setText("Run to");
      } else {
        timer.start();
        btnRunPse.setText("Pause");
      }
    }  
    else if (src == btnStep) {
      model.applyStep();
      model.computeStep(stepSz);
      repaint();
      stepNo ++;
      lblStep.setText(String.format("  %06d     ", stepNo));
    }
    else if (src == timer) {
      if (stepNo < runLim) {
        model.applyStep();
        model.computeStep(stepSz);
        repaint();
        stepNo ++;
      } else {
        timer.stop();
        btnRunPse.setText("Run to");
      }        
      lblStep.setText(String.format("  %06d     ", stepNo));
    } else if (src == btnSlwr && timer.getDelay() < 1024) {
      timer.setDelay(timer.getDelay()*2);
      lblTmrInt.setText(String.format("%12d", timer.getDelay()));
    } else if (src == btnFstr && timer.getDelay() > 16) {
      timer.setDelay(timer.getDelay()/2);
      lblTmrInt.setText(String.format("%12d", timer.getDelay()));
    } else if (src == btnZoomIn && scFact < 10000) {
      scFact *= sfMul;
      sfMul = sfMul==5? 2: 5;
      lblScFct.setText(String.format("%12.4f", scFact));
      repaint();
    } else if (src == btnZoomOut && scFact > 0.0001) {
      sfMul = sfMul==5? 2: 5;
      scFact /= sfMul;
      lblScFct.setText(String.format("%12.4f", scFact));
      repaint();
    } else if (src == btnDmpSt) {
      try {
        JFileChooser chsr = new JFileChooser(".");
        if (chsr.showSaveDialog(this) == JFileChooser.APPROVE_OPTION)
          model.saveState(chsr.getSelectedFile().getPath());
      } catch (IOException x) {
        System.err.println(x);
      }
    } else if (src == btnSvSwm) {
      try {
        JFileChooser chsr = new JFileChooser(".");
        if (chsr.showSaveDialog(this) == JFileChooser.APPROVE_OPTION)
          model.saveSwarm(chsr.getSelectedFile().getPath());
      } catch (IOException x) {
        System.err.println(x);
      }
    } else if (src == chkCohLns || src == chkFnGrd) {
      repaint();
    }
  }

  /** Given a model, construct view and assemble it into a JFrame */
  static void makeViewFrame(SwarmModel model, String name) {
    SwarmView view = new SwarmView(model);
    JFrame frame = new JFrame("Swarm View - " + name);
    JScrollPane p = new JScrollPane(view);
    frame.add(p, BorderLayout.CENTER);
    frame.add(view.pnlCtrl, BorderLayout.WEST);
    frame.setSize(800,800);
    frame.setVisible(true);
    frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
    frame.doLayout();
    p.getHorizontalScrollBar().setValue(GSIZE/3);
    p.getVerticalScrollBar().setValue(GSIZE/3);
  }

  /********************************* Main ********************************/
  public static void main(String[] args)
                     throws IOException, org.json.JSONException {
    if (args.length == 0) {
      System.out.println(
        "Usage: (java -cp .:org.json.jar) SwarmView path-to-config [f]\n"
        + "Use F option to choose flat rather than json format.");
      return;
    }
    if (args.length < 2 || args[1].charAt(0) != 'f')
      makeViewFrame(SwarmModel.loadSwarmJson(args[0]), args[0]);
    else
      makeViewFrame(SwarmModel.loadSwarmFlat(args[0]), args[0]);
  } // end main

} //end class
