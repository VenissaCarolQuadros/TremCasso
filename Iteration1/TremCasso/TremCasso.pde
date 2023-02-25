 /**
 **********************************************************************************************************************
 * @file       TremCasso.pde
 * @author     Venissa C Quadros
 * @version    V1.0.0
 * @date       23-February-2023
 * @brief      1st iteration of an accessible drawing tool for people with tremors
 **********************************************************************************************************************
 * @attention
 *
 *
 **********************************************************************************************************************
 */
 
 
 
 /* library imports *****************************************************************************************************/ 
import processing.serial.*;
import static java.util.concurrent.TimeUnit.*;
import java.util.concurrent.*;
/* end library imports *************************************************************************************************/  

import controlP5.*;

ControlP5 cp5;
ColourPicker cp;
Coloring col;
FBox paint;
FBox bottom;
FBox settings;
float[] counters;
int delay=1200;
int page=0;
int colour;
PGraphics canvas;
FPoly              up;
FPoly              down;
FPoly              left;
FPoly              right;
FPoly              diag;
FBox               back;
boolean            actionMode=false;
/* scheduler definition ************************************************************************************************/ 
private final ScheduledExecutorService scheduler      = Executors.newScheduledThreadPool(1);
/* end scheduler definition ********************************************************************************************/ 


/* device block definitions ********************************************************************************************/
Board             haplyBoard;
Device            widgetOne;
Mechanisms        pantograph;

byte              widgetOneID                         = 3;
int               CW                                  = 0;
int               CCW                                 = 1;
boolean           rendering_force                     = false;
/* end device block definition *****************************************************************************************/



/* framerate definition ************************************************************************************************/
long              baseFrameRate                       = 120;
/* end framerate definition ********************************************************************************************/ 



/* elements definition *************************************************************************************************/

/* Screen and world setup parameters */
float             pixelsPerCentimeter                 = 40.0;

/* generic data for a 2DOF device */
/* joint space */
PVector           angles                              = new PVector(0, 0);
PVector           torques                             = new PVector(0, 0);

/* task space */
PVector           pos_ee                              = new PVector(0, 0);
PVector           f_ee                                = new PVector(0, 0); 
PImage img;
/* World boundaries */
FWorld            world;
float             worldWidth                          = 25.0;  
float             worldHeight                         = 10.0; 

float             edgeTopLeftX                        = 0.0; 
float             edgeTopLeftY                        = 0.0; 
float             edgeBottomRightX                    = worldWidth; 
float             edgeBottomRightY                    = worldHeight;

/* Initialization of virtual tool */
HVirtualCoupling  s;

/* end elements definition *********************************************************************************************/



/* setup section *******************************************************************************************************/
void setup(){
  /* put setup code here, run once: */
  
  /* screen size definition */
  size(1000, 700);
  
  /* device setup */
  img = loadImage("assets/img_colormap.gif");
  
  /**  
   * The board declaration needs to be changed depending on which USB serial port the Haply board is connected.
   * In the base example, a connection is setup to the first detected serial device, this parameter can be changed
   * to explicitly state the serial port will look like the following for different OS:
   *
   *      windows:      haplyBoard = new Board(this, "COM10", 0);
   *      linux:        haplyBoard = new Board(this, "/dev/ttyUSB0", 0);
   *      mac:          haplyBoard = new Board(this, "/dev/cu.usbmodem1411", 0);
   */
  haplyBoard          = new Board(this, "COM3", 0);
  widgetOne           = new Device(widgetOneID, haplyBoard);
  pantograph          = new Pantograph();

  
  widgetOne.set_mechanism(pantograph);
  
  widgetOne.add_actuator(1, CCW, 2);
  widgetOne.add_actuator(2, CW, 1);
 
  widgetOne.add_encoder(1, CCW, 241, 10752, 2);
  widgetOne.add_encoder(2, CW, -61, 10752, 1);
  
  widgetOne.device_set_parameters();
  
  /* 2D physics scaling and world creation */
  hAPI_Fisica.init(this); 
  hAPI_Fisica.setScale(pixelsPerCentimeter); 
  world               = new FWorld();
  
  drawButtons(new float[] {4.5, 9.75});
  drawGUI();
  /* Setup the Virtual Coupling Contact Rendering Technique */
  s                   = new HVirtualCoupling((1)); 
  s.h_avatar.setDensity(2); 
  s.h_avatar.setFill(255,0,0); 
  s.init(world, edgeTopLeftX+worldWidth/2, edgeTopLeftY+2); 

  /* World conditions setup */
  //world.setEdges((edgeTopLeftX), (edgeTopLeftY), (edgeBottomRightX), (edgeBottomRightY)); 
  world.setEdgesRestitution(.4);
  world.setEdgesFriction(0.5);
  
  cp=new ColourPicker(new int[]{0,0}, new int[]{0,0}, img, world);
  col= new Coloring();
  canvas= createGraphics(1000,620);
  world.draw();
  
  colour=color(0,0,0);
  /* setup framerate speed */
  frameRate(baseFrameRate);
  
  counters= new float[]{millis(), millis(), millis(), millis()};
  /* setup simulation thread to run at 1kHz */ 
  SimulationThread st = new SimulationThread();
  scheduler.scheduleAtFixedRate(st, 1, 1, MILLISECONDS);
}



/* draw section ********************************************************************************************************/
void draw(){
  /* put graphical code here, runs repeatedly at defined framerate in setup, else default at 60fps: */
  background(255);
  pageSelector(); 
}
/* end draw section ****************************************************************************************************/
void keyPressed(){
  if (key==' '){
    if (actionMode)
      actionMode=false;
    else
      actionMode=true;
  }
  if (page==1){
    if (key=='s'){
      cp.down();
    }
    if (key=='w'){
       cp.up();
    }
    if (key=='a'){
       cp.left();
    }
    if (key=='d'){
       cp.right();
    }
  }
}


/* simulation section **************************************************************************************************/
class SimulationThread implements Runnable{
  
  public void run(){
    /* put haptic simulation code here, runs repeatedly at 1kHz as defined in setup */
    rendering_force = true;
    
    if(haplyBoard.data_available()){
      /* GET END-EFFECTOR STATE (TASK SPACE) */
      widgetOne.device_read_data();
    
      angles.set(widgetOne.get_device_angles()); 
      pos_ee.set(widgetOne.get_device_position(angles.array()));
      pos_ee.set(pos_ee.copy().mult(200));  
    }
    
    s.setToolPosition(edgeTopLeftX+worldWidth/2-(pos_ee).x+2, edgeTopLeftY+(pos_ee).y-3); 
    s.updateCouplingForce();
    f_ee.set(-s.getVCforceX(), s.getVCforceY());
    f_ee.div(20000); //
    
    torques.set(widgetOne.set_device_torques(f_ee.array()));
    widgetOne.device_write_torques();
  
    world.step(1.0f/1000.0f);
    if (page==1 && actionMode==true){
    if (s.h_avatar.isTouchingBody(up)){
      if (checkDelay(0))
        cp.up();
    }
    if (s.h_avatar.isTouchingBody(down)){
      if (checkDelay(1))
        cp.down();
    }
    if (s.h_avatar.isTouchingBody(left)){
      if (checkDelay(2))
        cp.left();
    }
    if (s.h_avatar.isTouchingBody(right)){
      if (checkDelay(3))
        cp.right();
    }
    
    if (s.h_avatar.isTouchingBody(diag)){
      s.h_avatar.setDamping(900);
    }
    else{
      s.h_avatar.setDamping(0);
    }
    
  }
    if (s.h_avatar.isTouchingBody(back)){
        actionMode=false;
        page=0;
    }
    
    if (s.h_avatar.isTouchingBody(paint)){
      actionMode=true;
      page=1;
    }
    /*if (s.h_avatar.isTouchingBody(settings)){
      page=0;
    }*/
    rendering_force = false;
  }
}
/* end simulation section **********************************************************************************************/



/* helper functions section, place helper functions here ***************************************************************/
public void drawButtons(float[] positions)
    {
      up=new FPoly();
      up.vertex(positions[0]+0.75, positions[1]);
      up.vertex(positions[0]+5.25, positions[1]);
      up.vertex(positions[0]+3, positions[1]+2.25);
      up.setStatic(true);
      up.setSensor(true);
      up.setNoStroke();
      world.add(up);
      
      down=new FPoly();
      down.vertex(positions[0]+0.75, positions[1]+5);
      down.vertex(positions[0]+5.25, positions[1]+5);
      down.vertex(positions[0]+3, positions[1]+2.75);
      down.setStatic(true);
      down.setSensor(true);
      down.setNoStroke();
      world.add(down);
      
      left=new FPoly();
      left.vertex(positions[0]+0.5, positions[1]+0.25);
      left.vertex(positions[0]+0.5, positions[1]+4.75);
      left.vertex(positions[0]+2.75, positions[1]+2.5);
      left.setStatic(true);
      left.setSensor(true);
      left.setNoStroke();
      world.add(left);
      
      right=new FPoly();
      right.vertex(positions[0]+5.5, positions[1]+0.25);
      right.vertex(positions[0]+5.5, positions[1]+4.75);
      right.vertex(positions[0]+3.25, positions[1]+2.5);
      right.setStatic(true);
      right.setSensor(true);
      right.setNoStroke();
      world.add(right);
      
      diag= new FPoly();
      diag.vertex(positions[0]+3.25, positions[1]+2.5);
      diag.vertex(positions[0]+5.5, positions[1]+4.75);
      diag.vertex(positions[0]+5.25, positions[1]+5);
      diag.vertex(positions[0]+3, positions[1]+2.75);
      diag.vertex(positions[0]+0.75, positions[1]+5);
      diag.vertex(positions[0]+0.5, positions[1]+4.75);
      diag.vertex(positions[0]+2.75, positions[1]+2.5);
      diag.vertex(positions[0]+0.5, positions[1]+0.25);
      diag.vertex(positions[0]+0.75, positions[1]);
      diag.vertex(positions[0]+3, positions[1]+2.25);
      diag.vertex(positions[0]+5.25, positions[1]);
      diag.vertex(positions[0]+5.5, positions[1]+0.25);
      diag.setNoFill();
      //diag.setFill(255,0,0);
      diag.setStatic(true);
      diag.setSensor(true);
      diag.setNoStroke();
      world.add(diag);
      
      back=new FBox(2,1);
      PImage img=loadImage("assets/exit.png");
      img.resize(0,100);
      back.attachImage(img);
      back.setPosition(23,2);
      back.setStatic(true);
      back.setSensor(true);
      back.setNoStroke();
      world.add(back);
      
    }
    
public boolean checkDelay(int index){
  if (millis()-counters[index]>delay){
    counters= new float[]{millis(), millis(), millis(), millis()};
    return true;
  }
  else
    return false;
}

void page0(){
  Vec2 pos=hAPI_Fisica.worldToScreen(s.h_avatar.getX(), s.h_avatar.getY());
  up.setNoFill();
  down.setNoFill();
  right.setNoFill();
  left.setNoFill();
  back.dettachImage();
  s.h_avatar.setFill(red(colour), green(colour), blue(colour));
  col.draw(canvas, red(colour), green(colour), blue(colour), Math.round(pos.x), Math.round(pos.y), 20, actionMode);
  image(canvas,0,0);
  world.draw();
  
}
void page1(){
  cp.draw(g);
  up.setFill(0,0,0);
  down.setFill(0,0,0);
  left.setFill(0,0,0);
  right.setFill(0,0,0);
  PImage img=loadImage("assets/exit.png");
  img.resize(0,100);
  back.attachImage(img);
  s.h_avatar.setFill(255, 0, 0);
  world.draw();
  
  colour=cp.getColour(g);
}
void pageSelector() { // Replaces method("") for PJS
  switch(page) {
  case 0:
    page0();
    break;
 
  case 1: 
    page1();
    break;
 
  }
}

public void drawGUI(){
  bottom= new FBox(25, 2);
  bottom.setFill(0, 114, 160);
  bottom.setPosition(12.5,16.5);
  bottom.setStatic(true);
  bottom.setSensor(true);
  bottom.setNoStroke();
  world.add(bottom);
  
  
  paint=new FBox(3.5, 1.90);
  PImage img=loadImage("assets/paint.png");
  img.resize(0,70);
  paint.attachImage(img);
  paint.setFill(255,255,255);
  paint.setPosition(23,16.5);
  paint.setStatic(true);
  paint.setSensor(true);
  paint.setNoStroke();
  world.add(paint);
  
  FBox partition=new FBox(0.5, 5);
  partition.setNoFill();
  partition.setPosition(20.5,18);
  partition.setStatic(true);
  partition.setNoStroke();
  world.add(partition);
  
  settings=new FBox(3.5, 1.90);
  PImage img1=loadImage("assets/settings.png");
  img1.resize(0,70);
  settings.attachImage(img1);
  settings.setFill(255,255,255);
  settings.setPosition(18,16.5);
  settings.setStatic(true);
  settings.setSensor(true);
  settings.setNoStroke();
  world.add(settings);
}
/* end helper functions section ****************************************************************************************/
