 /**
 **********************************************************************************************************************
 * @file       Haptic_Physics_Template.pde
 * @author     Steve Ding, Colin Gallacher
 * @version    V3.0.0
 * @date       27-September-2018
 * @brief      Base project template for use with pantograph 2-DOF device and 2-D physics engine
 *             creates a blank world ready for creation
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

int damping = 800; // damping parameter


/* generic data for a 2DOF device */
/* joint space */
PVector           angles                              = new PVector(0, 0);
PVector           torques                             = new PVector(0, 0);

/* task space */
PVector           pos_ee                              = new PVector(0, 0);
PVector           f_ee                                = new PVector(0, 0); 

/* World boundaries */
FWorld            world;
float             worldWidth                          = 25.0;  
float             worldHeight                         = 12.0; 

float             edgeTopLeftX                        = 0.0; 
float             edgeTopLeftY                        = 0.0; 
float             edgeBottomRightX                    = worldWidth; 
float             edgeBottomRightY                    = worldHeight;

/* Initialization of virtual tool */
HVirtualCoupling  s;

/* Hello Wall*/
FBox wall;
FCircle ball;
FBox winner;
FCircle tst;
PFont font;
boolean touch = true;
boolean win = false;

FLine line;
FBox palette;
FPoly polygon;
FCircle circle;
PImage buttonImage;

ArrayList<FPoly> colorSelector1 = new ArrayList<FPoly>();
ArrayList<PShape> colorSelectorVisible = new ArrayList<PShape>();
FBox[] colorSelector2 = new FBox[5]; 
ArrayList<FBox> swatchesSensors = new ArrayList<FBox>();

boolean removed = true;

PShape testing;
color effectorColor = color(255, 255, 255);
int count = 20; // number of colors to choose from

boolean painting = false;
PGraphics canvas = new PGraphics();


// PID Controller
// used to compute the time difference between two loops for differentiation
long oldtime = 0;
// for changing update rate
int iter = 0;
/// PID stuff
float P = 0.3;
// for I
float I = 0.2;
float cumerrorx = 0;
float cumerrory = 0;
// for D
float oldex = 0.0f;
float oldey = 0.0f;
float D = 0.1;
//for exponential filter on differentiation
float diffx = 0;
float diffy = 0;
float buffx = 0;
float buffy = 0;
float smoothing = 0.80;

float xr = 0;
float yr = 0;

float x_m,y_m;



/* end elements definition *********************************************************************************************/



/* setup section *******************************************************************************************************/
void setup(){
  /* put setup code here, run once: */
  
  /* screen size definition */
  size(1000, 480);
  
  
  /* device setup */
  
  /**  
   * The board declaration needs to be changed depending on which USB serial port the Haply board is connected.
   * In the base example, a connection is setup to the first detected serial device, this parameter can be changed
   * to explicitly state the serial port will look like the following for different OS:
   *
   *      windows:      haplyBoard = new Board(this, "COM10", 0);
   *      linux:        haplyBoard = new Board(this, "/dev/ttyUSB0", 0);
   *      mac:          haplyBoard = new Board(this, "/dev/cu.usbmodem1411", 0);
   */
  haplyBoard          = new Board(this, Serial.list()[2], 0);
  widgetOne           = new Device(widgetOneID, haplyBoard);
  pantograph          = new Pantograph();
  
  println(Serial.list());
  
  //ArrayList<FPoly> colorSelector1 = new ArrayList<FPoly>();
  
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
  
  font = createFont("Arial", 14, true);
  PFont.list();
  
  palette = new FBox(2.3, 10.55);
  palette.setPosition(23.15, 6);
  palette.setStrokeWeight(3);
  palette.setFill(220, 220, 220);
  palette.setStatic(true);
  palette.setSensor(true);
  //world.add(palette);
  
  
  buttonImage = new PImage();
  buttonImage = loadImage("../data/colorWheel2.png");
  circle = new FCircle(3);
  circle.setSensor(true);
  circle.setPosition(24.25, 0.75);
  circle.setStatic(true);
  circle.attachImage(buttonImage);
  world.add(circle);
  
  FBox c = new FBox(2, 10.5);
  c.setPosition(23.25, 6);
  c.setSensor(true);
  c.setStatic(true);
  c.setNoStroke();
  c.setNoFill();
  swatchesSensors.add(c);
  world.add(swatchesSensors.get(0));

  
  canvas = createGraphics(1200, 680);
  canvas.beginDraw();
  canvas.background(255);
  canvas.endDraw();
  
  
  /* Setup the Virtual Coupling Contact Rendering Technique */
  s                   = new HVirtualCoupling((0.5)); 
  s.h_avatar.setDensity(2); 
  s.h_avatar.setFill(effectorColor); 
  s.init(world, edgeTopLeftX+worldWidth/2, edgeTopLeftY+2); 
  s.h_avatar.setDamping(damping);
  
  /* World conditions setup */
  world.setGravity((0.0), (300.0)); //1000 cm/(s^2)
  world.setEdges((edgeTopLeftX), (edgeTopLeftY), (edgeBottomRightX), (edgeBottomRightY)); 
  world.setEdgesRestitution(.4);
  world.setEdgesFriction(0.5);
  
  world.draw();
  
  
  /* setup framerate speed */
  frameRate(baseFrameRate);
  
  
  /* setup simulation thread to run at 1kHz */ 
  SimulationThread st = new SimulationThread();
  scheduler.scheduleAtFixedRate(st, 1, 1, MILLISECONDS);
}

void makeWall(float widthh, float heightt, float x, float y){
  wall = new FBox(widthh,heightt);
  wall.setPosition(x, y);
  wall.setFill(0,0,0);
  wall.setStatic(true);  
  world.add(wall);
}
void makeBall(float radius, float x, float y){
  ball = new FCircle(radius);
  ball.setPosition(x, y);
  ball.setFill(random(70, 255),random(70, 255),random(70, 255));
  ball.setStatic(false);  
  world.add(ball);
}
//void makeColorSwatch(float x, float y, float r, float g, float b, int jj){
//  int i = swatchesSensors.size();
//  FBox c = new FBox(2, 10.5);
//  c.setPosition(x+0.1, y);
//  c.setSensor(true);
//  c.setStatic(true);
//  //c.setNoStroke();
//  //c.setNoFill();
//  swatchesSensors.add(c);
//  world.add(swatchesSensors.get(i));
  
//}

void selectColor1(){
  if(removed == true){
    world.remove(circle);
    removed = false;
  }
}

void removeColor1(){
  if(removed == false){
    world.add(circle);
    removed = true;
  }
}

void makeSwatch(int x, int y, int hight){
  beginShape();
  rect(x, y, 70, hight);
  endShape();
}
void makePalette1(){
  int hight = 420/count;
  colorMode(HSB, 360, 100, 100);
  for (int i = 0; i<count; i++){
    
    fill((360/count)*i, 100, 100);
    
    makeSwatch(900, 30+i*hight, hight);
  }
  colorMode(RGB, 255, 255, 255);

  
}

void keyPressed() {
  if (key == ' ') { // pressing spacebar makes walls flexible
    if(!removed && s.h_avatar.isTouchingBody(swatchesSensors.get(0))){
    }else{
      painting = !painting;
      if(painting){
        s.h_avatar.setDamping(damping);
        s.h_avatar.setHapticStiffness(0);
        world.remove(circle);
        //println("circle should be removed");
      }else
        s.h_avatar.setDamping(damping);
        s.h_avatar.setHapticStiffness(800);
        world.add(circle);
    }
  }
}

void determineColor(){
  int yLoc = (int)(pixelsPerCentimeter*s.h_avatar.getY());
  
  int hight = 420/count;
  colorMode(HSB, 360, 100, 100);
  int tile = (int) (yLoc-30) / hight;
  effectorColor = color(360/count*tile, 100, 100);
  colorMode(RGB, 255, 255, 255);
  //return colour;
}

void colorSelectorHaptics(){
  float hapticRange = 0.1 * pixelsPerCentimeter; //0.3
  int hight = 420/count;
  int yLocation = (int)(pixelsPerCentimeter*s.h_avatar.getY());
  
  for (int jj = 1; jj<count; jj++){
    
    if(abs(yLocation-(30+jj*hight)) < hapticRange){
      //println("line crossed");
      //println(constrain(P*(yLocation-(30+jj*hight)),-4,4) + constrain(I*cumerrory,-4,4) + constrain(D*diffy,-8,8));
      if(yLocation-(30+jj*hight) < 0){
        //println(constrain(P*((yLocation-(30+jj*hight))+hapticRange),-3,3) + constrain(I*cumerrory,-4,4) + constrain(D*diffy,-8,8));
        f_ee.y += constrain(P*((yLocation-(30+jj*hight))+hapticRange),-3,3) + constrain(I*cumerrory,-4,4) + constrain(D*diffy,-8,8);
      }else{
        //println(constrain(P*((yLocation-(30+jj*hight))-hapticRange),-3,3) + constrain(I*cumerrory,-4,4) + constrain(D*diffy,-8,8));
        f_ee.y += constrain(P*((yLocation-(30+jj*hight))-hapticRange),-3,3) + constrain(I*cumerrory,-4,4) + constrain(D*diffy,-8,8);
      }
    }
      
  }
  

}

/* draw section ********************************************************************************************************/
void draw(){
  /* put graphical code here, runs repeatedly at defined framerate in setup, else default at 60fps: */
  background(255);
    
  if(painting){
    if(world.getBodies().contains(circle)){
      world.remove(circle);
    }
    s.h_avatar.setStroke(255, 0, 0);
    s.h_avatar.setStrokeWeight(3);
    canvas.beginDraw();
    canvas.noStroke();
    canvas.fill(effectorColor);
    int x = (int)(pixelsPerCentimeter*s.h_avatar.getX());
    int y = (int)(pixelsPerCentimeter*s.h_avatar.getY());
    canvas.circle(x, y, pixelsPerCentimeter);
    
    world.draw();
    canvas.endDraw();
    //image(canvas, 0, 0);
  }else{
    s.h_avatar.setStroke(0, 0, 0);
    s.h_avatar.setStrokeWeight(1);
  }
  image(canvas, 0, 0);
  
  // Calculating time and force parameters for the PID controller
  float xE = pixelsPerCentimeter * pos_ee.x;
  float yE = pixelsPerCentimeter * pos_ee.y;
  long timedif = System.nanoTime()-oldtime;

  float dist_X = x_m-xE;
  float dist_Y = y_m-yE;
  //println(dist_Y*k + " " +dist_Y*k);
  // println(timedif);
  if(timedif > 0) {
    buffx = (dist_X-oldex)/timedif*1000*1000;
    buffy = (dist_Y-oldey)/timedif*1000*1000;            

    diffx = smoothing*diffx + (1.0-smoothing)*buffx;
    diffy = smoothing*diffy + (1.0-smoothing)*buffy;
    oldex = dist_X;
    oldey = dist_Y;
    oldtime=System.nanoTime();
  }
  if(!removed){
    makePalette1();
    determineColor();
  }else{
    s.h_avatar.setFillColor(effectorColor);
  }

  
  //s.h_avatar.setSize(1);
  fill(effectorColor);
  circle((int)(pixelsPerCentimeter*s.h_avatar.getX()), (int)(pixelsPerCentimeter*s.h_avatar.getY()), 40);
  world.draw(); 
}
/* end draw section ****************************************************************************************************/



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
    
    s.setToolPosition(edgeTopLeftX+worldWidth/2-(pos_ee).x+2, edgeTopLeftY+(pos_ee).y-4); 
    s.updateCouplingForce();
    //f_ee.set(-s.getVCforceX(), s.getVCforceY());
    f_ee.set(-s.getVirtualCouplingForceX(), s.getVirtualCouplingForceY());
    f_ee.div(10000); //
    
    
    if (s.h_avatar.isTouchingBody(circle)){
      selectColor1();
    }else{
      //touch = false;
    }
    if (s.h_avatar.isTouchingBody(winner)){
      win = true;
    }
    if(!removed){
      if(s.h_avatar.isTouchingBody(swatchesSensors.get(0))){
        colorSelectorHaptics();
      }else{
        removeColor1();
      }
    }
    
    torques.set(widgetOne.set_device_torques(f_ee.array()));
    widgetOne.device_write_torques();
  
    world.step(1.0f/1000.0f);
  
    rendering_force = false;
    //if(!init){
    //  init = true;
    //}
  }
}
/* end simulation section **********************************************************************************************/



/* helper functions section, place helper functions here ***************************************************************/

/* end helper functions section ****************************************************************************************/
