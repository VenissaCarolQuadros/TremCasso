 /**
 **********************************************************************************************************************
 * @file       TremCasso.pde
 * @author     Team TremCasso
 * @version    V1.0
 * @date       17-March-2023
 * @brief      2nd iteration of an accessible drawing tool/colour picker for people with tremors
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

Buttons paintB, nextB, prevB;
Coloring col;
FBox paint, bottom, boundary;
ArrayList<FBody> bodies;

int page=0; // page 0= canvas; page 1= colour picker
int colour; //change this value to set colour of h_avatar in canvas
PGraphics canvas;
boolean            actionMode=false;
boolean            pageChange=false;

/* Variables */
FBox              b1,b2;
FBox              v1,v2,v3,v4,v5,v6,v7,v8,v9,v10;
FBox              g1,g2;
FBox              c1,c2,c3,c4,c5,c6,c7,c8,c9,c10;

FBox              next, prev;


/* graphical elements */
/*
PShape color1, color2, color3, color4, color5, color6;
PShape color11, color12,color13,color14,color15;
PShape color21, color22,color23,color24,color25;
PShape color31, color32,color33,color34,color35;
PShape color41, color42,color43,color44,color45;
PShape color51, color52,color53,color54,color55;
*/

PGraphics pickedColour;

int baseColor;
char  orientation='v';
int rows= 2;
int NoOfSwatches =5; 
float distanceBetweenSwatches=2.5;
float swatchSize=2.5;

float bPosX, gPosX;
float vPosX, cPosX;



/* scheduler definition ************************************************************************************************/ 
private final ScheduledExecutorService scheduler      = Executors.newScheduledThreadPool(1);
/* end scheduler definition ********************************************************************************************/ 


/* device block definitions ********************************************************************************************/
Board             haplyBoard;
Device            widgetOne;
Mechanisms        pantograph;

byte              widgetOneID                         = 5;
int               CW                                  = 0;
int               CCW                                 = 1;
boolean           renderingForce                     = false;
/* end device block definition *****************************************************************************************/



/* framerate definition ************************************************************************************************/
long              baseFrameRate                       = 120;
/* end framerate definition ********************************************************************************************/ 



/* elements definition *************************************************************************************************/

/* Screen and world setup parameters */
float             pixelsPerMeter                      = 4000.0;
float             radsPerDegree                       = 0.01745;
float             pixelsPerCentimeter                 = 40.0;


/* end effector radius in meters */
float             rEE                                 = 0.004;
float             rEEContact                          = 0.006;

/* generic data for a 2DOF device */
/* joint space */
PVector           angles                              = new PVector(0, 0);
PVector           torques                             = new PVector(0, 0);

/* task space */
PVector           posEE                               = new PVector(0, 0);
PVector           fEE                                 = new PVector(0, 0); 

/* device graphical position */
PVector           deviceOrigin                        = new PVector(0, 0);

/* World boundaries reference */
final int         worldPixelWidth                     = 1000;
final int         worldPixelHeight                    = 900;

FWorld            world;
float             worldWidth                          = 25.5;  
float             worldHeight                         = 25.0; 

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
  size(1200, 900);
  //swatchSize = ((1200)/ (distanceBetweenSwatches*NoOfSwatches))/32;
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
  haplyBoard          = new Board(this, "COM8", 0);
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

  deviceOrigin.add(worldPixelWidth/2, 0);
  
  drawGUI();
  drawColourPicker();
  /* Setup the Virtual Coupling Contact Rendering Technique */
  s                   = new HVirtualCoupling((0.75)); 
  s.h_avatar.setName("reserved");
  s.h_avatar.setSensor(false);
  //s.h_avatar.setDensity(2); 
  s.h_avatar.setFill(255,0,0); 
  s.init(world, edgeTopLeftX+worldWidth/2, edgeTopLeftY+2); 

  /* World conditions setup */
  //world.setEdges((edgeTopLeftX), (edgeTopLeftY), (edgeBottomRightX), (edgeBottomRightY)); 
  world.setEdgesRestitution(.1);
  world.setEdgesFriction(0.1);
  
  paintB= new Buttons(1040, 1300, 'h');
  nextB= new Buttons(750, 1000, 'v');
  prevB= new Buttons(0, 150, 'v');
  col= new Coloring();
  canvas= createGraphics(1200,900);
  world.draw();
  
  colour=color(0,0,0);
  /* setup framerate speed */
  frameRate(baseFrameRate);
  
  /* setup simulation thread to run at 1kHz */ 
  SimulationThread st = new SimulationThread();
  scheduler.scheduleAtFixedRate(st, 1, 1, MILLISECONDS);
}



/* draw section ********************************************************************************************************/
void draw(){
  /* put graphical code here, runs repeatedly at defined framerate in setup, else default at 60fps: */
  
  if(renderingForce == false){
    background(255);
    pageSelector(); 
    update_animation(angles.x*radsPerDegree, angles.y*radsPerDegree, posEE.x, posEE.y);
  }
  
}
/* end draw section ****************************************************************************************************/
void keyPressed(){
  if (key==' '){
    if (actionMode)
      actionMode=false;
    else
      actionMode=true;
  }
  if (key=='a'){
  print(hAPI_Fisica.worldToScreen(s.h_avatar.getX(), s.h_avatar.getY()));
      
}

  
  
}


/* simulation section **************************************************************************************************/
class SimulationThread implements Runnable{
  
  public void run(){
    /* put haptic simulation code here, runs repeatedly at 1kHz as defined in setup */
    renderingForce = true;
    
    if(haplyBoard.data_available()){
      /* GET END-EFFECTOR STATE (TASK SPACE) */
      widgetOne.device_read_data();
    
      angles.set(widgetOne.get_device_angles()); 
      posEE.set(widgetOne.get_device_position(angles.array()));
      
      posEE.set(posEE.copy().mult(200)); 
    }
    

    s.setToolPosition(edgeTopLeftX+worldWidth/2-(posEE).x, edgeTopLeftY+(posEE).y-4); 
    s.updateCouplingForce();
 
    fEE.set(-s.getVirtualCouplingForceX(), s.getVirtualCouplingForceY());
    fEE.div(100000); //dynes to newtons
    
    forceSetter();
    world.step(1.0f/1000.0f);
    
    torques.set(widgetOne.set_device_torques(fEE.array()));
    widgetOne.device_write_torques(); 
    
    if (s.h_avatar.isTouchingBody(paint)){
      if (page==0 && !pageChange)
      {
         page=1;
         pageChange=true;
         //print(pageChange);
      }
      if (page==1 && !pageChange)
      {
        page=0;
        pageChange=true;
        //print(pageChange);
      }
    }
    
    if (s.h_avatar.isTouchingBody(boundary)){
      pageChange=false;
      //print(pageChange);
    }
    
    if (page==1){
      if (s.h_avatar.isTouchingBody(next)){
      print("next");
      }
      if (s.h_avatar.isTouchingBody(prev)){
      print("previous");
      }
    }

    renderingForce = false;
  }
}
/* end simulation section **********************************************************************************************/



/* helper functions section, place helper functions here ***************************************************************/

    

void page0(){
  bodies=world.getBodies();
  //print(bodies);
  for (FBody b : bodies) { 
           if(b.getName()!="reserved"){
           b.setSensor(true);
           b.setNoFill();
           }
      }
  prev.dettachImage();
  next.dettachImage();
  Vec2 pos=hAPI_Fisica.worldToScreen(s.h_avatar.getX(), s.h_avatar.getY());
  s.h_avatar.setFill(red(colour), green(colour), blue(colour));
  col.draw(canvas, red(colour), green(colour), blue(colour), Math.round(pos.x), Math.round(pos.y), 20, actionMode);
  image(canvas,0,0);
  if (!pageChange){
    PImage img=loadImage("assets/paint1.png");
    img.resize(50,0);
    paint.attachImage(img);
  }
  world.draw();
  
}
void page1(){
  bodies=world.getBodies();
  //print(bodies);
  for (FBody b : bodies) { 
           if(b.getName()!="reserved"){
           b.setSensor(false);
           b.setFill(0);
           }
      }
  PImage img=loadImage("assets/up.png");
  img.resize(0,90);
  prev.attachImage(img);
  prev.setSensor(true);
  img=loadImage("assets/down.png");
  img.resize(0,90);
  next.attachImage(img);
  next.setSensor(true);
  if (!pageChange){
    img=loadImage("assets/canvas.png");
    img.resize(50,0);
    paint.attachImage(img);
  }
  world.draw();
}


void pageSelector() {
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
  bottom= new FBox(2, 25.5);
  bottom.setFill(0, 114, 160);
  bottom.setPosition(29,12.5);
  bottom.setStatic(true);
  bottom.setSensor(true);
  bottom.setNoStroke();
  bottom.setName("reserved");
  world.add(bottom);
  
  
  paint=new FBox(2, 25.5);
  PImage img=loadImage("assets/paint1.png");
  img.resize(50,0);
  paint.attachImage(img);
  paint.setFill(255,255,255);
  paint.setPosition(29,10);
  paint.setStatic(true);
  paint.setSensor(true);
  paint.setNoStroke();
  paint.setName("reserved");
  world.add(paint);
  
  boundary=new FBox(0.5, 25.5);
  boundary.setNoFill();
  boundary.setPosition(26.9,12.5);
  boundary.setStatic(true);
  boundary.setSensor(true);
  boundary.setNoStroke();
  boundary.setName("reserved");
  world.add(boundary);
  
  /*
  
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
  */
}

public void drawColourPicker(){
  b1                  = new FBox(0.3, 15.5);
  b1.setPosition(edgeTopLeftX+worldWidth/1.0-25, edgeTopLeftY+worldHeight/2.0-0.8); 
  b1.setFill(0);
  b1.setNoStroke();
  b1.setStaticBody(true);
  world.add(b1);
  
// Walls  
if(orientation == 'v'){
    bPosX = 25 ;
    gPosX = 25 - (swatchSize +(distanceBetweenSwatches/2)) ;
    
   for(int i =0; i< NoOfSwatches-1 ; i++){
    // vertical walls
    bPosX = bPosX - swatchSize;
    b1                  = new FBox(0.3, 4.0);
    b1.setPosition(edgeTopLeftX+worldWidth/1.0-bPosX, edgeTopLeftY+worldHeight/2.0-4.5); 
    b1.setFill(0);
    b1.setNoStroke();
    b1.setStaticBody(true);
    world.add(b1);
    
    bPosX = bPosX - distanceBetweenSwatches;
    b2                  = new FBox(0.3, 4.0);
    b2.setPosition(edgeTopLeftX+worldWidth/1.0-bPosX, edgeTopLeftY+worldHeight/2.0-4.5); 
    b2.setFill(0);
    b2.setNoStroke();
    b2.setStaticBody(true);
    world.add(b2);
    
    // horizantal walls
     g1                  = new FBox(distanceBetweenSwatches+0.3, 0.3);
    g1.setPosition(edgeTopLeftX+worldWidth/1.0-gPosX, edgeTopLeftY+worldHeight/2.0-6.4); 
    g1.setFill(0);
    g1.setNoStroke();
    g1.setStaticBody(true);
    world.add(g1);
  
     g2                  = new FBox(distanceBetweenSwatches+0.3, 0.3);
    g2.setPosition(edgeTopLeftX+worldWidth/1.0-gPosX, edgeTopLeftY+worldHeight/2.0-2.6); 
    g2.setFill(0);
    g2.setNoStroke();
    g2.setStaticBody(true);
    world.add(g2);
    
    gPosX = gPosX - (distanceBetweenSwatches + swatchSize);
   }
  
   
    b1                  = new FBox(0.3, 15.5);
    b1.setPosition(edgeTopLeftX+worldWidth/1.0-25+((distanceBetweenSwatches + swatchSize)*NoOfSwatches-distanceBetweenSwatches), edgeTopLeftY+worldHeight/2.0-0.8); 
    b1.setFill(0);
    b1.setNoStroke();
    b1.setStaticBody(true);
    world.add(b1);
   
   // shades
    if (rows == 2){
      vPosX = 25;
      cPosX = 25 - (swatchSize +(distanceBetweenSwatches/2));
      
      for(int i =0; i< NoOfSwatches-1 ; i++){
        vPosX = vPosX - swatchSize;
        v1                  = new FBox(0.3, 4.0);
        v1.setPosition(edgeTopLeftX+worldWidth/1.0-vPosX, edgeTopLeftY+worldHeight/2.0+3); 
        v1.setFill(0);
        v1.setNoStroke();
        v1.setStaticBody(true);
        world.add(v1);
        
        vPosX = vPosX -distanceBetweenSwatches;
        
        v2                  = new FBox(0.3, 4.0);
        v2.setPosition(edgeTopLeftX+worldWidth/1.0-vPosX, edgeTopLeftY+worldHeight/2.0+3); 
        v2.setFill(0);
        v2.setNoStroke();
        v2.setStaticBody(true);
        world.add(v2);
  
        c1                  = new FBox(distanceBetweenSwatches+0.3, 0.3);
        c1.setPosition(edgeTopLeftX+worldWidth/1.0-cPosX, edgeTopLeftY+worldHeight/2.0+1.1); 
        c1.setFill(0);
        c1.setNoStroke();
        c1.setStaticBody(true);
        world.add(c1);
        
        c2                  = new FBox(distanceBetweenSwatches+0.3, 0.3);
        c2.setPosition(edgeTopLeftX+worldWidth/1.0-cPosX, edgeTopLeftY+worldHeight/2.0+4.9); 
        c2.setFill(0);
        c2.setNoStroke();
        c2.setStaticBody(true);
        world.add(c2);
        
        cPosX = cPosX - (distanceBetweenSwatches + swatchSize);;
      }
     
   }
   
    prev=new FBox(26, 2);
    prev.setPosition(14, 2);
    prev.setStatic(true);
    prev.setSensor(true);
    prev.setNoStroke();
    world.add(prev);
    
    next=new FBox(26, 2);
    next.setPosition(14, 21);
    next.setStatic(true);
    next.setSensor(true);
    next.setNoStroke();
    world.add(next);
  
  }
 
 
 
 // Colors, what is the range of colors ???

  //color   x,  y,  w,  h, r,  g,  b
  /*
  if(NoOfSwatches >= 1){
    //color 1
    color1 =  create_rect(25, 160, swatchSize*40,160, 255, 237, 0);
    if (rows == 2){
      // first shade of each color
      color11 = create_rect(25, 460, swatchSize*40,160, 255, 252, 217);
      color21 = create_rect(25, 460, swatchSize*40,160, 255, 217, 217);
      color31 = create_rect(25, 460, swatchSize*40,160, 217, 227, 242);
      color41 = create_rect(25, 460, swatchSize*40,160, 217, 244, 217);   
      color51 = create_rect(25, 460, swatchSize*40,160, 255, 240, 217);
    }
  }  
  if(NoOfSwatches >= 2) {
   //color 2
    color2 =  create_rect((25+(swatchSize +distanceBetweenSwatches)*40), 160, swatchSize*40,160, 255, 0, 0);
    if (rows == 2){
      // second shade of each color
      color12 = create_rect(25+(swatchSize+distanceBetweenSwatches)*40, 460, swatchSize*40,160, 255, 249, 166);
      color22 = create_rect(25+(swatchSize+distanceBetweenSwatches)*40, 460, swatchSize*40,160, 255, 166, 166);
      color32 = create_rect(25+(swatchSize+distanceBetweenSwatches)*40, 460, swatchSize*40,160, 166, 191, 226);
      color42 = create_rect(25+(swatchSize+distanceBetweenSwatches)*40, 460, swatchSize*40,160, 166, 229, 166);  
      color52 = create_rect(25+(swatchSize+distanceBetweenSwatches)*40, 460, swatchSize*40,160, 255, 219, 166);
    }
  }  
  if(NoOfSwatches >= 3) {
    //color 3 
    color3 =  create_rect((25+(swatchSize +distanceBetweenSwatches)*80), 160, swatchSize*40,160, 0, 71, 171);
    if (rows == 2){
      // third shade of each color
      color13 = create_rect(25+(swatchSize+distanceBetweenSwatches)*80, 460, swatchSize*40,160, 255, 237, 0);
      color23 = create_rect(25+(swatchSize+distanceBetweenSwatches)*80, 460, swatchSize*40,160, 255, 0, 0);
      color33 = create_rect(25+(swatchSize+distanceBetweenSwatches)*80, 460, swatchSize*40,160, 0, 71, 171);
      color43 = create_rect(25+(swatchSize+distanceBetweenSwatches)*80, 460, swatchSize*40,160, 0, 181, 0);
      color53 = create_rect(25+(swatchSize+distanceBetweenSwatches)*80, 460, swatchSize*40,160, 255, 153, 0);
    }
  }  
  if (NoOfSwatches >=4 ) {
    //color 4
    color4 =  create_rect((25+(swatchSize +distanceBetweenSwatches)*120), 160, swatchSize*40,160, 0, 181, 0);
      if (rows == 2){
      // fourth shade of each color
      color14 = create_rect(25+(swatchSize+distanceBetweenSwatches)*120, 460, swatchSize*40,160, 230, 213, 0);
      color24 = create_rect(25+(swatchSize+distanceBetweenSwatches)*120, 460, swatchSize*40,160, 230, 0, 0);
      color34 = create_rect(25+(swatchSize+distanceBetweenSwatches)*120, 460, swatchSize*40,160, 0, 64, 154);
      color44 = create_rect(25+(swatchSize+distanceBetweenSwatches)*120, 460, swatchSize*40,160, 0, 163, 0);
      color54 = create_rect(25+(swatchSize+distanceBetweenSwatches)*120, 460, swatchSize*40,160, 230, 138, 0);
    }
  }  
  if (NoOfSwatches >= 5) {
    //color 5
    color5 =  create_rect((25+(swatchSize +distanceBetweenSwatches)*160), 160, swatchSize*40,160, 255, 153, 0);
    if (rows == 2){
      // fifth shade of each color
      color15 = create_rect(25+(swatchSize+distanceBetweenSwatches)*160, 460, swatchSize*40,160, 191, 178, 0);
      color25 = create_rect(25+(swatchSize+distanceBetweenSwatches)*160, 460, swatchSize*40,160, 191, 0, 0);
      color35 = create_rect(25+(swatchSize+distanceBetweenSwatches)*160, 460, swatchSize*40,160, 0, 33, 128);
      color45 = create_rect(25+(swatchSize+distanceBetweenSwatches)*160, 460, swatchSize*40,160, 0, 136, 0);
      color55 = create_rect(25+(swatchSize+distanceBetweenSwatches)*160, 460, swatchSize*40,160, 191, 115, 0);
    }
  }
  */
  
  
}

void update_animation(float th1, float th2, float xE, float yE){
  xE = pixelsPerMeter * xE;
  yE = pixelsPerMeter * yE;
  
  th1 = 3.14 - th1;
  th2 = 3.14 - th2;
  
  translate(xE, yE);
 
}

public void forceSetter(){
  Vec2 pos=hAPI_Fisica.worldToScreen(s.h_avatar.getX(), s.h_avatar.getY());
  PVector f= paintB.applyForces(5, 10, pos.x, fEE);
  if (page==1){
    f=prevB.applyForces(5, 10, pos.y, fEE);
    f=nextB.applyForces(-10, -20, pos.y, fEE);
  }
  fEE.set(f);
}

PVector device_to_graphics(PVector deviceFrame){
  return deviceFrame.set(-deviceFrame.x, deviceFrame.y);
}


PVector graphics_to_device(PVector graphicsFrame){
  return graphicsFrame.set(-graphicsFrame.x, graphicsFrame.y);
}


/* end helper functions section ****************************************************************************************/
