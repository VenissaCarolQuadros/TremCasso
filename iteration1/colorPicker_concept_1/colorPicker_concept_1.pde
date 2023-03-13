/*
 * @file       project.pde
 * @author     Yaman
 * @version    V1.0.0
 * @date       23-Feb-2023
 * @brief      iteration 1
 */

  /* library imports *****************************************************************************************************/ 
import processing.serial.*;
import static java.util.concurrent.TimeUnit.*;
import java.util.concurrent.*;
import controlP5.*;
/* end library imports *************************************************************************************************/  

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


/* virtual wall parameters */
PVector           fWall                               = new PVector(0, 0);
float             kWall                               = 800; // N/m

//PVector           endSwingWall                       = new PVector(0, 0);
//PVector           penWallv1                          = new PVector(0, 0);
//PVector           penWallv2                         = new PVector(0, 0);


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

float             gravityAcceleration                 = 980; //cm/s2

HVirtualCoupling  s;

/* World objects */
FBox              b1,b2,b3,b4,b5,b6,b7,b8,b9,b10,b11,b12;
FBox              v1,v2,v3,v4,v5,v6,v7,v8,v9,v10;
FBox              g1,g2,g3,g4,g5,g6,g7,g8,g9,g10;
FBox              c1,c2,c3,c4,c5,c6,c7,c8,c9,c10;


/* graphical elements */
PShape endEffector;
PShape vertical1;
PShape color1, color2, color3, color4, color5, color6;
PShape color11, color12,color13,color14,color15;
PShape color21, color22,color23,color24,color25;
PShape color31, color32,color33,color34,color35;
PShape color41, color42,color43,color44,color45;
PShape color51, color52,color53,color54,color55;

PImage eeImage, targetImage;

PGraphics canvas;

int baseColor;
/* end elements definition *********************************************************************************************/ 


/* setup section *******************************************************************************************************/
void setup(){
  /* screen size definition */
  size(1000, 900);
  
  /* device setup */
  haplyBoard          = new Board(this, Serial.list()[0], 0);
  widgetOne           = new Device(widgetOneID, haplyBoard);
  pantograph          = new Pantograph();
  
  widgetOne.set_mechanism(pantograph);
  
  widgetOne.add_actuator(1, CCW, 2);
  widgetOne.add_actuator(2, CW, 1);
 
  widgetOne.add_encoder(1, CCW, 241, 10752, 2);
  widgetOne.add_encoder(2, CW, -61, 10752, 1);
  
  widgetOne.device_set_parameters();
    
  hAPI_Fisica.init(this); 
  hAPI_Fisica.setScale(pixelsPerCentimeter); 
  world               = new FWorld();
  /* visual elements setup */
 
  background(255);

  deviceOrigin.add(worldPixelWidth/2, 0);
  
  /* create end effector graphics */
  //create_endEffector();
 
  /* Color1 boundaries */
  b1                  = new FBox(0.3, 15.5);
  b1.setPosition(edgeTopLeftX+worldWidth/1.0-25, edgeTopLeftY+worldHeight/2.0-2.8); 
  b1.setFill(0);
  b1.setNoStroke();
  b1.setStaticBody(true);
  world.add(b1);
 
  b2                  = new FBox(0.3, 4.0);
  b2.setPosition(edgeTopLeftX+worldWidth/1.0-22.5, edgeTopLeftY+worldHeight/2.0-6.5); 
  b2.setFill(0);
  b2.setNoStroke();
  b2.setStaticBody(true);
  world.add(b2);
  
   g1                  = new FBox(2.3, 0.3);
  g1.setPosition(edgeTopLeftX+worldWidth/1.0-21.5, edgeTopLeftY+worldHeight/2.0-8.4); 
  g1.setFill(0);
  g1.setNoStroke();
  g1.setStaticBody(true);
  world.add(g1);

     g2                  = new FBox(2.3, 0.3);
  g2.setPosition(edgeTopLeftX+worldWidth/1.0-21.5, edgeTopLeftY+worldHeight/2.0-4.6); 
  g2.setFill(0);
  g2.setNoStroke();
  g2.setStaticBody(true);
  world.add(g2);
  
  /* Color2 boundaries */
  b3                  = new FBox(0.3, 4.0);
  b3.setPosition(edgeTopLeftX+worldWidth/1.0-20.5, edgeTopLeftY+worldHeight/2.0-6.5); 
  b3.setFill(0);
  b3.setNoStroke();
  b3.setStaticBody(true);
  world.add(b3);
  
    b4                  = new FBox(0.3, 4.0);
  b4.setPosition(edgeTopLeftX+worldWidth/1.0-18, edgeTopLeftY+worldHeight/2.0-6.5); 
  b4.setFill(0);
  b4.setNoStroke();
  b4.setStaticBody(true);
  world.add(b4);
  
    g3                  = new FBox(2.3, 0.3);
  g3.setPosition(edgeTopLeftX+worldWidth/1.0-17.0, edgeTopLeftY+worldHeight/2.0-8.4); 
  g3.setFill(0);
  g3.setNoStroke();
  g3.setStaticBody(true);
  world.add(g3);
  
       g4                  = new FBox(2.3, 0.3);
  g4.setPosition(edgeTopLeftX+worldWidth/1.0-17, edgeTopLeftY+worldHeight/2.0-4.6); 
  g4.setFill(0);
  g4.setNoStroke();
  g4.setStaticBody(true);
  world.add(g4);
  
  
  /* Color3 boundaries */
    b5                  = new FBox(0.3, 4.0);
  b5.setPosition(edgeTopLeftX+worldWidth/1.0-16, edgeTopLeftY+worldHeight/2.0-6.5); 
  b5.setFill(0);
  b5.setNoStroke();
  b5.setStaticBody(true);
  world.add(b5);
  
    b6                  = new FBox(0.3, 4.0);
  b6.setPosition(edgeTopLeftX+worldWidth/1.0-13.5, edgeTopLeftY+worldHeight/2.0-6.5); 
  b6.setFill(0);
  b6.setNoStroke();
  b6.setStaticBody(true);
  world.add(b6);
  
      g5                  = new FBox(2.3, 0.3);
  g5.setPosition(edgeTopLeftX+worldWidth/1.0-12.5, edgeTopLeftY+worldHeight/2.0-8.4); 
  g5.setFill(0);
  g5.setNoStroke();
  g5.setStaticBody(true);
  world.add(g5);
  
       g6                  = new FBox(2.3, 0.3);
  g6.setPosition(edgeTopLeftX+worldWidth/1.0-12.5, edgeTopLeftY+worldHeight/2.0-4.6); 
  g6.setFill(0);
  g6.setNoStroke();
  g6.setStaticBody(true);
  world.add(g6);
  
  
  /* Color4 boundaries */
      b7                  = new FBox(0.3, 4.0);
  b7.setPosition(edgeTopLeftX+worldWidth/1.0-11.5, edgeTopLeftY+worldHeight/2.0-6.5); 
  b7.setFill(0);
  b7.setNoStroke();
  b7.setStaticBody(true);
  world.add(b7);
  
    b8                  = new FBox(0.3, 4.0);
  b8.setPosition(edgeTopLeftX+worldWidth/1.0-9, edgeTopLeftY+worldHeight/2.0-6.5); 
  b8.setFill(0);
  b8.setNoStroke();
  b8.setStaticBody(true);
  world.add(b8);
  
        g7                  = new FBox(2.3, 0.3);
  g7.setPosition(edgeTopLeftX+worldWidth/1.0-8, edgeTopLeftY+worldHeight/2.0-8.4); 
  g7.setFill(0);
  g7.setNoStroke();
  g7.setStaticBody(true);
  world.add(g7);
  
       g8                  = new FBox(2.3, 0.3);
  g8.setPosition(edgeTopLeftX+worldWidth/1.0-8, edgeTopLeftY+worldHeight/2.0-4.6); 
  g8.setFill(0);
  g8.setNoStroke();
  g8.setStaticBody(true);
  world.add(g8);
  
  
  /* Color5 boundaries */
        b9                  = new FBox(0.3, 4.0);
  b9.setPosition(edgeTopLeftX+worldWidth/1.0-7, edgeTopLeftY+worldHeight/2.0-6.5); 
  b9.setFill(0);
  b9.setNoStroke();
  b9.setStaticBody(true);
  world.add(b9);
  
    b10                  = new FBox(0.3, 4.0);
  b10.setPosition(edgeTopLeftX+worldWidth/1.0-4.5, edgeTopLeftY+worldHeight/2.0-6.5); 
  b10.setFill(0);
  b10.setNoStroke();
  b10.setStaticBody(true);
  world.add(b10);
  
          g9                  = new FBox(1.5, 0.3);
  g9.setPosition(edgeTopLeftX+worldWidth/1.0-3.9, edgeTopLeftY+worldHeight/2.0-8.4); 
  g9.setFill(0);
  g9.setNoStroke();
  g9.setStaticBody(true);
  world.add(g9);
  
       g10                  = new FBox(1.5, 0.3);
  g10.setPosition(edgeTopLeftX+worldWidth/1.0-3.9, edgeTopLeftY+worldHeight/2.0-4.6); 
  g10.setFill(0);
  g10.setNoStroke();
  g10.setStaticBody(true);
  world.add(g10);
  
  /* Black boundaries */
          b11                  = new FBox(0.3, 11.6);
  b11.setPosition(edgeTopLeftX+worldWidth/1.0-3, edgeTopLeftY+worldHeight/2.0-2.75); 
  b11.setFill(0);
  b11.setNoStroke();
  b11.setStaticBody(true);
  world.add(b11);
  
    b12                  = new FBox(0.3, 15.5);
  b12.setPosition(edgeTopLeftX+worldWidth/1.0-1.0, edgeTopLeftY+worldHeight/2.0-2.8); 
  b12.setFill(0);
  b12.setNoStroke();
  b12.setStaticBody(true);
  world.add(b12);
  
  
    /* Color1 variations boundaries */
  v1                  = new FBox(0.3, 4.0);
  v1.setPosition(edgeTopLeftX+worldWidth/1.0-25, edgeTopLeftY+worldHeight/2.0+1); 
  v1.setFill(0);
  v1.setNoStroke();
  v1.setStaticBody(true);
  world.add(v1);
 
  v2                  = new FBox(0.3, 4.0);
  v2.setPosition(edgeTopLeftX+worldWidth/1.0-22.5, edgeTopLeftY+worldHeight/2.0+1); 
  v2.setFill(0);
  v2.setNoStroke();
  v2.setStaticBody(true);
  world.add(v2);
  
   c1                  = new FBox(2.3, 0.3);
  c1.setPosition(edgeTopLeftX+worldWidth/1.0-21.5, edgeTopLeftY+worldHeight/2.0-0.9); 
  c1.setFill(0);
  c1.setNoStroke();
  c1.setStaticBody(true);
  world.add(c1);
  
     c2                  = new FBox(2.3, 0.3);
  c2.setPosition(edgeTopLeftX+worldWidth/1.0-21.5, edgeTopLeftY+worldHeight/2.0+2.9); 
  c2.setFill(0);
  c2.setNoStroke();
  c2.setStaticBody(true);
  world.add(c2);
  
  /* Color2 variations boundaries */
    v3                  = new FBox(0.3, 4.0);
  v3.setPosition(edgeTopLeftX+worldWidth/1.0-20.5, edgeTopLeftY+worldHeight/2.0+1); 
  v3.setFill(0);
  v3.setNoStroke();
  v3.setStaticBody(true);
  world.add(v3);
 
  v4                  = new FBox(0.3, 4.0);
  v4.setPosition(edgeTopLeftX+worldWidth/1.0-18, edgeTopLeftY+worldHeight/2.0+1); 
  v4.setFill(0);
  v4.setNoStroke();
  v4.setStaticBody(true);
  world.add(v4);
  
     c3                  = new FBox(2.3, 0.3);
  c3.setPosition(edgeTopLeftX+worldWidth/1.0-17, edgeTopLeftY+worldHeight/2.0-0.9); 
  c3.setFill(0);
  c3.setNoStroke();
  c3.setStaticBody(true);
  world.add(c3);
  
     c4                  = new FBox(2.3, 0.3);
  c4.setPosition(edgeTopLeftX+worldWidth/1.0-17, edgeTopLeftY+worldHeight/2.0+2.9); 
  c4.setFill(0);
  c4.setNoStroke();
  c4.setStaticBody(true);
  world.add(c4);
  
  /* Color3 variations boundaries */
      v5                  = new FBox(0.3, 4.0);
  v5.setPosition(edgeTopLeftX+worldWidth/1.0-16, edgeTopLeftY+worldHeight/2.0+1); 
  v5.setFill(0);
  v5.setNoStroke();
  v5.setStaticBody(true);
  world.add(v5);
 
  v6                  = new FBox(0.3, 4.0);
  v6.setPosition(edgeTopLeftX+worldWidth/1.0-13.5, edgeTopLeftY+worldHeight/2.0+1); 
  v6.setFill(0);
  v6.setNoStroke();
  v6.setStaticBody(true);
  world.add(v6);
  
       c5                  = new FBox(2.3, 0.3);
  c5.setPosition(edgeTopLeftX+worldWidth/1.0-12.5, edgeTopLeftY+worldHeight/2.0-0.9); 
  c5.setFill(0);
  c5.setNoStroke();
  c5.setStaticBody(true);
  world.add(c5);
  
     c6                  = new FBox(2.3, 0.3);
  c6.setPosition(edgeTopLeftX+worldWidth/1.0-12.5, edgeTopLeftY+worldHeight/2.0+2.9); 
  c6.setFill(0);
  c6.setNoStroke();
  c6.setStaticBody(true);
  world.add(c6);
  
    /* Color4 variations boundaries */
      v7                  = new FBox(0.3, 4.0);
  v7.setPosition(edgeTopLeftX+worldWidth/1.0-11.5, edgeTopLeftY+worldHeight/2.0+1); 
  v7.setFill(0);
  v7.setNoStroke();
  v7.setStaticBody(true);
  world.add(v7);
 
  v8                  = new FBox(0.3, 4.0);
  v8.setPosition(edgeTopLeftX+worldWidth/1.0-9, edgeTopLeftY+worldHeight/2.0+1); 
  v8.setFill(0);
  v8.setNoStroke();
  v8.setStaticBody(true);
  world.add(v8);
  
         c7                  = new FBox(2.3, 0.3);
  c7.setPosition(edgeTopLeftX+worldWidth/1.0-8, edgeTopLeftY+worldHeight/2.0-0.9); 
  c7.setFill(0);
  c7.setNoStroke();
  c7.setStaticBody(true);
  world.add(c7);
  
     c8                  = new FBox(2.3, 0.3);
  c8.setPosition(edgeTopLeftX+worldWidth/1.0-8, edgeTopLeftY+worldHeight/2.0+2.9); 
  c8.setFill(0);
  c8.setNoStroke();
  c8.setStaticBody(true);
  world.add(c8);
  
  
      /* Color5 variations boundaries */
      v9                  = new FBox(0.3, 4.0);
  v9.setPosition(edgeTopLeftX+worldWidth/1.0-7, edgeTopLeftY+worldHeight/2.0+1); 
  v9.setFill(0);
  v9.setNoStroke();
  v9.setStaticBody(true);
  world.add(v9);
 
  v10                  = new FBox(0.3, 4.0);
  v10.setPosition(edgeTopLeftX+worldWidth/1.0-4.5, edgeTopLeftY+worldHeight/2.0+1); 
  v10.setFill(0);
  v10.setNoStroke();
  v10.setStaticBody(true);
  world.add(v10);
  
  c9                  = new FBox(1.5, 0.3);
  c9.setPosition(edgeTopLeftX+worldWidth/1.0-3.9, edgeTopLeftY+worldHeight/2.0-0.9); 
  c9.setFill(0);
  c9.setNoStroke();
  c9.setStaticBody(true);
  world.add(c9);
  
  c10                  = new FBox(1.5, 0.3);
  c10.setPosition(edgeTopLeftX+worldWidth/1.0-3.9, edgeTopLeftY+worldHeight/2.0+2.9); 
  c10.setFill(0);
  c10.setNoStroke();
  c10.setStaticBody(true);
  world.add(c10);
 

  //yellow
  color1 =  create_rect(25, 160, 90,160, 255, 237, 0);
  // yellow variations
  color11 = create_rect(25, 460, 90,160, 255, 252, 217);
  color12 = create_rect(204, 460, 90,160, 255, 249, 166);
  color13 = create_rect(384, 460, 90,160, 255, 237, 0);
  color14 = create_rect(564, 460, 90,160, 230, 213, 0);
  color15 = create_rect(744, 460, 90,160, 191, 178, 0);
  
  
  //red
  color2 =  create_rect(205, 160, 90,160, 255, 0, 0);
  //red variations
  color21 = create_rect(25, 460, 90,160, 255, 217, 217);
  color22 = create_rect(204, 460, 90,160, 255, 166, 166);
  color23 = create_rect(384, 460, 90,160, 255, 0, 0);
  color24 = create_rect(564, 460, 90,160, 230, 0, 0);
  color25 = create_rect(744, 460, 90,160, 191, 0, 0);

  //blue
  color3 =  create_rect(384, 160, 90,160, 0, 71, 171);
  // blue variations
  color31 = create_rect(25, 460, 90,160, 217, 227, 242);
  color32 = create_rect(204, 460, 90,160, 166, 191, 226);
  color33 = create_rect(384, 460, 90,160, 0, 71, 171);
  color34 = create_rect(564, 460, 90,160, 0, 64, 154);
  color35 = create_rect(744, 460, 90,160, 0, 33, 128);
  
  //green
  color4 =  create_rect(564, 160, 90,160, 0, 181, 0);
  // green variations
  color41 = create_rect(25, 460, 90,160, 217, 244, 217);
  color42 = create_rect(204, 460, 90,160, 166, 229, 166);
  color43 = create_rect(384, 460, 90,160, 0, 181, 0);
  color44 = create_rect(564, 460, 90,160, 0, 163, 0);
  color45 = create_rect(744, 460, 90,160, 0, 136, 0);
  
  //orange
  color5 =  create_rect(744, 160, 90,160, 255, 153, 0);
  //orange variations
  color51 = create_rect(25, 460, 90,160, 255, 240, 217);
  color52 = create_rect(204, 460, 90,160, 255, 219, 166);
  color53 = create_rect(384, 460, 90,160, 255, 153, 0);
  color54 = create_rect(564, 460, 90,160, 230, 138, 0);
  color55 = create_rect(744, 460, 90,160, 191, 115, 0);
  
  
  //black
  color6 = create_rect(917, 190, 45,400, 0, 0, 0);
  
  baseColor = 0;
  /* Setup the Virtual Coupling Contact Rendering Technique */
  s                   = new HVirtualCoupling((0.75)); 
  //eeImage = loadImage("ball.png"); 
  //eeImage.resize(40, 40); 
  //s.h_avatar.attachImage(eeImage); 
  s.h_avatar.setSensor(true);

  s.init(world, edgeTopLeftX+worldWidth/2, edgeTopLeftY+2); 

  canvas = createGraphics(150, 150);
  
  /* World conditions setup */
  world.setGravity((0.0), gravityAcceleration); //1000 cm/(s^2) 
  world.setEdgesRestitution(.1);
  world.setEdgesFriction(0.1);
  
  world.draw();
 
  /* setup framerate speed */
  frameRate(baseFrameRate);
  
  /* setup simulation thread to run at 1kHz */ 
  SimulationThread st = new SimulationThread();
  scheduler.scheduleAtFixedRate(st, 1, 1, MILLISECONDS);
}
/* end setup section ***************************************************************************************************/


/* draw section ********************************************************************************************************/
void draw(){
  /* put graphical code here, runs repeatedly at defined framerate in setup, else default at 60fps: */
  if(renderingForce == false){
    update_animation(angles.x*radsPerDegree, angles.y*radsPerDegree, posEE.x, posEE.y);
  }
}
/* end draw section ****************************************************************************************************/


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
    
    world.step(1.0f/1000.0f);
    s.h_avatar.setSensor(false);


   torques.set(widgetOne.set_device_torques(fEE.array()));
   widgetOne.device_write_torques(); 
   /* end force feedback calculation */

    renderingForce = false;
  }
}
/* end simulation section **********************************************************************************************/


/* helper functions section, place helper functions here ***************************************************************/
void create_endEffector(){
  float rEEAni = pixelsPerMeter * rEE;

  endEffector = createShape(ELLIPSE, deviceOrigin.x, deviceOrigin.y-3, 4*rEEAni, 4*rEEAni);
  endEffector.setStroke(color(0));
  strokeWeight(5);
  
}


PShape create_wall(float x1, float y1, float x2, float y2){
  x1 = pixelsPerMeter * x1;
  y1 = pixelsPerMeter * y1;
  x2 = pixelsPerMeter * x2;
  y2 = pixelsPerMeter * y2;
  

  return createShape(LINE, deviceOrigin.x + x1, deviceOrigin.y + y1, deviceOrigin.x + x2, deviceOrigin.y+y2);
}

PShape create_rect(float x, float y, float w, float h,int r, int g, int b){
  stroke(255);
  fill(r,g,b);
  return createShape(RECT,  x,  y,  w, h);
}

void update_animation(float th1, float th2, float xE, float yE){
  background(255);
  
  //print(s.getToolPositionX());
  //print("--------");
  //var canvasColor = s.h_avatar.;
   canvas.beginDraw();
   canvas.background(255);
   canvas.rect(edgeTopLeftX+worldWidth/1.0+10, edgeTopLeftY+worldHeight/1.0+10, 80,80);
   
  if (s.getToolPositionY() < 4.4){
   int r = 255;
   int g = 255;
   int b = 255;
   s.h_avatar.setFill(r, g, b);
   canvas.fill(r, g, b);
   
   baseColor = 0;
  }
  
  // yellow
 if((s.getToolPositionX() > 0.5 && s.getToolPositionX() < 3.0) && (s.getToolPositionY() > 4.4 &&  (s.getToolPositionY() < 7.6 ))){
   int r = 255;
   int g = 237;
   int b = 0;
   s.h_avatar.setFill(r, g, b);
   canvas.fill(r, g, b);
   
   baseColor = 1;
   

 }
 // red
 else if ((s.getToolPositionX() > 5.0 && s.getToolPositionX() < 7.5) && (s.getToolPositionY() > 4.4 &&  (s.getToolPositionY() < 7.6 ))){
   int r = 255;
   int g = 0;
   int b = 0;
   s.h_avatar.setFill(r, g, b);
   canvas.fill(r, g, b);
   
   baseColor = 2;

   
 } 
 // blue 
 else if((s.getToolPositionX() > 9.5 && s.getToolPositionX() < 12.0) && (s.getToolPositionY() > 4.4 &&  (s.getToolPositionY() < 7.6 ))){
   int r = 0;
   int g = 71;
   int b = 171;
   s.h_avatar.setFill(r, g, b);
   canvas.fill(r, g, b);
   
   baseColor = 3;
 }
 //green
  else if((s.getToolPositionX() > 14.0 && s.getToolPositionX() < 16.5) && (s.getToolPositionY() > 4.4 &&  (s.getToolPositionY() < 7.6 ))){
   int r = 0;
   int g = 181;
   int b = 0;
   s.h_avatar.setFill(r, g, b);
   canvas.fill(r, g, b);
   
   baseColor = 4;
 }
  //orange
  else if((s.getToolPositionX() > 18.5 && s.getToolPositionX() < 21.0) && (s.getToolPositionY() > 4.4 &&  (s.getToolPositionY() < 7.6 ))){
   int r = 255;
   int g = 153;
   int b = 0;
   s.h_avatar.setFill(r, g, b);
   canvas.fill(r, g, b);
   
   baseColor = 5;
 }
   //black
else if((s.getToolPositionX() > 22.5 && s.getToolPositionX() < 24.5) && (s.getToolPositionY() > 4.7 &&  (s.getToolPositionY() < 14.6 ))){
   int r = 0;
   int g = 0;
   int b = 0;

   s.h_avatar.setFill(r, g, b);
   
   canvas.fill(r, g, b);
   
   baseColor = 6;
 }
 
 
 // yellow variations
 if(baseColor == 1){
   if((s.getToolPositionX() > 0.5 && s.getToolPositionX() < 3.0) && (s.getToolPositionY() > 11.6 &&  (s.getToolPositionY() < 15.4 ))){
    int r = 255;
    int g = 252;
    int b = 217;
    s.h_avatar.setFill(r, g, b);
    canvas.fill(r, g, b);
   } else if((s.getToolPositionX() > 5.0 && s.getToolPositionX() < 7.5) && (s.getToolPositionY() > 11.6 &&  (s.getToolPositionY() < 15.4 ))){
    int r = 255;
    int g = 249;
    int b = 166;
    s.h_avatar.setFill(r, g, b);
    canvas.fill(r, g, b);
   } else if((s.getToolPositionX() > 9.5 && s.getToolPositionX() < 12.0) && (s.getToolPositionY() > 11.6 &&  (s.getToolPositionY() < 15.4 ))){
    int r = 255;
    int g = 237;
    int b = 0;
    s.h_avatar.setFill(r, g, b);
    canvas.fill(r, g, b);
   } else if((s.getToolPositionX() > 14.0 && s.getToolPositionX() < 16.5) && (s.getToolPositionY() > 11.6 &&  (s.getToolPositionY() < 15.4 ))){
    int r = 230;
    int g = 213;
    int b = 0;
    s.h_avatar.setFill(r, g, b);
    canvas.fill(r, g, b);
   } else if((s.getToolPositionX() > 18.5 && s.getToolPositionX() < 21.0) && (s.getToolPositionY() > 11.6 &&  (s.getToolPositionY() < 15.4 ))){
    int r = 191;
    int g = 178;
    int b = 0;
    s.h_avatar.setFill(r, g, b);
    canvas.fill(r, g, b);
   }
 }
  // red variations
 else if (baseColor == 2){
   if((s.getToolPositionX() > 0.5 && s.getToolPositionX() < 3.0) && (s.getToolPositionY() > 11.6 &&  (s.getToolPositionY() < 15.4 ))){
    int r = 255;
    int g = 217;
    int b = 217;
    s.h_avatar.setFill(r, g, b);
    canvas.fill(r, g, b);
   } else if((s.getToolPositionX() > 5.0 && s.getToolPositionX() < 7.5) && (s.getToolPositionY() > 11.6 &&  (s.getToolPositionY() < 15.4 ))){
    int r = 255;
    int g = 166;
    int b = 166;
    s.h_avatar.setFill(r, g, b);
    canvas.fill(r, g, b);
   } else if((s.getToolPositionX() > 9.5 && s.getToolPositionX() < 12.0) && (s.getToolPositionY() > 11.6 &&  (s.getToolPositionY() < 15.4 ))){
    int r = 255;
    int g = 0;
    int b = 0;
    s.h_avatar.setFill(r, g, b);
    canvas.fill(r, g, b);
   } else if((s.getToolPositionX() > 14.0 && s.getToolPositionX() < 16.5) && (s.getToolPositionY() > 11.6 &&  (s.getToolPositionY() < 15.4 ))){
    int r = 230;
    int g = 0;
    int b = 0;
    s.h_avatar.setFill(r, g, b);
    canvas.fill(r, g, b);
   } else if((s.getToolPositionX() > 18.5 && s.getToolPositionX() < 21.0) && (s.getToolPositionY() > 11.6 &&  (s.getToolPositionY() < 15.4 ))){
    int r = 191;
    int g = 0;
    int b = 0;
    s.h_avatar.setFill(r, g, b);
    canvas.fill(r, g, b);
   }
 }
 // blue variations
else if (baseColor == 3){
   if((s.getToolPositionX() > 0.5 && s.getToolPositionX() < 3.0) && (s.getToolPositionY() > 11.6 &&  (s.getToolPositionY() < 15.4 ))){
    int r = 217;
    int g = 227;
    int b = 242;
    s.h_avatar.setFill(r, g, b);
    canvas.fill(r, g, b);
   } else if((s.getToolPositionX() > 5.0 && s.getToolPositionX() < 7.5) && (s.getToolPositionY() > 11.6 &&  (s.getToolPositionY() < 15.4 ))){
    int r = 166;
    int g = 191;
    int b = 226;
    s.h_avatar.setFill(r, g, b);
    canvas.fill(r, g, b);
   } else if((s.getToolPositionX() > 9.5 && s.getToolPositionX() < 12.0) && (s.getToolPositionY() > 11.6 &&  (s.getToolPositionY() < 15.4 ))){
    int r = 0;
    int g = 71;
    int b = 171;
    s.h_avatar.setFill(r, g, b);
    canvas.fill(r, g, b);
   } else if((s.getToolPositionX() > 14.0 && s.getToolPositionX() < 16.5) && (s.getToolPositionY() > 11.6 &&  (s.getToolPositionY() < 15.4 ))){
    int r = 0;
    int g = 64;
    int b = 154;
    s.h_avatar.setFill(r, g, b);
    canvas.fill(r, g, b);
   } else if((s.getToolPositionX() > 18.5 && s.getToolPositionX() < 21.0) && (s.getToolPositionY() > 11.6 &&  (s.getToolPositionY() < 15.4 ))){
    int r = 0;
    int g = 33;
    int b = 128;
    s.h_avatar.setFill(r, g, b);
    canvas.fill(r, g, b);
   }
 }
 // green variations
 else if (baseColor == 4){
   if((s.getToolPositionX() > 0.5 && s.getToolPositionX() < 3.0) && (s.getToolPositionY() > 11.6 &&  (s.getToolPositionY() < 15.4 ))){
    int r = 217;
    int g = 244;
    int b = 217;
    s.h_avatar.setFill(r, g, b);
    canvas.fill(r, g, b);
   } else if((s.getToolPositionX() > 5.0 && s.getToolPositionX() < 7.5) && (s.getToolPositionY() > 11.6 &&  (s.getToolPositionY() < 15.4 ))){
    int r = 166;
    int g = 229;
    int b = 166;
    s.h_avatar.setFill(r, g, b);
    canvas.fill(r, g, b);
   } else if((s.getToolPositionX() > 9.5 && s.getToolPositionX() < 12.0) && (s.getToolPositionY() > 11.6 &&  (s.getToolPositionY() < 15.4 ))){
    int r = 0;
    int g = 181;
    int b = 0;
    s.h_avatar.setFill(r, g, b);
    canvas.fill(r, g, b);
   } else if((s.getToolPositionX() > 14.0 && s.getToolPositionX() < 16.5) && (s.getToolPositionY() > 11.6 &&  (s.getToolPositionY() < 15.4 ))){
    int r = 0;
    int g = 163;
    int b = 0;
    s.h_avatar.setFill(r, g, b);
    canvas.fill(r, g, b);
   } else if((s.getToolPositionX() > 18.5 && s.getToolPositionX() < 21.0) && (s.getToolPositionY() > 11.6 &&  (s.getToolPositionY() < 15.4 ))){
    int r = 0;
    int g = 136;
    int b = 0;
    s.h_avatar.setFill(r, g, b);
    canvas.fill(r, g, b);
   }
 }
 // orange variations
else if (baseColor == 5){
   if((s.getToolPositionX() > 0.5 && s.getToolPositionX() < 3.0) && (s.getToolPositionY() > 11.6 &&  (s.getToolPositionY() < 15.4 ))){
    int r = 255;
    int g = 240;
    int b = 217;
    s.h_avatar.setFill(r, g, b);
    canvas.fill(r, g, b);
   } else if((s.getToolPositionX() > 5.0 && s.getToolPositionX() < 7.5) && (s.getToolPositionY() > 11.6 &&  (s.getToolPositionY() < 15.4 ))){
    int r = 255;
    int g = 219;
    int b = 166;
    s.h_avatar.setFill(r, g, b);
    canvas.fill(r, g, b);
   } else if((s.getToolPositionX() > 9.5 && s.getToolPositionX() < 12.0) && (s.getToolPositionY() > 11.6 &&  (s.getToolPositionY() < 15.4 ))){
    int r = 255;
    int g = 153;
    int b = 0;
    s.h_avatar.setFill(r, g, b);
    canvas.fill(r, g, b);
   } else if((s.getToolPositionX() > 14.0 && s.getToolPositionX() < 16.5) && (s.getToolPositionY() > 11.6 &&  (s.getToolPositionY() < 15.4 ))){
    int r = 230;
    int g = 138;
    int b = 0;
    s.h_avatar.setFill(r, g, b);
    canvas.fill(r, g, b);
   } else if((s.getToolPositionX() > 18.5 && s.getToolPositionX() < 21.0) && (s.getToolPositionY() > 11.6 &&  (s.getToolPositionY() < 15.4 ))){
    int r = 191;
    int g = 115;
    int b = 0;
    s.h_avatar.setFill(r, g, b);
    canvas.fill(r, g, b);
   }
 }



 

   canvas.endDraw();
   image(canvas, edgeTopLeftX+worldWidth+425/1.0, edgeTopLeftY+worldHeight+700/1.0); 




 
  xE = pixelsPerMeter * xE;
  yE = pixelsPerMeter * yE;
  
  th1 = 3.14 - th1;
  th2 = 3.14 - th2;
  
  //shape(vertical1);
  shape(color1);
  shape(color2);
  shape(color3);
  shape(color4);
  shape(color5);
  shape(color6);
  
  if(baseColor == 1){
    shape(color11);
    shape(color12);
    shape(color13);
    shape(color14);
    shape(color15);
  } else if(baseColor == 2){
   shape(color21);
   shape(color22);
   shape(color23);
   shape(color24);
   shape(color25);
  } else if(baseColor == 3){
   shape(color31);
   shape(color32);
   shape(color33);
   shape(color34);
   shape(color35);
  } else if(baseColor == 4){
   shape(color41);
   shape(color42);
   shape(color43);
   shape(color44);
   shape(color45);
  } else if(baseColor == 5){
   shape(color51);
   shape(color52);
   shape(color53);
   shape(color54);
   shape(color55);
  }
  
  stroke(0);
  world.draw();

  translate(xE, yE);
 
}

PVector device_to_graphics(PVector deviceFrame){
  return deviceFrame.set(-deviceFrame.x, deviceFrame.y);
}


PVector graphics_to_device(PVector graphicsFrame){
  return graphicsFrame.set(-graphicsFrame.x, graphicsFrame.y);
}
/* end helper functions section ****************************************************************************************/
