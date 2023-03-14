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
FBox              b1,b2;
FBox              v1,v2,v3,v4,v5,v6,v7,v8,v9,v10;
FBox              g1,g2;
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
char  orientation;
int rows;
int NoOfSwatches; 
float swatchSize;
float distanceBetweenSwatches;

float bPosX, gPosX;
float vPosX, cPosX;
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

// Parameters
  size(1200, 900);
  orientation = 'v'; // v or h
  rows = 1; // 1 or 2
  NoOfSwatches = 6;
  swatchSize = 2.5;
  distanceBetweenSwatches = 2.5;
  
  
  b1                  = new FBox(0.3, 15.5);
  b1.setPosition(edgeTopLeftX+worldWidth/1.0-25, edgeTopLeftY+worldHeight/2.0-2.8); 
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
    b1.setPosition(edgeTopLeftX+worldWidth/1.0-bPosX, edgeTopLeftY+worldHeight/2.0-6.5); 
    b1.setFill(0);
    b1.setNoStroke();
    b1.setStaticBody(true);
    world.add(b1);
    
    bPosX = bPosX - distanceBetweenSwatches;
    b2                  = new FBox(0.3, 4.0);
    b2.setPosition(edgeTopLeftX+worldWidth/1.0-bPosX, edgeTopLeftY+worldHeight/2.0-6.5); 
    b2.setFill(0);
    b2.setNoStroke();
    b2.setStaticBody(true);
    world.add(b2);
    
    // horizantal walls
     g1                  = new FBox(distanceBetweenSwatches+0.3, 0.3);
    g1.setPosition(edgeTopLeftX+worldWidth/1.0-gPosX, edgeTopLeftY+worldHeight/2.0-8.4); 
    g1.setFill(0);
    g1.setNoStroke();
    g1.setStaticBody(true);
    world.add(g1);
  
     g2                  = new FBox(distanceBetweenSwatches+0.3, 0.3);
    g2.setPosition(edgeTopLeftX+worldWidth/1.0-gPosX, edgeTopLeftY+worldHeight/2.0-4.6); 
    g2.setFill(0);
    g2.setNoStroke();
    g2.setStaticBody(true);
    world.add(g2);
    
    gPosX = gPosX - (distanceBetweenSwatches + swatchSize);
   }
  
   
       b1                  = new FBox(0.3, 15.5);
    b1.setPosition(edgeTopLeftX+worldWidth/1.0-25+((distanceBetweenSwatches + swatchSize)*NoOfSwatches-distanceBetweenSwatches), edgeTopLeftY+worldHeight/2.0-2.8); 
    b1.setFill(0);
    b1.setNoStroke();
    b1.setStaticBody(true);
    world.add(b1);
   
   // second level
    if (rows == 2){
      vPosX = 25;
      cPosX = 25 - (swatchSize +(distanceBetweenSwatches/2));
      
      for(int i =0; i< NoOfSwatches-1 ; i++){
        vPosX = vPosX - swatchSize;
        v1                  = new FBox(0.3, 4.0);
        v1.setPosition(edgeTopLeftX+worldWidth/1.0-vPosX, edgeTopLeftY+worldHeight/2.0+1); 
        v1.setFill(0);
        v1.setNoStroke();
        v1.setStaticBody(true);
        world.add(v1);
        
        vPosX = vPosX -distanceBetweenSwatches;
        
        v2                  = new FBox(0.3, 4.0);
        v2.setPosition(edgeTopLeftX+worldWidth/1.0-vPosX, edgeTopLeftY+worldHeight/2.0+1); 
        v2.setFill(0);
        v2.setNoStroke();
        v2.setStaticBody(true);
        world.add(v2);
  
        c1                  = new FBox(distanceBetweenSwatches+0.3, 0.3);
        c1.setPosition(edgeTopLeftX+worldWidth/1.0-cPosX, edgeTopLeftY+worldHeight/2.0-0.9); 
        c1.setFill(0);
        c1.setNoStroke();
        c1.setStaticBody(true);
        world.add(c1);
        
        c2                  = new FBox(distanceBetweenSwatches+0.3, 0.3);
        c2.setPosition(edgeTopLeftX+worldWidth/1.0-cPosX, edgeTopLeftY+worldHeight/2.0+2.9); 
        c2.setFill(0);
        c2.setNoStroke();
        c2.setStaticBody(true);
        world.add(c2);
        
        cPosX = cPosX - (distanceBetweenSwatches + swatchSize);;
      }
     
   }
  }
 
 
 
 // Colors

  //color   x,  y,  w,  h, r,  g,  b
  if(NoOfSwatches >= 1){
    //color 1
    color1 =  create_rect(25, 160, swatchSize*40,160, 255, 237, 0);
  }  if(NoOfSwatches >= 2) {
   //color 2
    color2 =  create_rect((25+(swatchSize +distanceBetweenSwatches)*40), 160, swatchSize*40,160, 255, 0, 0);
  }  if(NoOfSwatches >= 3) {
    //color 3 
    color3 =  create_rect((25+(swatchSize +distanceBetweenSwatches)*80), 160, swatchSize*40,160, 0, 71, 171);
  }  if (NoOfSwatches >=4 ) {
    //color 4
    color4 =  create_rect((25+(swatchSize +distanceBetweenSwatches)*120), 160, swatchSize*40,160, 0, 181, 0);
  }  if (NoOfSwatches >= 5) {
    //color 5
    color5 =  create_rect((25+(swatchSize +distanceBetweenSwatches)*160), 160, swatchSize*40,160, 255, 153, 0);
  }
  
  //color 1 shades
  if (rows == 2){
  //color11 = create_rect(25, 460, 90,160, 255, 252, 217);
  //color12 = create_rect(204, 460, 90,160, 255, 249, 166);
  //color13 = create_rect(384, 460, 90,160, 255, 237, 0);
  //color14 = create_rect(564, 460, 90,160, 230, 213, 0);
  //color15 = create_rect(744, 460, 90,160, 191, 178, 0);
  }
  

  //red variations
  //color21 = create_rect(25, 460, 90,160, 255, 217, 217);
  //color22 = create_rect(204, 460, 90,160, 255, 166, 166);
  //color23 = create_rect(384, 460, 90,160, 255, 0, 0);
  //color24 = create_rect(564, 460, 90,160, 230, 0, 0);
  //color25 = create_rect(744, 460, 90,160, 191, 0, 0);


  // blue variations
  //color31 = create_rect(25, 460, 90,160, 217, 227, 242);
  //color32 = create_rect(204, 460, 90,160, 166, 191, 226);
  //color33 = create_rect(384, 460, 90,160, 0, 71, 171);
  //color34 = create_rect(564, 460, 90,160, 0, 64, 154);
  //color35 = create_rect(744, 460, 90,160, 0, 33, 128);
  

  // green variations
  //color41 = create_rect(25, 460, 90,160, 217, 244, 217);
  //color42 = create_rect(204, 460, 90,160, 166, 229, 166);
  //color43 = create_rect(384, 460, 90,160, 0, 181, 0);
  //color44 = create_rect(564, 460, 90,160, 0, 163, 0);
  //color45 = create_rect(744, 460, 90,160, 0, 136, 0);
  

  //orange variations
  //color51 = create_rect(25, 460, 90,160, 255, 240, 217);
  //color52 = create_rect(204, 460, 90,160, 255, 219, 166);
  //color53 = create_rect(384, 460, 90,160, 255, 153, 0);
  //color54 = create_rect(564, 460, 90,160, 230, 138, 0);
  //color55 = create_rect(744, 460, 90,160, 191, 115, 0);
  
  
  //black
  //color6 = create_rect(917, 190, 45,400, 0, 0, 0);
  
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
   
 // if (s.getToolPositionY() < 4.4){
 //  int r = 255;
 //  int g = 255;
 //  int b = 255;
 //  s.h_avatar.setFill(r, g, b);
 //  canvas.fill(r, g, b);
   
 //  baseColor = 0;
 // }
  
 // // yellow
 //if((s.getToolPositionX() > 0.5 && s.getToolPositionX() < 3.0) && (s.getToolPositionY() > 4.4 &&  (s.getToolPositionY() < 7.6 ))){
 //  int r = 255;
 //  int g = 237;
 //  int b = 0;
 //  s.h_avatar.setFill(r, g, b);
 //  canvas.fill(r, g, b);
   
 //  baseColor = 1;
   

 //}
 //// red
 //else if ((s.getToolPositionX() > 5.0 && s.getToolPositionX() < 7.5) && (s.getToolPositionY() > 4.4 &&  (s.getToolPositionY() < 7.6 ))){
 //  int r = 255;
 //  int g = 0;
 //  int b = 0;
 //  s.h_avatar.setFill(r, g, b);
 //  canvas.fill(r, g, b);
   
 //  baseColor = 2;

   
 //} 
 //// blue 
 //else if((s.getToolPositionX() > 9.5 && s.getToolPositionX() < 12.0) && (s.getToolPositionY() > 4.4 &&  (s.getToolPositionY() < 7.6 ))){
 //  int r = 0;
 //  int g = 71;
 //  int b = 171;
 //  s.h_avatar.setFill(r, g, b);
 //  canvas.fill(r, g, b);
   
 //  baseColor = 3;
 //}
 ////green
 // else if((s.getToolPositionX() > 14.0 && s.getToolPositionX() < 16.5) && (s.getToolPositionY() > 4.4 &&  (s.getToolPositionY() < 7.6 ))){
 //  int r = 0;
 //  int g = 181;
 //  int b = 0;
 //  s.h_avatar.setFill(r, g, b);
 //  canvas.fill(r, g, b);
   
 //  baseColor = 4;
 //}
 // //orange
 // else if((s.getToolPositionX() > 18.5 && s.getToolPositionX() < 21.0) && (s.getToolPositionY() > 4.4 &&  (s.getToolPositionY() < 7.6 ))){
 //  int r = 255;
 //  int g = 153;
 //  int b = 0;
 //  s.h_avatar.setFill(r, g, b);
 //  canvas.fill(r, g, b);
   
 //  baseColor = 5;
 //}
   //black
//else if((s.getToolPositionX() > 22.5 && s.getToolPositionX() < 24.5) && (s.getToolPositionY() > 4.7 &&  (s.getToolPositionY() < 14.6 ))){
//   int r = 0;
//   int g = 0;
//   int b = 0;

//   s.h_avatar.setFill(r, g, b);
   
//   canvas.fill(r, g, b);
   
//   baseColor = 6;
// }
 
 
// // yellow variations
// if(baseColor == 1){
//   if((s.getToolPositionX() > 0.5 && s.getToolPositionX() < 3.0) && (s.getToolPositionY() > 11.6 &&  (s.getToolPositionY() < 15.4 ))){
//    int r = 255;
//    int g = 252;
//    int b = 217;
//    s.h_avatar.setFill(r, g, b);
//    canvas.fill(r, g, b);
//   } else if((s.getToolPositionX() > 5.0 && s.getToolPositionX() < 7.5) && (s.getToolPositionY() > 11.6 &&  (s.getToolPositionY() < 15.4 ))){
//    int r = 255;
//    int g = 249;
//    int b = 166;
//    s.h_avatar.setFill(r, g, b);
//    canvas.fill(r, g, b);
//   } else if((s.getToolPositionX() > 9.5 && s.getToolPositionX() < 12.0) && (s.getToolPositionY() > 11.6 &&  (s.getToolPositionY() < 15.4 ))){
//    int r = 255;
//    int g = 237;
//    int b = 0;
//    s.h_avatar.setFill(r, g, b);
//    canvas.fill(r, g, b);
//   } else if((s.getToolPositionX() > 14.0 && s.getToolPositionX() < 16.5) && (s.getToolPositionY() > 11.6 &&  (s.getToolPositionY() < 15.4 ))){
//    int r = 230;
//    int g = 213;
//    int b = 0;
//    s.h_avatar.setFill(r, g, b);
//    canvas.fill(r, g, b);
//   } else if((s.getToolPositionX() > 18.5 && s.getToolPositionX() < 21.0) && (s.getToolPositionY() > 11.6 &&  (s.getToolPositionY() < 15.4 ))){
//    int r = 191;
//    int g = 178;
//    int b = 0;
//    s.h_avatar.setFill(r, g, b);
//    canvas.fill(r, g, b);
//   }
// }
//  // red variations
// else if (baseColor == 2){
//   if((s.getToolPositionX() > 0.5 && s.getToolPositionX() < 3.0) && (s.getToolPositionY() > 11.6 &&  (s.getToolPositionY() < 15.4 ))){
//    int r = 255;
//    int g = 217;
//    int b = 217;
//    s.h_avatar.setFill(r, g, b);
//    canvas.fill(r, g, b);
//   } else if((s.getToolPositionX() > 5.0 && s.getToolPositionX() < 7.5) && (s.getToolPositionY() > 11.6 &&  (s.getToolPositionY() < 15.4 ))){
//    int r = 255;
//    int g = 166;
//    int b = 166;
//    s.h_avatar.setFill(r, g, b);
//    canvas.fill(r, g, b);
//   } else if((s.getToolPositionX() > 9.5 && s.getToolPositionX() < 12.0) && (s.getToolPositionY() > 11.6 &&  (s.getToolPositionY() < 15.4 ))){
//    int r = 255;
//    int g = 0;
//    int b = 0;
//    s.h_avatar.setFill(r, g, b);
//    canvas.fill(r, g, b);
//   } else if((s.getToolPositionX() > 14.0 && s.getToolPositionX() < 16.5) && (s.getToolPositionY() > 11.6 &&  (s.getToolPositionY() < 15.4 ))){
//    int r = 230;
//    int g = 0;
//    int b = 0;
//    s.h_avatar.setFill(r, g, b);
//    canvas.fill(r, g, b);
//   } else if((s.getToolPositionX() > 18.5 && s.getToolPositionX() < 21.0) && (s.getToolPositionY() > 11.6 &&  (s.getToolPositionY() < 15.4 ))){
//    int r = 191;
//    int g = 0;
//    int b = 0;
//    s.h_avatar.setFill(r, g, b);
//    canvas.fill(r, g, b);
//   }
// }
// // blue variations
//else if (baseColor == 3){
//   if((s.getToolPositionX() > 0.5 && s.getToolPositionX() < 3.0) && (s.getToolPositionY() > 11.6 &&  (s.getToolPositionY() < 15.4 ))){
//    int r = 217;
//    int g = 227;
//    int b = 242;
//    s.h_avatar.setFill(r, g, b);
//    canvas.fill(r, g, b);
//   } else if((s.getToolPositionX() > 5.0 && s.getToolPositionX() < 7.5) && (s.getToolPositionY() > 11.6 &&  (s.getToolPositionY() < 15.4 ))){
//    int r = 166;
//    int g = 191;
//    int b = 226;
//    s.h_avatar.setFill(r, g, b);
//    canvas.fill(r, g, b);
//   } else if((s.getToolPositionX() > 9.5 && s.getToolPositionX() < 12.0) && (s.getToolPositionY() > 11.6 &&  (s.getToolPositionY() < 15.4 ))){
//    int r = 0;
//    int g = 71;
//    int b = 171;
//    s.h_avatar.setFill(r, g, b);
//    canvas.fill(r, g, b);
//   } else if((s.getToolPositionX() > 14.0 && s.getToolPositionX() < 16.5) && (s.getToolPositionY() > 11.6 &&  (s.getToolPositionY() < 15.4 ))){
//    int r = 0;
//    int g = 64;
//    int b = 154;
//    s.h_avatar.setFill(r, g, b);
//    canvas.fill(r, g, b);
//   } else if((s.getToolPositionX() > 18.5 && s.getToolPositionX() < 21.0) && (s.getToolPositionY() > 11.6 &&  (s.getToolPositionY() < 15.4 ))){
//    int r = 0;
//    int g = 33;
//    int b = 128;
//    s.h_avatar.setFill(r, g, b);
//    canvas.fill(r, g, b);
//   }
// }
// // green variations
// else if (baseColor == 4){
//   if((s.getToolPositionX() > 0.5 && s.getToolPositionX() < 3.0) && (s.getToolPositionY() > 11.6 &&  (s.getToolPositionY() < 15.4 ))){
//    int r = 217;
//    int g = 244;
//    int b = 217;
//    s.h_avatar.setFill(r, g, b);
//    canvas.fill(r, g, b);
//   } else if((s.getToolPositionX() > 5.0 && s.getToolPositionX() < 7.5) && (s.getToolPositionY() > 11.6 &&  (s.getToolPositionY() < 15.4 ))){
//    int r = 166;
//    int g = 229;
//    int b = 166;
//    s.h_avatar.setFill(r, g, b);
//    canvas.fill(r, g, b);
//   } else if((s.getToolPositionX() > 9.5 && s.getToolPositionX() < 12.0) && (s.getToolPositionY() > 11.6 &&  (s.getToolPositionY() < 15.4 ))){
//    int r = 0;
//    int g = 181;
//    int b = 0;
//    s.h_avatar.setFill(r, g, b);
//    canvas.fill(r, g, b);
//   } else if((s.getToolPositionX() > 14.0 && s.getToolPositionX() < 16.5) && (s.getToolPositionY() > 11.6 &&  (s.getToolPositionY() < 15.4 ))){
//    int r = 0;
//    int g = 163;
//    int b = 0;
//    s.h_avatar.setFill(r, g, b);
//    canvas.fill(r, g, b);
//   } else if((s.getToolPositionX() > 18.5 && s.getToolPositionX() < 21.0) && (s.getToolPositionY() > 11.6 &&  (s.getToolPositionY() < 15.4 ))){
//    int r = 0;
//    int g = 136;
//    int b = 0;
//    s.h_avatar.setFill(r, g, b);
//    canvas.fill(r, g, b);
//   }
// }
// // orange variations
//else if (baseColor == 5){
//   if((s.getToolPositionX() > 0.5 && s.getToolPositionX() < 3.0) && (s.getToolPositionY() > 11.6 &&  (s.getToolPositionY() < 15.4 ))){
//    int r = 255;
//    int g = 240;
//    int b = 217;
//    s.h_avatar.setFill(r, g, b);
//    canvas.fill(r, g, b);
//   } else if((s.getToolPositionX() > 5.0 && s.getToolPositionX() < 7.5) && (s.getToolPositionY() > 11.6 &&  (s.getToolPositionY() < 15.4 ))){
//    int r = 255;
//    int g = 219;
//    int b = 166;
//    s.h_avatar.setFill(r, g, b);
//    canvas.fill(r, g, b);
//   } else if((s.getToolPositionX() > 9.5 && s.getToolPositionX() < 12.0) && (s.getToolPositionY() > 11.6 &&  (s.getToolPositionY() < 15.4 ))){
//    int r = 255;
//    int g = 153;
//    int b = 0;
//    s.h_avatar.setFill(r, g, b);
//    canvas.fill(r, g, b);
//   } else if((s.getToolPositionX() > 14.0 && s.getToolPositionX() < 16.5) && (s.getToolPositionY() > 11.6 &&  (s.getToolPositionY() < 15.4 ))){
//    int r = 230;
//    int g = 138;
//    int b = 0;
//    s.h_avatar.setFill(r, g, b);
//    canvas.fill(r, g, b);
//   } else if((s.getToolPositionX() > 18.5 && s.getToolPositionX() < 21.0) && (s.getToolPositionY() > 11.6 &&  (s.getToolPositionY() < 15.4 ))){
//    int r = 191;
//    int g = 115;
//    int b = 0;
//    s.h_avatar.setFill(r, g, b);
//    canvas.fill(r, g, b);
//   }
// }



 

   canvas.endDraw();
   image(canvas, edgeTopLeftX+worldWidth+425/1.0, edgeTopLeftY+worldHeight+700/1.0); 




 
  xE = pixelsPerMeter * xE;
  yE = pixelsPerMeter * yE;
  
  th1 = 3.14 - th1;
  th2 = 3.14 - th2;
  
  //shape(vertical1);
    if(NoOfSwatches >= 1){
    //color 1
      shape(color1);
  }  if(NoOfSwatches >= 2) {
   //color 2
      shape(color2);
  }  if(NoOfSwatches >= 3) {
    //color 3 
      shape(color3);
  }  if (NoOfSwatches >=4 ) {
    //color 4
      shape(color4);
  }  if (NoOfSwatches >= 5) {
    //color 5
     shape(color5);
  }


  
  //if(baseColor == 1){
  //  shape(color11);
  //  shape(color12);
  //  shape(color13);
  //  shape(color14);
  //  shape(color15);
  //} else if(baseColor == 2){
  // shape(color21);
  // shape(color22);
  // shape(color23);
  // shape(color24);
  // shape(color25);
  //} else if(baseColor == 3){
  // shape(color31);
  // shape(color32);
  // shape(color33);
  // shape(color34);
  // shape(color35);
  //} else if(baseColor == 4){
  // shape(color41);
  // shape(color42);
  // shape(color43);
  // shape(color44);
  // shape(color45);
  //} else if(baseColor == 5){
  // shape(color51);
  // shape(color52);
  // shape(color53);
  // shape(color54);
  // shape(color55);
  //}
  
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
