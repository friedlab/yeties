/* --------------------------------------------------------------------------
* Yeties - A funky kinect snowball fight game 
* --------------------------------------------------------------------------
* prog:  Matthias Wölfel,
*        Elke Müller,
*        Jan Felix Reuter,
         Thomas Schenker,
	 Alexander Liebrich
*        and others (need_to_be_added)
* date:  04/12/2011 (m/d/y)
* ver:   0.1
* ----------------------------------------------------------------------------
*/

// === setup ==================================================
boolean autoPoseDetection = true;
boolean useFullscreen = false;

static int CHECK_SKELETON_COUNT = 6;
static int PLAYER_COUNT = 3;
static boolean ROTATE_PLAYER = true;
static boolean ROTATE_POSE_ON_LOAD = false;
static boolean NORMALIZE_POSE_ON_LOAD = true;

static boolean showDebugUI = true;

// ============================================================

// import fullscreen.*; 
// FullScreen fs; 

import SimpleOpenNI.*;
import oscP5.*;
import netP5.*;
// import processing.opengl.*;

SimpleOpenNI context;
OscP5 oscP5;
NetAddress myRemoteLocation;

// settings

/////////////////////
// debug UI
PVector[] PLAYER_COLORS = new PVector[PLAYER_COUNT];
PFont fontA32;
PFont fontA12;
PImage[] foto;
PImage[][] warnings;
PImage shapeOfUser;  
PImage kineticspace;

boolean switchDisplay = false;
boolean updateDisplay = true;
int warning[] = new int[PLAYER_COUNT];

// gesture recognition: 
int displayCost = 1;

// gesture recognition: 
int N = 25;
int M = 2*N;

int counter = 0;
int counter2 = 0;
int counterEvent = 0;

// gesture recognition: Relative Array of objects
Pose[][] grid;
Pose[][] move;

// gesture recognition
boolean foundSkeleton = false;


static int MAX_GESTURE_COUNT = 10;
int steps[] = new int[MAX_GESTURE_COUNT];
float speed[] = new float[MAX_GESTURE_COUNT];
//TODO: more than 2 players
float cost[][] = new float[PLAYER_COUNT][MAX_GESTURE_COUNT]; 
float costLast[][] = new float[PLAYER_COUNT][MAX_GESTURE_COUNT]; 
boolean empty[] = new boolean[MAX_GESTURE_COUNT];

Data data;
RingBuffer[] ringbuffer;

// ==== COPY ===
// LHA
// help variables

// ring buffer for hand pose, which contains only the left hand and right hand in the absolute coordinates
RingBufferForThrowAction[] ringbufferHand;

// the actual hand pose
HandPoseAbsolute actualHandPose = new HandPoseAbsolute();

// the old hand pose ( 10 poses before)
HandPoseAbsolute oldHandPose = new HandPoseAbsolute();

// latenz
int X = 10;
// ==== COPY ===


////////////////////////////////////
// game logic
PVector playerNecks[] = new PVector[PLAYER_COUNT];
PVector playerActualHands[] = new PVector[PLAYER_COUNT];
PVector playerOldHands[] = new PVector[PLAYER_COUNT];


// removed: is in PLAYER_COUNT: int spielerzahl = 3;
//int modus = 3; //TODO: welcher modus hat welche bedeutung?
static int PLAYER_MODUS_COUNT = 3;

boolean ballThrow = false;
PVector throwStartPos;
PVector throwEndPos;

////////////////////////////////////
// game ui
PGraphics pg;

PImage[][] playas = new PImage[PLAYER_COUNT][PLAYER_MODUS_COUNT];
PImage bg;


float kinectHeight = 0.5f;
float kinectDistance = 0.2f;
float otherKinectHeight = 0.5f;
float otherKinectDistance = 0.2f;
float cameraDistance = 2f;
float cameraHeight = 1.4f;

PMatrix3D kinectScale = new PMatrix3D(0.001f, 0, 0, 0,
                                        0,      0.001f, 0, 0,
                                        0,      0, 0.001f, 0,
                                        0, 0,0,1);
                                        
PMatrix3D kinectToField = new PMatrix3D(1, 0, 0, 0,
                                        0, 1, 0, kinectHeight,
                                        0, 0, 1, kinectDistance,
                                        0, 0, 0, 1);
                                        

PMatrix3D kinectToFieldScaled = new PMatrix3D();


PMatrix3D otherKinectToFieldRotate = new PMatrix3D(-1, 0, 0, 0,
                                                   0, 1, 0, 0,
                                                   0, 0, -1, 0,
                                                   0, 0, 0, 1);
                                                  
PMatrix3D otherKinectToFieldTranslate = new PMatrix3D(1, 0, 0, 0,
                                                      0, 1, 0, otherKinectHeight,
                                                      0, 0, 1, -otherKinectDistance,
                                                      0, 0, 0, 1);
                                                      
PMatrix3D otherKinectToFieldScaled = new PMatrix3D();

PMatrix3D fieldToCameraRotation = new PMatrix3D(1, 0, 0, 0,
                                                 0, -1, 0, 0,
                                                 0, 0, 1, 0,
                                                 0, 0, 0, 1);
                                                 
PMatrix3D fieldToCameraTranslation = new PMatrix3D(1, 0, 0, 0,
                                                   0, 1, 0, cameraHeight,
                                                   0, 0, 1, -cameraDistance,
                                                   0, 0, 0, 1);


class Pose
{
    PVector jointLeftShoulderRelative = new PVector();
    PVector jointLeftElbowRelative = new PVector();
    PVector jointLeftHandRelative = new PVector();
  
    PVector jointRightShoulderRelative = new PVector();
    PVector jointRightElbowRelative = new PVector();
    PVector jointRightHandRelative = new PVector();
}

// ==== COPY ===
// @author: LHA
// a hand pose class contains only 2 points: the left hand and the right hand 
// with absolute coordinates
class HandPoseAbsolute{
    PVector leftHandAbsolute = new PVector();
    PVector rightHandAbsolute = new PVector();
}
// ==== COPY ===

// ==== COPY ===
// @author: LHA
// a new class, used to store multiple poses in a ringbuffer data structure
class RingBufferForThrowAction{
  
  // all hand poses are located here in the ringbuffer data structure
  HandPoseAbsolute[] poseArray;
  
  // the actual pointer of the ring buffer
  private int actualPointer = 0;
  
  // constructor
  RingBufferForThrowAction(){
      
      // init
      poseArray = new HandPoseAbsolute[M];
      
      // init
      for(int i=0;i<M;i++){
          poseArray[i] = new HandPoseAbsolute();
      }
  }
  
  // METHOD
  
  // adding a new pose in the ringbuffer
  void addANewPose(HandPoseAbsolute aNewPose){
      // calculate the actual pointer
      actualPointer = (actualPointer+1)%M;
      
      // store the new hand pose in the ringbuffer 
      poseArray[actualPointer].leftHandAbsolute.x = aNewPose.leftHandAbsolute.x;
      poseArray[actualPointer].leftHandAbsolute.y = aNewPose.leftHandAbsolute.y;
      poseArray[actualPointer].leftHandAbsolute.z = aNewPose.leftHandAbsolute.z;
      
      poseArray[actualPointer].rightHandAbsolute.x = aNewPose.rightHandAbsolute.x;
      poseArray[actualPointer].rightHandAbsolute.y = aNewPose.rightHandAbsolute.y;
      poseArray[actualPointer].rightHandAbsolute.z = aNewPose.rightHandAbsolute.z;
  }
  
  // get the actual pointer
  int getActualPointer(){
      return actualPointer;
  }
  
  // get the older pose in X poses before
  HandPoseAbsolute getTheOlderPose(){
      return poseArray[(getActualPointer()-X+M)%M];
  }
}
// ==== COPY ===


//////////////////////////////////////////////////////////
// gesture recognition

// define the pose object

// @author: LHA
// save pose coordinates in the class RingBuffer

class RingBuffer
{
    Pose[] poseArray;
    int  startOfBuffer = 0;
    Float d[][][] = new Float[MAX_GESTURE_COUNT][N][M];
    int P[][][] = new int[MAX_GESTURE_COUNT][N][M];
    Float D[][] = new Float[N][M];
  
    // constructor
    RingBuffer () {
        poseArray = new Pose[M];

        for(int m = 0; m < M; m++) 
        {
            poseArray[m] = new Pose();
            for(int n = 0; n < N; n++)
            {
                for(int moveID = 0; moveID < 10; moveID++)
                {
                    d[moveID][n][m] = 0.0;
                }
            }
        }
    }
 
   // @author: a new pose will be saved in the ringbuffer
   // with the ring buffer mechanism
   // using 1 pointer: startOfBuffer. This pointer means where one should write a new pose in the ring
    void fillBuffer(Pose newPose) {
        startOfBuffer = (startOfBuffer + 1) % M;
        counter++;
    
        // copy data
        poseArray[startOfBuffer].jointLeftShoulderRelative.x = newPose.jointLeftShoulderRelative.x;
        poseArray[startOfBuffer].jointLeftShoulderRelative.y = newPose.jointLeftShoulderRelative.y;
        poseArray[startOfBuffer].jointLeftShoulderRelative.z = newPose.jointLeftShoulderRelative.z;
        
        poseArray[startOfBuffer].jointLeftElbowRelative.x = newPose.jointLeftElbowRelative.x;
        poseArray[startOfBuffer].jointLeftElbowRelative.y = newPose.jointLeftElbowRelative.y;
        poseArray[startOfBuffer].jointLeftElbowRelative.z = newPose.jointLeftElbowRelative.z;
        
        poseArray[startOfBuffer].jointLeftHandRelative.x = newPose.jointLeftHandRelative.x;
        poseArray[startOfBuffer].jointLeftHandRelative.y = newPose.jointLeftHandRelative.y;
        poseArray[startOfBuffer].jointLeftHandRelative.z = newPose.jointLeftHandRelative.z;

        poseArray[startOfBuffer].jointRightShoulderRelative.x = newPose.jointRightShoulderRelative.x;
        poseArray[startOfBuffer].jointRightShoulderRelative.y = newPose.jointRightShoulderRelative.y;
        poseArray[startOfBuffer].jointRightShoulderRelative.z = newPose.jointRightShoulderRelative.z;
                
        poseArray[startOfBuffer].jointRightElbowRelative.x = newPose.jointRightElbowRelative.x;
        poseArray[startOfBuffer].jointRightElbowRelative.y = newPose.jointRightElbowRelative.y;
        poseArray[startOfBuffer].jointRightElbowRelative.z = newPose.jointRightElbowRelative.z;        
        
        poseArray[startOfBuffer].jointRightHandRelative.x = newPose.jointRightHandRelative.x;
        poseArray[startOfBuffer].jointRightHandRelative.y = newPose.jointRightHandRelative.y;
        poseArray[startOfBuffer].jointRightHandRelative.z = newPose.jointRightHandRelative.z;
    }
  
    void copyBuffer(int which) {
        println("copy buffer!");

        for (int i=0; i<N; i++) {
            move[which][i].jointLeftShoulderRelative.x = poseArray[(startOfBuffer + i + N) % M].jointLeftShoulderRelative.x;
            move[which][i].jointLeftShoulderRelative.y = poseArray[(startOfBuffer + i + N) % M].jointLeftShoulderRelative.y;
            move[which][i].jointLeftShoulderRelative.z = poseArray[(startOfBuffer + i + N) % M].jointLeftShoulderRelative.z;            

            move[which][i].jointLeftElbowRelative.x = poseArray[(startOfBuffer + i + N) % M].jointLeftElbowRelative.x;
            move[which][i].jointLeftElbowRelative.y = poseArray[(startOfBuffer + i + N) % M].jointLeftElbowRelative.y;
            move[which][i].jointLeftElbowRelative.z = poseArray[(startOfBuffer + i + N) % M].jointLeftElbowRelative.z;

            move[which][i].jointLeftHandRelative.x = poseArray[(startOfBuffer + i + N) % M].jointLeftHandRelative.x;
            move[which][i].jointLeftHandRelative.y = poseArray[(startOfBuffer + i + N) % M].jointLeftHandRelative.y;
            move[which][i].jointLeftHandRelative.z = poseArray[(startOfBuffer + i + N) % M].jointLeftHandRelative.z;
      
            move[which][i].jointRightShoulderRelative.x = poseArray[(startOfBuffer + i + N) % M].jointRightShoulderRelative.x;
            move[which][i].jointRightShoulderRelative.y = poseArray[(startOfBuffer + i + N) % M].jointRightShoulderRelative.y;
            move[which][i].jointRightShoulderRelative.z = poseArray[(startOfBuffer + i + N) % M].jointRightShoulderRelative.z;            

            move[which][i].jointRightElbowRelative.x = poseArray[(startOfBuffer + i + N) % M].jointRightElbowRelative.x;
            move[which][i].jointRightElbowRelative.y = poseArray[(startOfBuffer + i + N) % M].jointRightElbowRelative.y;
            move[which][i].jointRightElbowRelative.z = poseArray[(startOfBuffer + i + N) % M].jointRightElbowRelative.z;            

            move[which][i].jointRightHandRelative.x = poseArray[(startOfBuffer + i + N) % M].jointRightHandRelative.x;
            move[which][i].jointRightHandRelative.y = poseArray[(startOfBuffer + i + N) % M].jointRightHandRelative.y;
            move[which][i].jointRightHandRelative.z = poseArray[(startOfBuffer + i + N) % M].jointRightHandRelative.z;
        }
    }  
  
    float cost(int moveID, int j, int i) 
    {
  // part to adjust between left and right arm
    // left arm only set value to -1.0
    // right arm only set value to 1.0
    // both ams equally set value to 0.0
    // give more weight to the right arm you could use anysthing between 0.0 and 1.0
    // give more weight to the left arm you could use anysthing between -1.0 and 0.0

    float weight_left_or_right = 0.0;


    // === COPY ===
    // @author: LHA
    // if it is the left hand throw --> concentration of the left side
    if ((moveID>=0)&&(moveID<=4)) {
      weight_left_or_right = 1.0;
    }
    // if it is the right hand throw --> concentration of the right side
    if ((moveID>=5)&&(moveID<=9)) {
      weight_left_or_right = -1.0;
    }
    // === COPY ===

    if (weight_left_or_right > 1.0) weight_left_or_right = 1.0;
    if (weight_left_or_right < -1.0) weight_left_or_right = -1.0;

    float weight_left = 1.0 - weight_left_or_right;
    float weight_right = 1.0 + weight_left_or_right;

    float mse = 0.0;

    float weight_x = 0.5  * 0.15;
    float weight_y = 0.75 * 0.15;
    float weight_z = 1.0  * 0.15;

    mse += weight_left * sqrt( weight_x * pow((move[moveID][j].jointLeftShoulderRelative.x - poseArray[i].jointLeftShoulderRelative.x), 2) 
      + weight_y * pow((move[moveID][j].jointLeftShoulderRelative.y - poseArray[i].jointLeftShoulderRelative.y), 2)
      + weight_z * pow((move[moveID][j].jointLeftShoulderRelative.z - poseArray[i].jointLeftShoulderRelative.z), 2) );
    mse += weight_left * sqrt( weight_x * pow((move[moveID][j].jointLeftElbowRelative.x - poseArray[i].jointLeftElbowRelative.x), 2) 
      + weight_y * pow((move[moveID][j].jointLeftElbowRelative.y - poseArray[i].jointLeftElbowRelative.y), 2)
      + weight_z * pow((move[moveID][j].jointLeftElbowRelative.z - poseArray[i].jointLeftElbowRelative.z), 2) );
    mse += weight_left * sqrt( weight_x * pow((move[moveID][j].jointLeftHandRelative.x - poseArray[i].jointLeftHandRelative.x), 2) 
      + weight_y * pow((move[moveID][j].jointLeftHandRelative.y - poseArray[i].jointLeftHandRelative.y), 2)
      + weight_z * pow((move[moveID][j].jointLeftHandRelative.z - poseArray[i].jointLeftHandRelative.z), 2) );
    mse += weight_right * sqrt( weight_x * pow((move[moveID][j].jointRightShoulderRelative.x - poseArray[i].jointRightShoulderRelative.x), 2) 
      + weight_y * pow((move[moveID][j].jointRightShoulderRelative.y - poseArray[i].jointRightShoulderRelative.y), 2)
      + weight_z * pow((move[moveID][j].jointRightShoulderRelative.z - poseArray[i].jointRightShoulderRelative.z), 2) );
    mse += weight_right * sqrt( weight_x * pow((move[moveID][j].jointRightElbowRelative.x - poseArray[i].jointRightElbowRelative.x), 2) 
      + weight_y * pow((move[moveID][j].jointRightElbowRelative.y - poseArray[i].jointRightElbowRelative.y), 2)
      + weight_z * pow((move[moveID][j].jointRightElbowRelative.z - poseArray[i].jointRightElbowRelative.z), 2) );
    mse += weight_right * sqrt( weight_x * pow((move[moveID][j].jointRightHandRelative.x - poseArray[i].jointRightHandRelative.x), 2) 
      + weight_y * pow((move[moveID][j].jointRightHandRelative.y - poseArray[i].jointRightHandRelative.y), 2)
      + weight_z * pow((move[moveID][j].jointRightHandRelative.z - poseArray[i].jointRightHandRelative.z), 2) );
        return mse;
    }
	
    // calculate the 'cost' of the different moves using DTW
    float pathcost(int moveID)
    {  		
        for(int n=0;n<N;n++)
	{
            d[moveID][n][(startOfBuffer + M - 1) % M] = cost( moveID, (n+N+1)% N,(0 + startOfBuffer) % M);
	}
                
        float cost = 0;
        if (counter > M+1)
        {
            D[0][0] = d[moveID][0][(startOfBuffer) % M];
	    P[moveID][0][0] = 0;
		
	    for(int n=1;n<N;n++)
	    {
                D[n][0]=d[moveID][n][(startOfBuffer) % M] + D[n-1][0];
                P[moveID][n][0] = 1;
	    }

	    for(int m=1;m<M;m++)
	    {
                D[0][m] = d[moveID][0][(m + startOfBuffer) % M];
                P[moveID][0][m] = -1;
	    }
		
	    for(int n=1;n<N;n++)
	    {
	        for(int m=1;m<M;m++)
		{
                    D[n][m] = d[moveID][n][(m + startOfBuffer) % M] + min( D[n-1][m-1], D[n][m-1], D[n][m-1] );
		}
	    }

            float countAdjust = 3.0;
            for(int n=1;n<N;n++)
	    {
	        for(int m=1;m<M;m++)
		{
                    P[moveID][n][m] = 0;
                    if (D[n][m-1] < D[n-1][m-1]) P[moveID][n][m] = -1;
                    if (D[n-1][m] < D[n-1][m-1]) 
                    {
                        P[moveID][n][m] = 1;
                        if (D[n][m-1] < D[n-1][m]) P[moveID][n][m] = -1;
                    }
                    // adjust a little here to detect faster events
                    if (P[moveID][n][m] < 0)
                    {
                        D[N-2][M-2] -= 0.01/countAdjust*(1.0-(counterEvent/25.0))*D[n][m]; 
                        countAdjust++;
                    }
		}
	    }

            int n = N-2;
            int m = M-2;   
            speed[moveID] = 0.0;
            float adjust = N;
            for (int i = 0; i < 2*M; i++) 
            {
                int tempN = n;
                if (P[moveID][n][m] >= 0) tempN--;
                if (P[moveID][n][m] <= 0) m--;
                n = tempN;  
                
                // average speed values 
                // speed[moveID] -=  m-0.5*M-n;
                
                if (n == N-4) 
                {
                    speed[moveID] = m;
                }
                        
                if (n <= 0) 
                {
                    steps[moveID] = i;
                    adjust = (((float) M)-m) / ((float) N);
                    i = 2*M;
                }
                if (m < 0) m = 0;                        
            }
            steps[moveID]++;
            speed[moveID] -= m;
            speed[moveID] /= N-4.0;
            // speed[moveID] /= (float) steps[moveID];
           
            // better results by normalizing by N instead of steps
            // cost = D[N-2][M-2]/((float) N);
            cost = D[N-2][M-2]/steps[moveID];
        }

	return cost;
    }
        
    void display() 
    {
        if (counter < M+1) return;
        float maximum = 0;
                
        noStroke(); 
        for (int n = 0; n < (N-1); n++) 
        {
            for (int m = 0; m < (M-1); m++) 
            {
                if (d[displayCost][n][m] > maximum)
                {
                    maximum  = d[displayCost][n][m];
                }
            }
        }

        for (int i = 0; i < (N-1); i++)
        {
            for (int j = 0; j < (M-1); j++)
            {
                float value = 255-255*d[displayCost][i][(j + startOfBuffer) % M]/maximum;
                fill(value);
                rect(context.depthWidth() + i*400/M,j*400/M+90,400/M,400/M);
            } 
        }
                
        int n = N-2;
        int m = M-2;   
        for (int i = 0; i <= steps[displayCost]; i++) 
        {
            float value = 255-255*d[displayCost][n][(m + startOfBuffer) % N]/maximum;
            fill(value,0,0);
            rect(context.depthWidth() + n*400/M,m*400/M+90,400/M,400/M);
                        
            int tempN = n;
            if (P[displayCost][n][m] >= 0) tempN--;
            if (P[displayCost][n][m] <= 0) m--;
            n = tempN;
            if (n < 0) n = 0;            
        }
        
        fill(0,0,0);
        rect(context.depthWidth(), 0, 195, 90);
        
        textAlign(CENTER);
        textFont(fontA32, 32);
        fill(255,255,255);
        text("analyse", context.depthWidth() + 100, 35);
        text("figure #" + displayCost, context.depthWidth() + 100, 75);
        
        // find best match
        float bestcost = cost[0][0];
        int whichcost = 0;        
        for (int i=1; i<10; i++)
        {
            if ( (cost[0][i] < bestcost) && (!empty[i]) )
            {
                bestcost = cost[0][i];
                whichcost = i;
            }
        }
        
        if ( ( cost[0][whichcost] < 0.3 ) && ( costLast[0][whichcost] >= 0.3 ) )
        {
            fill(0,0,0);
            rect(context.depthWidth() + 200, 0, 200, 90);
                
            stroke(0,0,0);
            fill(255,255,255);
            
            text("found", context.depthWidth() + 300, 35);  
            text("event #" + whichcost, context.depthWidth() + 300, 75);
            counterEvent = 25;
        }
        
        if (counterEvent < 1)
        {
            fill(0,0,0);
            rect(context.depthWidth() + 200, 0, 200, 200);
                
            stroke(0,0,0);
            fill(255,255,255);
          
            text("found", context.depthWidth() + 300, 35);
            text("no event", context.depthWidth() + 300, 75);
        }
        else
        {
            counterEvent--;
        }
        
        if ( ( costLast[0][whichcost] < 0.3 ) && ( costLast[0][whichcost] > cost[0][whichcost] ) )
        {
            fill(0,0,0);
            rect(context.depthWidth() + 200, 80, 200, 200);
          
            fill(255,255,255);
            text("motion speed:", context.depthWidth() + 300, 115); 
          
            if (speed[whichcost] < 1.0/1.5) 
            {
                text("much faster", context.depthWidth() + 300, 155);                
            }
            else if (speed[whichcost] < 1.0/1.25) 
            {
                text("faster", context.depthWidth() + 300, 155);        
            }
            if ( (speed[whichcost] >= 1.0/1.25) && (speed[whichcost] <= 1.25) )
            {
                text("similar", context.depthWidth() + 300, 155);
            }
            if (speed[whichcost] > 1.5) 
            {
                text("much slower", context.depthWidth() + 300, 155);  
            } 
            else if (speed[whichcost] > 1.25) 
            {
                text("slower", context.depthWidth() + 300, 155);
            }
            
            counterEvent = 25;
        }  
    }
}

Pose normalizePose(Pose p) {
// size normalization
    float scaleFactor = 1.0;
    float normalShoulderWidth = 370*scaleFactor;
    
    float normalLeftUpperArmLength = 320*scaleFactor;
    float normalRightUpperArmLength = 320*scaleFactor;
    
    float normalLeftLowerArmLength = 300*scaleFactor;
    float normalRightLowerArmLength = 300*scaleFactor;
// normalize shoulder width

    PVector shoulderVector = new PVector();
    
    shoulderVector.x = p.jointLeftShoulderRelative.x - p.jointRightShoulderRelative.x;
    shoulderVector.y = p.jointLeftShoulderRelative.y - p.jointRightShoulderRelative.y;
    shoulderVector.z = p.jointLeftShoulderRelative.z - p.jointRightShoulderRelative.z;

    float shoulderWidth = shoulderVector.mag();
//    println("shoulderWidth: " + shoulderWidth);
    
    float shoulderNormalizationFactor = normalShoulderWidth/shoulderWidth;
    
    p.jointLeftShoulderRelative.x *= shoulderNormalizationFactor;
    p.jointLeftShoulderRelative.y *= shoulderNormalizationFactor;
    p.jointLeftShoulderRelative.z *= shoulderNormalizationFactor;
  
    p.jointLeftElbowRelative.x *= shoulderNormalizationFactor;
    p.jointLeftElbowRelative.y *= shoulderNormalizationFactor;
    p.jointLeftElbowRelative.z *= shoulderNormalizationFactor;
  
    p.jointLeftHandRelative.x *= shoulderNormalizationFactor;
    p.jointLeftHandRelative.y *= shoulderNormalizationFactor;
    p.jointLeftHandRelative.z *= shoulderNormalizationFactor;
  
    p.jointRightShoulderRelative.x *= shoulderNormalizationFactor;
    p.jointRightShoulderRelative.y *= shoulderNormalizationFactor;
    p.jointRightShoulderRelative.z *= shoulderNormalizationFactor;
  
    p.jointRightElbowRelative.x *= shoulderNormalizationFactor;
    p.jointRightElbowRelative.y *= shoulderNormalizationFactor;
    p.jointRightElbowRelative.z *= shoulderNormalizationFactor;
  
    p.jointRightHandRelative.x *= shoulderNormalizationFactor;
    p.jointRightHandRelative.y *= shoulderNormalizationFactor;
    p.jointRightHandRelative.z *= shoulderNormalizationFactor;
    
// normalize upper arm length
    PVector leftUpperArmVector = new PVector();
    PVector rightUpperArmVector = new PVector();

    leftUpperArmVector.x = p.jointLeftElbowRelative.x - p.jointLeftShoulderRelative.x;
    leftUpperArmVector.y = p.jointLeftElbowRelative.y - p.jointLeftShoulderRelative.y;
    leftUpperArmVector.z = p.jointLeftElbowRelative.z - p.jointLeftShoulderRelative.z;

    rightUpperArmVector.x = p.jointRightElbowRelative.x - p.jointRightShoulderRelative.x;
    rightUpperArmVector.y = p.jointRightElbowRelative.y - p.jointRightShoulderRelative.y;
    rightUpperArmVector.z = p.jointRightElbowRelative.z - p.jointRightShoulderRelative.z;
    
      
//    println("left upper arm length: " + leftUpperArmVector.mag());
//    println("right upper arm length: " + rightUpperArmVector.mag());
 
    
    float leftUpperArmLength = leftUpperArmVector.mag();
    float rightUpperArmLength = rightUpperArmVector.mag();
    
    float leftUpperArmNormalizationFactor = normalLeftUpperArmLength/leftUpperArmLength;
    float rightUpperArmNormalizationFactor = normalRightUpperArmLength/rightUpperArmLength;
    
    PVector oldLeftElbow = new PVector(p.jointLeftElbowRelative.x, p.jointLeftElbowRelative.y, p.jointLeftElbowRelative.z);
    PVector oldRightElbow = new PVector(p.jointRightElbowRelative.x, p.jointRightElbowRelative.y, p.jointRightElbowRelative.z);
      
    p.jointLeftElbowRelative.mult(leftUpperArmNormalizationFactor);
    p.jointRightElbowRelative.mult(rightUpperArmNormalizationFactor);
    
    PVector leftHandMoveVector = new PVector(p.jointLeftElbowRelative.x, p.jointLeftElbowRelative.y, p.jointLeftElbowRelative.z);
    leftHandMoveVector.sub(oldLeftElbow);
    PVector rightHandMoveVector = new PVector(p.jointRightElbowRelative.x, p.jointRightElbowRelative.y, p.jointRightElbowRelative.z);
    rightHandMoveVector.sub(oldRightElbow);
    
    p.jointLeftHandRelative.add(leftHandMoveVector);
  
    p.jointRightHandRelative.add(rightHandMoveVector);
   
// normalize lower arm length
    PVector leftLowerArmVector = new PVector();
    PVector rightLowerArmVector = new PVector();

    leftLowerArmVector.x = p.jointLeftElbowRelative.x - p.jointLeftHandRelative.x;
    leftLowerArmVector.y = p.jointLeftElbowRelative.y - p.jointLeftHandRelative.y;
    leftLowerArmVector.z = p.jointLeftElbowRelative.z - p.jointLeftHandRelative.z;

    rightLowerArmVector.x = p.jointRightElbowRelative.x - p.jointRightHandRelative.x;
    rightLowerArmVector.y = p.jointRightElbowRelative.y - p.jointRightHandRelative.y;
    rightLowerArmVector.z = p.jointRightElbowRelative.z - p.jointRightHandRelative.z;
    
//    println("left lower arm length: " + leftLowerArmVector.mag());
//    println("right lower arm length: " + rightLowerArmVector.mag());
    
    float leftLowerArmLength = leftLowerArmVector.mag();
    float rightLowerArmLength = rightLowerArmVector.mag();
    
    float leftLowerArmNormalizationFactor = normalLeftLowerArmLength/leftLowerArmLength;
    float rightLowerArmNormalizationFactor = normalRightLowerArmLength/rightLowerArmLength;
    
    leftLowerArmVector.mult(leftLowerArmNormalizationFactor);
    rightLowerArmVector.mult(rightLowerArmNormalizationFactor);
    
    PVector newLeftHandPosition = new PVector(p.jointLeftElbowRelative.x, p.jointLeftElbowRelative.y, p.jointLeftElbowRelative.z);   
    PVector newRightHandPosition = new PVector(p.jointRightElbowRelative.x, p.jointRightElbowRelative.y, p.jointRightElbowRelative.z);   
    
    newLeftHandPosition.sub(leftLowerArmVector);
    newRightHandPosition.sub(rightLowerArmVector);
    
    p.jointLeftHandRelative = newLeftHandPosition;
    p.jointRightHandRelative = newRightHandPosition;
    
    return p;
}

PImage snowball;

/* =====================================================================================
    setup
   ===================================================================================== */
void setup()
{
    // context = new SimpleOpenNI(this);
    context = new SimpleOpenNI(this,SimpleOpenNI.RUN_MODE_MULTI_THREADED);
  
    int portToListenTo = 7000; 
    int portToSendTo = 7000;
    String ipAddressToSendTo = "localhost";

    oscP5 = new OscP5(this,portToListenTo);
    myRemoteLocation = new NetAddress(ipAddressToSendTo, portToSendTo);  

    //Q: are there M gestures with N pose-frames??
    grid = new Pose[M][N];
    for (int i = 0; i < M; i++) {
      for (int j = 0; j < N; j++) {
        grid[i][j] = new Pose();
      }
    }

    // ten poses with N pose-frames
  //  pose = new Pose();//[10];
    move = new Pose[MAX_GESTURE_COUNT][N];

    // @author: LHA
    // RingBuffer[2] means, max. 2 persons can be tracked and all coordinate are saved in poseArray
    ringbuffer = new RingBuffer[PLAYER_COUNT];
    for (int i = 0; i < PLAYER_COUNT; i++) {
        ringbuffer[i] = new RingBuffer();
    }
    
    // ==== COPY ===
    // absolute coordinate for hands
    ringbufferHand = new RingBufferForThrowAction[PLAYER_COUNT];
    for (int i = 0; i < PLAYER_COUNT; i++) {
        ringbufferHand[i] = new RingBufferForThrowAction();
    }
    // ==== COPY ===
    
    
    data = new Data();
  
    for(int i = 0; i < MAX_GESTURE_COUNT; i++) {
        for(int j = 0; j < N; j++) move[i][j] = new Pose();
    }

    foto = new PImage[MAX_GESTURE_COUNT];
  
    // load the stored data
    for (int i = 0; i < MAX_GESTURE_COUNT; i++) {
        String str = Integer.toString(i);          
        empty[i] = false;
        
        File f = new File(dataPath("pose" + str + ".png"));
        if (!f.exists()) {
            foto[i] = loadImage(dataPath("empty.png"));  
            println("File " + dataPath("pose" + str + ".png") + " does not exist");
            empty[i] = true;
        } else { 
            foto[i] = loadImage(dataPath("pose" + str + ".png"));  
        }
      
        f = new File(dataPath("pose" + str + ".data"));      
        if (!f.exists()) {
            println("File " + dataPath("pose" + str + ".data") + " does not exist");
            for (int p=0; p<2; p++)
            {
                cost[0][i] = 10000.0;
            }
        } else { 
            loadData(i);
        }
    }

    warnings = new PImage[PLAYER_COUNT][8];

    for (int i=0; i < PLAYER_COUNT; i++)
    {

      warnings[i][0] = loadImage(dataPath("go_left_red.png"));    
      warnings[i][1] = loadImage(dataPath("go_lf_red.png"));
      warnings[i][2] = loadImage(dataPath("go_front_red.png"));
      warnings[i][3] = loadImage(dataPath("go_rf_red.png"));
      warnings[i][4] = loadImage(dataPath("go_right_red.png"));
      warnings[i][5] = loadImage(dataPath("go_rb_red.png"));
      warnings[i][6] = loadImage(dataPath("go_back_red.png"));
      warnings[i][7] = loadImage(dataPath("go_lb_red.png"));

      warning[i] = -1;
      
      PLAYER_COLORS[i] = new PVector();
      switch(i % 5){
        case 0: PLAYER_COLORS[i] = new PVector(255,0,0); break;
        case 1: PLAYER_COLORS[i] = new PVector(0,255,0); break;
        case 2: PLAYER_COLORS[i] = new PVector(0,0,255); break;
        case 3: PLAYER_COLORS[i] = new PVector(255,0,255); break;
        case 4: PLAYER_COLORS[i] = new PVector(255,255,0); break;
        default: PLAYER_COLORS[i] = new PVector(128,128,128); break;
        
      }
    }

   
    /*
    warnings[1][0] = loadImage(dataPath("go_left_blue.png"));    
    warnings[1][1] = loadImage(dataPath("go_lf_blue.png"));
    warnings[1][2] = loadImage(dataPath("go_front_blue.png"));
    warnings[1][3] = loadImage(dataPath("go_rf_blue.png"));
    warnings[1][4] = loadImage(dataPath("go_right_blue.png"));
    warnings[1][5] = loadImage(dataPath("go_rb_blue.png"));
    warnings[1][6] = loadImage(dataPath("go_back_blue.png"));
    warnings[1][7] = loadImage(dataPath("go_lb_blue.png"));
    */

    shapeOfUser = loadImage(dataPath("shape.png"));  
    kineticspace = loadImage(dataPath("kinetic_space.png"));
  
    // enable depthMap generation 
    context.enableDepth();
  
    // enable skeleton for particular joints
    // context.enableUser(SimpleOpenNI.SKEL_PROFILE_ALL);
    context.enableUser(SimpleOpenNI.SKEL_PROFILE_UPPER);
   
    fontA32 = createFont("Arial", 32);
    fontA12 = createFont("Arial", 16);

    // Set the font and its size (in units of pixels)
    textFont(fontA32, 32);
 
    //TODO: ui stuff
    if (showDebugUI)
    {
      background(0,0,0);
      stroke(0,0,255);
      strokeWeight(3);
      smooth();
      pg = createGraphics(context.depthWidth(), context.depthHeight(), P2D);
    }
    else {
      // we are in game mode
      
      if (!useFullscreen)
      {
        // size(1070, 850, OPENGL); 
        // size(1070, 850); 
        size(1170, 800, P3D);
      }
      else
      {
        size(1280, 800, P3D);
        // Create the fullscreen object
        // fs = new FullScreen(this); 
  
        // enter fullscreen mode
        // fs.enter(); 
      }
      
      
      kinectToFieldScaled.set(kinectToField);
      kinectToFieldScaled.apply(kinectScale);

      otherKinectToFieldScaled.set(otherKinectToFieldTranslate);
      otherKinectToFieldScaled.apply(otherKinectToFieldRotate);
      otherKinectToFieldScaled.apply(kinectScale);
    
      snowball = loadImage("Schneeball.png");
      bg = loadImage("Background.png");
      bg.resize(width, height);
    
      for(int i=0; i<PLAYER_COUNT; i++) 
      {
         for(int j=0; j<PLAYER_MODUS_COUNT;j++) 
         {
           String imageName = "held" + i + j + ".png";
           playas[i][j] = loadImage(imageName);
         
         }
      }
  }
    
}

/* incoming osc message are forwarded to the oscEvent method. */
void oscEvent(OscMessage updateMessage) {
  /* print the address pattern and the typetag of the received OscMessage */
//  print("### received an osc message.");
//  print(" addrpattern: "+updateMessage.addrPattern());
  
  if (!updateMessage.addrPattern().equals("/update"))
  {
    return;
  }

   for(int i = 0; i < PLAYER_COUNT; i++) {
     
      playerNecks[i] = new PVector();
      playerNecks[i].x = updateMessage.get(i*9+0).floatValue();
      playerNecks[i].y = updateMessage.get(i*9+1).floatValue();
      playerNecks[i].z = updateMessage.get(i*9+2).floatValue();


      playerOldHands[i] = new PVector();
      playerOldHands[i].x = updateMessage.get(i*9+3).floatValue();
      playerOldHands[i].y = updateMessage.get(i*9+4).floatValue();
      playerOldHands[i].z = updateMessage.get(i*9+5).floatValue();


      playerActualHands[i] = new PVector();
      playerActualHands[i].x = updateMessage.get(i*9+6).floatValue();
      playerActualHands[i].y = updateMessage.get(i*9+7).floatValue();
      playerActualHands[i].z = updateMessage.get(i*9+8).floatValue();
     
   }

   for(int i = 0; i < PLAYER_COUNT; i++) {
     
      if(playerNecks[i].x != 0) {
       
        println("player " + i);
        println("\tneck: ");
        println("\t" + playerNecks[i]);
        
        println("\told hand");
        println("\t" + playerOldHands[i]);
        
        println("\tactual hand");
        println("\t" + playerActualHands[i]);

      } else {
        //println("no information");
      }

     
   }
 
}

class Ball
{
  PVector pos;
  PVector dir;
}

ArrayList<Ball> balls = new ArrayList<Ball>();

void updateBalls()
{
  for (Ball b : balls)
  {
    b.pos.add(b.dir);
   
    line(b.pos.x, b.pos.y, b.pos.z, b.pos.x, 0, b.pos.z);
    
    pushMatrix();
    translate(b.pos.x, b.pos.y, b.pos.z);
    textureMode(NORMALIZED);
    beginShape();
    texture(snowball);
    vertex(-0.05f, -0.05f, 0f, 0, 0);
    vertex(0.05f, -0.05f, 0f, 1, 0);
    vertex(0.05f, 0.05f, 0f, 1, 1);
    vertex(-0.05f, 0.05f, 0f, 0, 1);
    endShape();
    popMatrix();
  }

  for (int i = 0; i < balls.size(); ++i)
  {
    if (isBrokenBall(balls.get(i)))
    {
      balls.remove(i);
      --i;
    }
  }
}

boolean isBrokenBall(Ball ball)
{
    return (ball.pos.mag() > 10 ||
            ball.pos.y <= 0 ||
            ball.pos.y >= 3);
}

// game ui
float screenWidthMeters = 4;

void drawGameField()
{
    // draw spielfeld 
    //background(255,255,255);

    pushMatrix();
    
    //TODO: ui man kann den ursprung in processing bei setup festlegen 
    translate(width/2, height/2);
		


    float aspect = (float)width/(float)height;
    float screenHeightMeters = screenWidthMeters / aspect;
    float metersToPixel = width/screenWidthMeters;
		
    scale(metersToPixel);
		
  
    frustum((float)screenWidthMeters/2, -(float)screenWidthMeters/2, -(float)screenHeightMeters/2, (float)screenHeightMeters/2, 3.1f, 15f);
		
		
				
//    applyMatrix(1,0,0,0,
//                0, -1, 0, 0,
//		0,0,1,0,
//		0,0,0,1);
//    translate(0, -cameraHeight, -cameraDist);
    applyMatrix(fieldToCameraTranslation);
    applyMatrix(fieldToCameraRotation);

    /* Weltkoordinaten */
		
    line(-2, 0, 0, 2, 0, 0);
    line(-2, 0, 4, -2, 0, -4);
    line(2, 0, 4, 2, 0, -4);
		
               
 		
		
   
    pushMatrix();
    applyMatrix(kinectToField);
		/* Kinectkoordinaten */
 
     //box(0.3f, 0.05f, 0.07f);

    popMatrix();
}

void drawPlayer(PVector neck, boolean transparent)
{
if (false)
{
  line(neck.x, 0, neck.z, neck.x, neck.y, neck.z);
 
             pushMatrix();
             translate(neck.x, neck.y, neck.z);
             box(0.1f,0.1f,0.1f);
             popMatrix();
}            
             pushMatrix();
             float aspect = (float)playas[0][0].width / (float)playas[0][0].height;
             float heldHeight = neck.y + 0.3f;
             float heldWidth = heldHeight * aspect;
             translate(neck.x, 0, neck.z);
             textureMode(NORMALIZED);
             beginShape();
             if (transparent)
             {
               tint(255, 128);
             }
             texture(playas[0][0]);
             vertex(-heldWidth/2, heldHeight, 0, 0, 0);
             vertex(heldWidth/2, heldHeight, 0, 1, 0);
             vertex(heldWidth/2, 0, 0, 1, 1);
             vertex(-heldWidth/2, 0, 0, 0, 1);
             endShape();
             
             noTint();
             popMatrix();
}

boolean vectorNullOrZero(PVector vec)
{
  return (vec == null) || (vec.x == 0 && vec.y == 0 && vec.z == 0);
}

void createBall(PVector throwStartPos, PVector throwEndPos, PMatrix3D transform)
{
                    Ball ball = new Ball();
                  PVector fieldThrowEndPos = new PVector();
                  transform.mult(throwEndPos, fieldThrowEndPos);
                  
                  PVector fieldThrowStartPos = new PVector();
                  transform.mult(throwStartPos, fieldThrowStartPos);
                 
                  ball.pos = fieldThrowEndPos;
                         
                  PVector dir = PVector.sub(fieldThrowEndPos, fieldThrowStartPos);
                  dir.y = 0;
                  
                  dir.normalize();
                  dir.mult(0.1f);
                  
                  ball.dir = dir; 
                
                  balls.add(ball);    
}

void draw()
{
  strokeWeight(0);
  background(bg); 

  // update the cam
    context.update();
  
    pg.beginDraw();

    //TODO: ui stuff

    if (showDebugUI)  
    {
      // draw depthImageMap
      pg.image(context.depthImage(),0,0);

      ringbuffer[0].display();
    }
    
    // process the skeleton if it's available
    foundSkeleton = false;
  int  person = 0;

    if (!showDebugUI)
    { 
      drawGameField();
    }
  
    for (int i = 0; i < playerNecks.length; ++i)
    {
      if (!showDebugUI)
      {
        if(!vectorNullOrZero(playerNecks[i]))      
        {
          PVector neck = new PVector();
          otherKinectToFieldScaled.mult(playerNecks[i], neck);
          drawPlayer(neck, false);      
        }
      }

      if (!vectorNullOrZero(playerActualHands[i]) &&
          !vectorNullOrZero(playerOldHands[i]))
      {
        createBall(playerOldHands[i], playerActualHands[i], otherKinectToFieldScaled);
        playerActualHands[i] = null;
        playerOldHands[i] = null;
      }
    }
    
    for (int i = 1; i< CHECK_SKELETON_COUNT; i++)
    {
          //TODO more than 2 players
  
        if ( (context.isTrackingSkeleton(i)) && (person < PLAYER_COUNT) )
        {
            
            PVector neck = evaluateSkeleton(i,person);
            
            //println(neck);
           
            if (!showDebugUI) 
            {
              
              kinectToFieldScaled.mult(neck, neck);
                
              //translate(neck.x * 0.001f, neck.y * 0.001f, -neck.z * 0.001f);
	      //TODO: ui stuff
              drawPlayer(neck, true);
         
                
           }
              //TODO: ball erzeugen nur bei abwurf
           if (ballThrow)
           {
               createBall(throwStartPos, throwEndPos, kinectToFieldScaled);           
               ballThrow = false;       
           }
           foundSkeleton = true;
           person++;
           
        }
    }
    if (!showDebugUI) 
    {
      updateBalls();
       
      counter2++;
      counter2 %= 25;
      if (counter2 == 0)
      {
        /* send OSC message */
        OscMessage myMessage = new OscMessage("/status");
        if (foundSkeleton) myMessage.add("tracking ...");
        if (!foundSkeleton) myMessage.add("looking for pose ...");
        oscP5.send(myMessage, myRemoteLocation); 
      }
    }
    pg.endDraw();
    
//    popMatrix();

    if (showDebugUI)
    {

      if (switchDisplay)
      {
        image(pg, 0, 0);
      }
      else
      {
        pushMatrix();
        scale(-1.0, 1.0);
        image(pg,-pg.width,0);
        popMatrix();
      }
  
      if (!foundSkeleton) 
      {
        stroke(0,0,0);
        fill(0,0,0);
        rect(context.depthWidth(), 0, 400, 485);
            
        stroke(0,0,255);
        fill(255,0,0);
        textFont(fontA32, 32);
        textAlign(CENTER);
        text("Please register user!", context.depthWidth() / 2, 40);
        image(shapeOfUser, 0, 0);
      }
      //TODO more than 2 players
      else 
      {
        for (int i=0; i < PLAYER_COUNT; i++)
        {
          if ((warning[i] >= 0) )
          {
            image(warnings[i][warning[i]],context.depthWidth()/2-100, context.depthHeight()/2 - 50);
          }
          
        }
      }

      if (updateDisplay)
      {
          updateDisplay = false;
        
          if (switchDisplay)
          {
              for (int i = 0; i<=4; i++)
              {
                  image(foto[i], i * (context.depthWidth() + 400) / 5 + i*5, context.depthHeight() + 15, (context.depthWidth() + 400) / 5, 130);
                  image(foto[(i+5)], i * (context.depthWidth() + 400) / 5 + i*5, context.depthHeight() + 170, (context.depthWidth() + 400) / 5, 130);
              }
          }
          else
          {
              for (int i = 0; i<=4; i++)
              {
                  pushMatrix();
                  scale(-1.0, 1.0);
                  image(foto[i], -(i+1) * (context.depthWidth() + 400) / 5 - i*5, context.depthHeight() + 15, (context.depthWidth() + 400) / 5, 130);
                  image(foto[(i+5)], -(i+1) * (context.depthWidth() + 400) / 5 - i*5, context.depthHeight() + 170, (context.depthWidth() + 400) / 5, 130);
                  popMatrix();
              }
          }
          
          for (int i = 0; i<=4; i++)
          {
              if (empty[i])
              {
                  pushMatrix();
                  scale(-1.0, 1.0);
                  image(foto[i], -(i+1) * (context.depthWidth() + 400) / 5 - i*5, context.depthHeight() + 15, (context.depthWidth() + 400) / 5, 130);
                  popMatrix();
              }
              if (empty[i+5])
              {
                  pushMatrix();
                  scale(-1.0, 1.0);
                  image(foto[(i+5)], -(i+1) * (context.depthWidth() + 400) / 5 - i*5, context.depthHeight() + 170, (context.depthWidth() + 400) / 5, 130);
                  popMatrix();
              }            
          }
  
          textFont(fontA12, 16);
          fill(255,255,255);
          textAlign(CENTER);
          for (int i = 0; i<=4; i++)
          {
              text(i, i * (context.depthWidth() + 400) / 5 + i*5 + 12, context.depthHeight() + 32);
              // image(foto[i], i * (context.depthWidth() + 400) / 5 + i*5, context.depthHeight() + 15, (context.depthWidth() + 400) / 5, 130);
              text((i+5), i * (context.depthWidth() + 400) / 5 + i*5 + 12, context.depthHeight() + 187);
              // image(foto[(i+5)], i * (context.depthWidth() + 400) / 5 + i*5, context.depthHeight() + 170, (context.depthWidth() + 400) / 5, 130);
          }                
      }
    }
    
    // evaluate and draw DTW
    if (foundSkeleton)
    {
     	//TODO:ui stuff
        if (showDebugUI)
        {
          noStroke();             
          fill(0,0,0);
          rect(0, context.depthHeight() + 145, 1070, 20);
          rect(0, context.depthHeight() + 300, 1070, 20);
        }
        
        // current person count
        for (int p = 0; p<person; p++)
        {
            for (int i = 0; i <= 9; i++)
            {
                if (!empty[i])
                {
                    costLast[p][i] = cost[p][i];
                    cost[p][i] = ringbuffer[p].pathcost(i);
                    cost[p][i] = (log(cost[p][i]-1.0) - 5.5)/2.0;
                    // println("cost(" + i + "): " + cost);
                    
                    if (showDebugUI) 
                    {            
                      fill(255,0,0);
                      if (p == 1) fill(0,0,255);
                      if ( cost[p][i] <= 0.25 )
                      {
                          fill(0,255,0);
                      }
            
                    
                      if ( ( cost[p][i] > 0.25 ) && ( cost[p][i] < 0.35 ) )
                      {
                        float normalized = 10.0 * (cost[p][i] - 0.25);
                        fill(255 * normalized,255 * (1.0-normalized),0);
                        if (p == 1) fill(0,255 * (1.0-normalized),255 * normalized); 
                      }
                                    
                      if (i < 5) rect(i * (context.depthWidth() + 400) / 5 + i*5, context.depthHeight() + 145 + 10*p, min(1.0, max(0.01, 1.0-cost[p][i])) * ((context.depthWidth() + 400) / 5), 10);
                      if (i >= 5) rect((i-5) * (context.depthWidth() + 400) / 5 + (i-5)*5, context.depthHeight() + 300 + 10*p, min(1.0, max(0.01, 1.0-cost[p][i])) * ((context.depthWidth() + 400) / 5), 10);
                    }
                    
                    if ( ( cost[p][i] < 0.3 ) && ( costLast[p][i] >= 0.3 ) )
                    {
                      ballThrow = true;
                    
                      println("Throw!");
                  
                    
                      PVector lHA = actualHandPose.leftHandAbsolute;
                      throwEndPos = new PVector(lHA.x, lHA.y, lHA.z);
                      PVector olHA = oldHandPose.leftHandAbsolute;
                      throwStartPos = new PVector(olHA.x, olHA.y, olHA.z);
                                        
                    }
                }
            }
        }
    }
}

// all the poses will be rotatet. need neck-relative position (as origin)
void normalizePoseRotation(Pose pose) {

  // get vector between shoulders and computer the normal in the middle of the [strecke]
  // only 2d vector, as angle between only computes one angle (x,y component)
  PVector leftToRight = new PVector();
  leftToRight.x = pose.jointRightShoulderRelative.x - pose.jointLeftShoulderRelative.x;
  leftToRight.y = pose.jointRightShoulderRelative.z - pose.jointLeftShoulderRelative.z;

  // normalize
  //leftToRight.normalize();
  // the orientation in the view from the kinect sensor
  PVector facingV = new PVector(1, 0); //use the normal to the z-direction (facing of the k.sensor) //0,1);
  // 0 -> front face to sensor face
  // 90 -> turned front to right
  // -90 -> turned front to left
  float fradians = PVector.angleBetween(leftToRight, facingV);
  float angle = degrees( fradians );

  if (leftToRight.y > 0){
     //angle = -angle;
     //fradians = -fradians;
  }    
  // TODO compute back-facing vector (test sign )
  // negative x is with face to the kinect device,   
  println(" shoulders vektor "+leftToRight + " angle "+angle + " rads "+ fradians); 



  // rotate all bones by this angle, so that the recognisable     
  float fcos = cos(-fradians);
  float fsin = sin(-fradians);

  println(" left shoulder before " +   pose.jointLeftShoulderRelative.x + " : " +   pose.jointLeftShoulderRelative.z+ "; "+
                                  pose.jointRightShoulderRelative.x    + " : " +   pose.jointRightShoulderRelative.z + "; ");
                                  
                                  
  PVector leftSR = new PVector();
  leftSR.x = fcos *     pose.jointLeftShoulderRelative.x  - fsin *     pose.jointLeftShoulderRelative.z;
  leftSR.z = fsin *     pose.jointLeftShoulderRelative.x  + fcos *     pose.jointLeftShoulderRelative.z;

  PVector leftER = new PVector();
  leftER.x = fcos * pose.jointLeftElbowRelative.x  - fsin * pose.jointLeftElbowRelative.z;
  leftER.z = fsin * pose.jointLeftElbowRelative.x  + fcos * pose.jointLeftElbowRelative.z;

  PVector leftHR = new PVector();
  leftHR.x = fcos * pose.jointLeftHandRelative.x  - fsin * pose.jointLeftHandRelative.z;
  leftHR.z = fsin * pose.jointLeftHandRelative.x  + fcos * pose.jointLeftHandRelative.z;

  PVector rightSR = new PVector();
  rightSR.x = fcos * pose.jointRightShoulderRelative.x  - fsin * pose.jointRightShoulderRelative.z;
  rightSR.z = fsin * pose.jointRightShoulderRelative.x  + fcos * pose.jointRightShoulderRelative.z;

  PVector rightER = new PVector();
  rightER.x = fcos * pose.jointRightElbowRelative.x  - fsin * pose.jointRightElbowRelative.z;
  rightER.z = fsin * pose.jointRightElbowRelative.x  + fcos * pose.jointRightElbowRelative.z;

  PVector rightHR = new PVector();
  rightHR.x = fcos * pose.jointRightHandRelative.x  - fsin * pose.jointRightHandRelative.z;
  rightHR.z = fsin * pose.jointRightHandRelative.x  + fcos * pose.jointRightHandRelative.z;


  println(" left shoulder after " +  leftSR.x + " : " +   leftSR.z+ "; "+
                                  rightSR.x    + " : " +   rightSR.z + "; ");

  pose.jointLeftShoulderRelative.x = leftSR.x;
  pose.jointLeftShoulderRelative.z = leftSR.z;
  
  pose.jointLeftElbowRelative.x = leftER.x;
  pose.jointLeftElbowRelative.z = leftER.z;

  pose.jointLeftHandRelative.x = leftHR.x;
  pose.jointLeftHandRelative.z = leftHR.z;
  
  pose.jointRightShoulderRelative.x = rightSR.x;
  pose.jointRightShoulderRelative.z = rightSR.z;
  
  pose.jointRightElbowRelative.x = rightER.x;
  pose.jointRightElbowRelative.z = rightER.z;
  
  pose.jointRightHandRelative.x = rightHR.x;
  pose.jointRightHandRelative.z = rightHR.z;
  
  
}

// draw the skeleton with the selected joints
PVector evaluateSkeleton(int userId, int person)
{
    Pose pose = new Pose();

	// @author: LHA
	// the draw functionality is called frequently
    PVector jointNeck3D = new PVector();
  
    PVector jointLeftShoulder3D = new PVector();
    PVector jointLeftElbow3D = new PVector();
    PVector jointLeftHand3D = new PVector();
  
    PVector jointRightShoulder3D = new PVector();
    PVector jointRightElbow3D = new PVector();
    PVector jointRightHand3D = new PVector();
  
    PVector jointNeck2D = new PVector();  
  
    PVector jointLeftShoulder2D = new PVector();
    PVector jointLeftElbow2D = new PVector();
    PVector jointLeftHand2D = new PVector();
  
    PVector jointRightShoulder2D = new PVector();
    PVector jointRightElbow2D = new PVector();
    PVector jointRightHand2D = new PVector();

    // get the joint positions
    context.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_NECK,jointNeck3D);  
  
    context.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_LEFT_SHOULDER,jointLeftShoulder3D);
    context.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_LEFT_ELBOW,jointLeftElbow3D);
    context.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_LEFT_HAND,jointLeftHand3D);

    context.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_RIGHT_SHOULDER,jointRightShoulder3D);
    context.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_RIGHT_ELBOW,jointRightElbow3D);
    context.getJointPositionSkeleton(userId,SimpleOpenNI.SKEL_RIGHT_HAND,jointRightHand3D);
  
    context.convertRealWorldToProjective(jointNeck3D, jointNeck2D);
  
    context.convertRealWorldToProjective(jointLeftShoulder3D, jointLeftShoulder2D);
    context.convertRealWorldToProjective(jointLeftElbow3D, jointLeftElbow2D);
    context.convertRealWorldToProjective(jointLeftHand3D, jointLeftHand2D);
  
    context.convertRealWorldToProjective(jointRightShoulder3D, jointRightShoulder2D);
    context.convertRealWorldToProjective(jointRightElbow3D, jointRightElbow2D);
    context.convertRealWorldToProjective(jointRightHand3D, jointRightHand2D);

    // calculate relative position  
    pose.jointLeftShoulderRelative.x = jointLeftShoulder3D.x - jointNeck3D.x;
    pose.jointLeftShoulderRelative.y = jointLeftShoulder3D.y - jointNeck3D.y;
    pose.jointLeftShoulderRelative.z = jointLeftShoulder3D.z - jointNeck3D.z;
  
    pose.jointLeftElbowRelative.x = jointLeftElbow3D.x - jointNeck3D.x;
    pose.jointLeftElbowRelative.y = jointLeftElbow3D.y - jointNeck3D.y;
    pose.jointLeftElbowRelative.z = jointLeftElbow3D.z - jointNeck3D.z;
  
    pose.jointLeftHandRelative.x = jointLeftHand3D.x - jointNeck3D.x;
    pose.jointLeftHandRelative.y = jointLeftHand3D.y - jointNeck3D.y;
    pose.jointLeftHandRelative.z = jointLeftHand3D.z - jointNeck3D.z;
  
    pose.jointRightShoulderRelative.x = jointRightShoulder3D.x - jointNeck3D.x;
    pose.jointRightShoulderRelative.y = jointRightShoulder3D.y - jointNeck3D.y;
    pose.jointRightShoulderRelative.z = jointRightShoulder3D.z - jointNeck3D.z;
  
    pose.jointRightElbowRelative.x = jointRightElbow3D.x - jointNeck3D.x;
    pose.jointRightElbowRelative.y = jointRightElbow3D.y - jointNeck3D.y;
    pose.jointRightElbowRelative.z = jointRightElbow3D.z - jointNeck3D.z;
  
    pose.jointRightHandRelative.x = jointRightHand3D.x - jointNeck3D.x;
    pose.jointRightHandRelative.y = jointRightHand3D.y - jointNeck3D.y;
    pose.jointRightHandRelative.z = jointRightHand3D.z - jointNeck3D.z;

    
    //TODO: more than 2 players, different colors
    if (person < PLAYER_COLORS.length) pg.stroke(PLAYER_COLORS[person].x,PLAYER_COLORS[person].y,PLAYER_COLORS[person].z,255);
    
    warning[person] = -1; 
  
	//TODO: ui stuff
    if (showDebugUI)
    {
      pg.strokeWeight(5);
      pg.line(jointNeck2D.x,jointNeck2D.y, jointLeftShoulder2D.x,jointLeftShoulder2D.y);
      pg.line(jointLeftShoulder2D.x,jointLeftShoulder2D.y, jointLeftElbow2D.x,jointLeftElbow2D.y);
      pg.line(jointLeftElbow2D.x,jointLeftElbow2D.y, jointLeftHand2D.x,jointLeftHand2D.y);  
      pg.line(jointNeck2D.x,jointNeck2D.y, jointRightShoulder2D.x,jointRightShoulder2D.y);
      pg.line(jointRightShoulder2D.x,jointRightShoulder2D.y, jointRightElbow2D.x,jointRightElbow2D.y);
      pg.line(jointRightElbow2D.x,jointRightElbow2D.y, jointRightHand2D.x,jointRightHand2D.y);

      
      textAlign(CENTER);
      textFont(fontA32, 32);
      fill(255,0,0);


      // TODO: ui: show the warnings in game mode ?!?
      if (jointNeck2D.x < 100) 
      {
          warning[person] = 0;
      }
    
      if (jointNeck2D.x >  540) 
      {
          warning[person] = 4;
      }
    
      if (jointNeck3D.z > 4000)
      {
          warning[person] = 2;
        
          if (jointNeck2D.x < 100) 
          {
              warning[person] = 1;
          }
        
          if (jointNeck2D.x > 540) 
          {
              warning[person] = 3;
          }
      }
    
      if (jointNeck2D.z <  1500)
      {
          warning[person] = 6;
        
          if (jointNeck2D.x < 100) 
          {
              warning[person] = 7;
          }
        
          if (jointNeck2D.x > 540) 
          {
              warning[person] = 5;
          }
      }
    }
    
    // ==== COPY ===
    // LHA
    // the actual handPose    
    actualHandPose.leftHandAbsolute.x = jointLeftHand3D.x;
    actualHandPose.leftHandAbsolute.y = jointLeftHand3D.y;
    actualHandPose.leftHandAbsolute.z = jointLeftHand3D.z;
    
    actualHandPose.rightHandAbsolute.x = jointRightHand3D.x;
    actualHandPose.rightHandAbsolute.y = jointRightHand3D.y;
    actualHandPose.rightHandAbsolute.z = jointRightHand3D.z;

    // add the new hand pose to the ringbuffer for hand poses
    ringbufferHand[person].addANewPose(actualHandPose);
    
    // the older pose
    oldHandPose = ringbufferHand[person].getTheOlderPose();
    
    if(oldHandPose == null) println("NO history");
    // ==== COPY ===
    
    // add new pose to ringbuffer
    pose = normalizePose(pose);
	if (ROTATE_PLAYER) normalizePoseRotation(pose);    
    
	ringbuffer[person].fillBuffer( pose );
    
    // the distance is called only if the throw gesture
    if(foundSkeleton){
    //  println("DISTANCE: ");
    //  println(dist(actualHandPose.leftHandAbsolute.x,
//            actualHandPose.leftHandAbsolute.y,
//            actualHandPose.leftHandAbsolute.z,
//            oldHandPose.leftHandAbsolute.x,
//            oldHandPose.leftHandAbsolute.y,
//            oldHandPose.leftHandAbsolute.z));

    }
    
    return jointNeck3D;
}

// -----------------------------------------------------------------
// SimpleOpenNI events

void onNewUser(int userId)
{
    println("onNewUser - userId: " + userId);
    println("  start pose detection");
  
    if (!autoPoseDetection)
    {
      context.startPoseDetection("Psi",userId);
    }
    else
    {  
      if(context.loadCalibrationDataSkeleton(userId,"calibration.skel"))
      {
        context.startTrackingSkeleton(userId);
        println("Load calibration from file.");
      }
      else
        println("Can't load calibration file.");
    }    
}

void onLostUser(int userId)
{
    println("onLostUser - userId: " + userId);
    context.stopTrackingSkeleton(userId);
}

void onStartCalibration(int userId)
{
    println("onStartCalibration - userId: " + userId);
}

void onEndCalibration(int userId, boolean successfull)
{
    println("onEndCalibration - userId: " + userId + ", successfull: " + successfull);
  
    if (successfull) 
    { 
        println("  User calibrated !!!");
        context.startTrackingSkeleton(userId); 
    } 
    else 
    { 
        println("  Failed to calibrate user !!!");
        println("  Start pose detection");
        context.startPoseDetection("Psi",userId);
    }
}

void onStartPose(String pose,int userId)
{
    println("onStartPose - userId: " + userId + ", pose: " + pose);
    println(" stop pose detection");
  
    context.stopPoseDetection(userId); 
    context.requestCalibrationSkeleton(userId, true);
}

void onEndPose(String pose,int userId)
{
    println("onEndPose - userId: " + userId + ", pose: " + pose);
}


class Data {
    ArrayList datalist;
    String filename,data[];
    int datalineId;
 
    // begin data saving
    void beginSave() {
        datalist=new ArrayList();
    }
 
    void add(String s) {
        datalist.add(s);
    }
 
    void add(float val) {
        datalist.add(""+val);
    }
 
    void add(int val) {
        datalist.add(""+val);
    }
 
    void add(boolean val) {
        datalist.add(""+val);
    }
 
    void endSave(String _filename) {
        filename=_filename;
 
        data=new String[datalist.size()];
        data=(String [])datalist.toArray(data);
     
        saveStrings(filename, data);
        println("Saved data to '"+filename+"', "+data.length+" lines.");
    }
 
    void load(String _filename) {
        filename=_filename;
     
        datalineId=0;
        data=loadStrings(filename);
        println("Loaded data from '"+filename+"', "+data.length+" lines.");
    }
 
    float readFloat() {
        return float(data[datalineId++]);
    }
 
    int readInt() {
      return int(data[datalineId++]);
    }
 
    boolean readBoolean() {
        return boolean(data[datalineId++]);
    }
 
    String readString() {
        return data[datalineId++];
    }
}

void saveData(int moveID) { 
    data.beginSave();
    for (int j=1; j < N; j++)
    {
        data.add(move[moveID][j].jointLeftShoulderRelative.x);
        data.add(move[moveID][j].jointLeftShoulderRelative.y);
        data.add(move[moveID][j].jointLeftShoulderRelative.z);
        
        data.add(move[moveID][j].jointLeftElbowRelative.x);
        data.add(move[moveID][j].jointLeftElbowRelative.y);
        data.add(move[moveID][j].jointLeftElbowRelative.z);
        
        data.add(move[moveID][j].jointLeftHandRelative.x);
        data.add(move[moveID][j].jointLeftHandRelative.y);
        data.add(move[moveID][j].jointLeftHandRelative.z);
        
        data.add(move[moveID][j].jointRightShoulderRelative.x);
        data.add(move[moveID][j].jointRightShoulderRelative.y);
        data.add(move[moveID][j].jointRightShoulderRelative.z);
        
        data.add(move[moveID][j].jointRightElbowRelative.x);
        data.add(move[moveID][j].jointRightElbowRelative.y);
        data.add(move[moveID][j].jointRightElbowRelative.z);
        
        data.add(move[moveID][j].jointRightHandRelative.x);
        data.add(move[moveID][j].jointRightHandRelative.y);
        data.add(move[moveID][j].jointRightHandRelative.z);
    }
    String str = Integer.toString(moveID);    
    data.endSave(dataPath("pose" + str + ".data"));
}

void loadData(int moveID) {
    // LOADING
    String str = Integer.toString(moveID);
    data.load(dataPath("pose" + str + ".data"));
    for (int j=1; j < N; j++)
    {
        move[moveID][j].jointLeftShoulderRelative.x = data.readFloat();
        move[moveID][j].jointLeftShoulderRelative.y = data.readFloat();
        move[moveID][j].jointLeftShoulderRelative.z = data.readFloat();
        
        move[moveID][j].jointLeftElbowRelative.x = data.readFloat();
        move[moveID][j].jointLeftElbowRelative.y = data.readFloat();
        move[moveID][j].jointLeftElbowRelative.z = data.readFloat();
        
        move[moveID][j].jointLeftHandRelative.x = data.readFloat();
        move[moveID][j].jointLeftHandRelative.y = data.readFloat();
        move[moveID][j].jointLeftHandRelative.z = data.readFloat();
        
        move[moveID][j].jointRightShoulderRelative.x = data.readFloat();
        move[moveID][j].jointRightShoulderRelative.y = data.readFloat();
        move[moveID][j].jointRightShoulderRelative.z = data.readFloat();
        
        move[moveID][j].jointRightElbowRelative.x = data.readFloat();
        move[moveID][j].jointRightElbowRelative.y = data.readFloat();
        move[moveID][j].jointRightElbowRelative.z = data.readFloat();
        
        move[moveID][j].jointRightHandRelative.x = data.readFloat();
        move[moveID][j].jointRightHandRelative.y = data.readFloat();
        move[moveID][j].jointRightHandRelative.z = data.readFloat();

        
        if (NORMALIZE_POSE_ON_LOAD) move[moveID][j] = normalizePose( move[moveID][j] );
        if (ROTATE_POSE_ON_LOAD) normalizePoseRotation( move[moveID][j]);
      
  } 
}


// -----------------------------------------------------------------
// Keyboard events

void keyPressed()
{
  // key interaction only in debug mode
  if (!showDebugUI) {
    return;
  }
  // check for active users
  IntVector userList = new IntVector();
  context.getUsers(userList);
  if(userList.size() < 1)
  {
    println("You need at least one active user!");
    return;
  }
  
  int user = userList.get(0);
  
  if ( (key >= '0') && (key <= '9') && (foundSkeleton) )
  {
      int keyIndex = key-'0';
    
      println("POSE " + keyIndex + " SAVED");
      ringbuffer[0].copyBuffer(keyIndex);
    
      String str = Integer.toString(keyIndex);
      pg.save(dataPath("pose" + str + ".png")); 
      saveData(keyIndex);
      foto[keyIndex] = loadImage(dataPath("pose" + str + ".png"));
      empty[keyIndex] = false;
      updateDisplay = true;
  }
  
  switch(key)
  {
  case 'c': 
    // savePose = -1; 
    pg.save ("capture.png");
    break; 

  case '+':
    displayCost++;
    if (displayCost > 9) displayCost = 9;
    break;
  
  case '-':
    displayCost--;
    if (displayCost < 0) displayCost = 0;
    break;
/* no dynamic switching is supported
  case 'u':
    showDebugUI = ! showDebugUI;
    println(" show debug "+showDebugUI);
    break;
    */
  case 'd':
    updateDisplay = true;
    if (switchDisplay) 
      switchDisplay = false;
    else
      switchDisplay = true;
    break;
  
  case 's':
    // check for active users
    context.getUsers(userList);
    if(userList.size() < 1)
    {
      println("You need at least one active user!");
      return;
    }
  
    user = userList.get(0);

    if(context.isTrackingSkeleton(user))
    {
      if(context.saveCalibrationDataSkeleton(user,"calibration.skel"))
        println("Saved current calibration to file.");      
      else
        println("Can't save current calibration to file.");         
    }
    else
      println("There is no calibration data to save.");    
    break;
    
  case 'l':    
    // check for active users
    context.getUsers(userList);
    if(userList.size() < 1)
    {
      println("You need at least one active user!");
      return;
    }
  
    user = userList.get(0);

    if(context.loadCalibrationDataSkeleton(user,"calibration.skel"))
    {
      context.startTrackingSkeleton(user);
      println("Load calibration from file.");
    }
    else
      println("Can't load calibration file.");
    break;
  }
  
}
