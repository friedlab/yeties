/* --------------------------------------------------------------------------
* Yeties - A funky kinect snowball fight game 
* --------------------------------------------------------------------------
* prog:  Matthias Wölfel,
*        Elke Müller,
*        Jan Felix Reuter,
         Thomas Schenker,
	 Alexander Liebrich,
	 Hoang Anh Le
*        and others (need_to_be_added)
* date:  04/12/2011 (m/d/y)
* ver:   0.1
* ----------------------------------------------------------------------------
*/

// === setup ==================================================
boolean autoPoseDetection = true;
boolean useFullscreen = false;

static boolean IS_DEBUG_MODE = false;

static int CHECK_SKELETON_COUNT = 6;
static final int PLAYER_COUNT = 3;
static boolean NORMALIZE_SIZE = true;
static boolean NORMALIZE_ROTATION = true;
// MWO: something wrong in the loading routine of NORMALIZE_ROTATION_ON_LOAD (shows in DTW, but can't find error)
static boolean NORMALIZE_ROTATION_ON_LOAD = true;
static boolean NORMALIZE_SIZE_ON_LOAD = true;

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
// which gesture buffer to show in the debug ui
int displayCost = 1;

// gesture recognition: 
int N = 10;
int M = 2*N;


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

//TODO:@LHA: this is only for one player 
// the actual hand pose
HandPoseAbsolute[] actualHandPose = new HandPoseAbsolute[PLAYER_COUNT];

// the old hand pose ( 10 poses before)
HandPoseAbsolute[] oldHandPose = new HandPoseAbsolute[PLAYER_COUNT];

// latenz
int X = 8;
// ==== COPY ===


////////////////////////////////////
// game logic
int counter = 0;
int counter2 = 0;
int counterEvent = 0;

// local player positions
PVector localPlayerNecks[] = new PVector[PLAYER_COUNT];
static int SUPPRESS_THROW = 25; // one throw every second;
// suppress multiple throws during a (frame) counter timespan
int localPlayerThrowCounter[] = new int[PLAYER_COUNT];


PVector remotePlayerNecks[] = new PVector[PLAYER_COUNT];
PVector remotePlayerActualHands[] = new PVector[PLAYER_COUNT];
PVector remotePlayerOldHands[] = new PVector[PLAYER_COUNT];


// removed: is in PLAYER_COUNT: int spielerzahl = 3;
//int modus = 3; //TODO: welcher modus hat welche bedeutung?
static int PLAYER_MODUS_COUNT = 3;

//PVector throwStartPos;
//PVector throwEndPos;

int localPointsSum = 0;
int remotePointsSum = 0;


static float HIT_DISTANCE = 0.6f; // 60cm normalized shoulder width
static float FRIGHTENED_DISTANCE = 1.2f; // 

static int HIT_SUCCESS_VALUE = 2;
static int HIT_FRIGHTENED_VALUE = 1;
static int HIT_FAILED_VALUE = 0;
// if the player is hit, he is suspended for 1 second
static int HIT_COUNTER = 25;

// if a player is hit, used as a timer (with HIT_COUNTER)
int localPlayerHit[] = new int[PLAYER_COUNT];
int remotePlayerHit[] = new int[PLAYER_COUNT];
// if a player is not really hit, used as a time (wit HIT_COUNTER)
int localPlayerFrightened[] = new int[PLAYER_COUNT];
int remotePlayerFrightened[] = new int[PLAYER_COUNT];

// counts the hits
int localPlayerHitCounter[] = new int[PLAYER_COUNT];
int remotePlayerHitCounter[] = new int[PLAYER_COUNT];


////////////////////////////////////
// game ui

PGraphics pg;

PImage[][] playas = new PImage[PLAYER_COUNT][PLAYER_MODUS_COUNT];
PImage bg;

PImage snowball;

float kinectHeight = 0.5f;
float kinectDistance = 0.2f;
float otherKinectHeight = 0.5f;
float otherKinectDistance = 0.2f;
float cameraDistance = 2f;
float cameraHeight = 1.4f;
float screenWidthMeters = 4;

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
      // poseArray[actualPointer].leftHandAbsolute.y = aNewPose.leftHandAbsolute.y;
      poseArray[actualPointer].leftHandAbsolute.y = 0.0;
      poseArray[actualPointer].leftHandAbsolute.z = aNewPose.leftHandAbsolute.z;
      
      poseArray[actualPointer].rightHandAbsolute.x = aNewPose.rightHandAbsolute.x;
      // poseArray[actualPointer].rightHandAbsolute.y = aNewPose.rightHandAbsolute.y;
      poseArray[actualPointer].rightHandAbsolute.y = 0.0;
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
  
  // get the actual pose
  HandPoseAbsolute getTheActualPose(){
    return poseArray[getActualPointer()];
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

    float weight_x = 0.5;
    float weight_y = 0.5;
    float weight_z = 1.5;

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

            if (n < 0) n = 0;
            if (m < 0) m = 0;
                        
            int tempN = n;
            if (P[displayCost][n][m] >= 0) tempN--;
            if (P[displayCost][n][m] <= 0) m--;
            n = tempN;
            if (n < 0) n = 0;
            if (m < 0) m = 0;            
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
    // String ipAddressToSendTo = "10.1.0.48";

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
    
    for (int i=0; i < PLAYER_COUNT; i++) {
      localPlayerNecks[i] = null;
      localPlayerThrowCounter[i] = SUPPRESS_THROW;
      //TODO: hit counters, player should not be lost for the whole game
      localPlayerHit[i] = 0;
      localPlayerHitCounter[i] = 0;
      localPlayerFrightened[i] = 0;
      remotePlayerHit[i] = 0;
      remotePlayerHitCounter[i] = 0;
      remotePlayerFrightened[i] = 0;
      
      actualHandPose[i] = new HandPoseAbsolute();
      oldHandPose[i] = new HandPoseAbsolute();
      
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
    textMode(SCREEN);
   
    //TODO: ui stuff
    if (IS_DEBUG_MODE)
    {
      background(0,0,0);
      stroke(0,0,255);
      strokeWeight(3);
      smooth();
      pg = createGraphics(context.depthWidth(), context.depthHeight(), P2D);
      size(1170, 800, P3D);

    }
    else {
      // we are in game mode
      pg = createGraphics(context.depthWidth(), context.depthHeight(), P2D);
      
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
     
      remotePlayerNecks[i] = new PVector();
      remotePlayerNecks[i].x = updateMessage.get(i*9+0).floatValue();
      // remotePlayerNecks[i].y = updateMessage.get(i*9+1).floatValue();
      remotePlayerNecks[i].y = 0.0;
      remotePlayerNecks[i].z = 0.5*updateMessage.get(i*9+2).floatValue();


      remotePlayerOldHands[i] = new PVector();
      remotePlayerOldHands[i].x = updateMessage.get(i*9+3).floatValue();
      // remotePlayerOldHands[i].y = updateMessage.get(i*9+4).floatValue();
      remotePlayerOldHands[i].y = 0.0;
      remotePlayerOldHands[i].z = updateMessage.get(i*9+5).floatValue();


      remotePlayerActualHands[i] = new PVector();
      remotePlayerActualHands[i].x = updateMessage.get(i*9+6).floatValue();
      // remotePlayerActualHands[i].y = updateMessage.get(i*9+7).floatValue();
      remotePlayerActualHands[i].y = 0.0;
      remotePlayerActualHands[i].z = updateMessage.get(i*9+8).floatValue();
     
   }
/*
   for(int i = 0; i < PLAYER_COUNT; i++) {
     
      if(remotePlayerNecks[i].x != 0) {
       
        println("player " + i);
        println("\tneck: ");
        println("\t" + remotePlayerNecks[i]);
        
        println("\told hand");
        println("\t" + remotePlayerOldHands[i]);
        
        println("\tactual hand");
        println("\t" + remotePlayerActualHands[i]);

      } else {
        //println("no information");
      }
 }
*/ 
}

class Ball
{
  PVector pos;
  PVector dir;
  boolean isLocal;
}

ArrayList<Ball> balls = new ArrayList<Ball>();

void updateBalls()
{
  for (Ball b : balls)
  {
    // update ball position
    b.pos.add(b.dir); // TODO: use game time
    //
    
    // debug drawing
    line(b.pos.x, b.pos.y, b.pos.z, b.pos.x, 0, b.pos.z);
    
    // draw ball
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

  // detect removable balls
  for (int i = 0; i < balls.size(); ++i)
  {
    if (isBrokenBall(balls.get(i)))
    {
      balls.remove(i);
      --i;
    }
    
  }
}

void updatePlayerHits() {
  // the transformed neck
  PVector neck = new PVector();

  // remote players
  for (int i=0; i < PLAYER_COUNT; i++) {
    if (vectorNullOrZero( remotePlayerNecks[i] ) ) continue;


    otherKinectToFieldScaled.mult(remotePlayerNecks[i], neck);
          
    int hv = isPlayerHit(neck, false );

    
    if (hv == HIT_SUCCESS_VALUE) {
      println("remote player "+i+" hit");
      remotePlayerHit[i] = HIT_COUNTER;
      remotePlayerHitCounter[i]++;
      localPointsSum++;
    }
    else if (hv == HIT_FRIGHTENED_VALUE ) {
      println("remote player "+i+" merly hit" +hv);
  
      remotePlayerFrightened[i] = HIT_COUNTER;
    }
  }
  
  // local players
  for (int i=0; i < PLAYER_COUNT; i++) {
    if (vectorNullOrZero( localPlayerNecks[i] ) ) continue;

    neck = localPlayerNecks[i];
    //already transformed
//    kinectToFieldScaled.mult(localPlayerNecks[i], neck);    
    int hv = isPlayerHit( neck ,true );

    if (hv == HIT_SUCCESS_VALUE) {
      println(" local player "+i+" hit ");
      localPlayerHit[i] = HIT_COUNTER;
      localPlayerHitCounter[i]++;
      remotePointsSum++;
    }
    else if (hv == HIT_FRIGHTENED_VALUE ) {

        println(" local player "+i+" merly hit "+hv);
          localPlayerFrightened[i] = HIT_COUNTER;
    }
    
  }
}



// checks all balls on remote players
int isPlayerHit(PVector pos,boolean localBalls) {
  for (int i=0; i < balls.size(); ++i) {
    Ball b = balls.get(i);
    
//    println(" check ball "+balls.get(i).isLocal + ": "+localBalls);
    if (b.isLocal == localBalls) continue; // dont check local balls with local players
//    println(" check ball  ok ");
    float distance = PVector.dist(pos , b.pos);
//    println(" ball "+i +" player pos "+pos + " ball pos "+b.pos+" dist "+distance);
    if (distance < HIT_DISTANCE) {
//      println("player hit by ball is local"+localBalls);
      
      // remove ball
      balls.remove(i);
      --i;
      
      //TODO: if we had also an array of the local players...
      return HIT_SUCCESS_VALUE;
    } 
    if (distance < FRIGHTENED_DISTANCE) {
//      println("player frightened : ball is local "+localBalls);
      return HIT_FRIGHTENED_VALUE;
    }
  }
  return HIT_FAILED_VALUE;
}


boolean isBrokenBall(Ball ball)
{
    return (ball.pos.mag() > 10 ||
            ball.pos.y <= 0 ||
            ball.pos.y >= 3);
}


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
		
    //TODO: lost popMatrix ? 
    //popMatrix();
    
 		
		
   
    pushMatrix();
    applyMatrix(kinectToField);
		/* Kinectkoordinaten */
 
     //box(0.3f, 0.05f, 0.07f);
    popMatrix();
}

void drawPlayer(PVector neck, boolean transparent,int hit,int frightened)
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
             // scale the yetie horziontally if player was hit
             float hf = 1f- ((float)hit / 25f);
             float heldHeight = (neck.y*hf) + 0.3f;
             float heldWidth = heldHeight * aspect;
             translate(neck.x, 0, neck.z);
             //println(" hit "+ hit + " fr "+frightened);
             //TODO rotate like tischfussballfigur is player is in hit state
             //rotate( (float) hit, (float)frightened,0f,0f);
             
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

void createBall(PVector throwStartPos, PVector throwEndPos, PMatrix3D transform,boolean isLocal)
{
                  Ball ball = new Ball();
                  ball.isLocal = isLocal;
                  PVector fieldThrowEndPos = new PVector();
                  transform.mult(throwEndPos, fieldThrowEndPos);
                  
                  PVector fieldThrowStartPos = new PVector();
                  transform.mult(throwStartPos, fieldThrowStartPos);
               
                    
                  ball.pos = fieldThrowEndPos;
                         
                  PVector dir = PVector.sub(fieldThrowEndPos, fieldThrowStartPos);
                  dir.normalize();
                  dir.y = -0.01;
                  dir.x *= 0.05f; // less horziontal sensivity
                  //dir.z *= 0.1f; 
                  // no speed info
                  if (dir.z < 0) dir.z = -0.1f;
                  else dir.z = 0.1f;
                                  
                                  

                  PVector notHit = new PVector();
                  notHit.x = dir.x;
                  notHit.y = dir.y;
                  notHit.z = dir.z;
                  
                  notHit.normalize();
                  notHit.mult(HIT_DISTANCE);
                  ball.pos.x += notHit.x;
                  ball.pos.y += notHit.y;
                  ball.pos.z += notHit.z;
                  
                  ball.dir = dir; 
                  println(" create ball "+ ball.pos + " dir "+ ball.dir );  
                
                  balls.add(ball);    
}

void computeCosts(int detectedPlayerCount) {
  //TODO: this must be also in game mode        
  // current person count
  for (int p = 0; p<detectedPlayerCount; p++) {
    for (int i = 0; i < MAX_GESTURE_COUNT; i++) {
      // compute cost
      if (!empty[i]) {
        costLast[p][i] = cost[p][i];
        cost[p][i] = ringbuffer[p].pathcost(i);
        cost[p][i] = (log(cost[p][i]-1.0) - 5.5)/2.0;
        // println("cost(" + i + "): " + cost);
      }
    }
  }
}

void drawDebug() {
  
    foundSkeleton = false;
    int  detectedPlayerCount = 0;

    // draw depthImageMap
    pg.image(context.depthImage(),0,0);
    // draw ringbuffer
    ringbuffer[0].display();

    // process the skeleton if it's available
    for (int i = 1; i< CHECK_SKELETON_COUNT; i++) {
      if ( (context.isTrackingSkeleton(i)) && (detectedPlayerCount < PLAYER_COUNT) ) {
            PVector neck = evaluateSkeleton(i,detectedPlayerCount);
            foundSkeleton = true;
            detectedPlayerCount++;
      }
    }
      
    if (switchDisplay) {
      image(pg, 0, 0);
    } else {
        pushMatrix();
        scale(-1.0, 1.0);
        image(pg,-pg.width,0);
        popMatrix();
    }
  
    // no skeleton found
    if (!foundSkeleton)  {
        stroke(0,0,0);
        fill(0,0,0);
        rect(context.depthWidth(), 0, 400, 485);
            
        stroke(0,0,255);
        fill(255,0,0);
        textFont(fontA32, 32);
        textAlign(CENTER);
        text("Please register user!", context.depthWidth() / 2, 40);
        image(shapeOfUser, 0, 0);
    } else { // show player warning messages
        for (int i=0; i < PLAYER_COUNT; i++) {
          if ((warning[i] >= 0) ) {
            image(warnings[i][warning[i]],context.depthWidth()/2-100, context.depthHeight()/2 - 50);
          }
       }
    }

    // if the display has changed, update only on demand
    if (updateDisplay) {
          updateDisplay = false;
        
          int ghc = (MAX_GESTURE_COUNT/2);
          if (switchDisplay) {
              for (int i = 0; i< ghc; i++) {
                  image(foto[i], i * (context.depthWidth() + 400) / ghc + i*ghc, context.depthHeight() + 15, (context.depthWidth() + 400) / ghc, 130);
                  image(foto[(i+ghc)], i * (context.depthWidth() + 400) / ghc + i*5, context.depthHeight() + 170, (context.depthWidth() + 400) / ghc, 130);
              }
          } else {
              for (int i = 0; i<ghc; i++) {
                  pushMatrix();
                  scale(-1.0, 1.0);
                  image(foto[i], -(i+1) * (context.depthWidth() + 400) / ghc - i*ghc, context.depthHeight() + 15, (context.depthWidth() + 400) / ghc, 130);
                  image(foto[(i+5)], -(i+1) * (context.depthWidth() + 400) / ghc - i*ghc, context.depthHeight() + 170, (context.depthWidth() + 400) / ghc, 130);
                  popMatrix();
              }
          }
          
          for (int i = 0; i<=4; i++) {
              if (empty[i]) {
                  pushMatrix();
                  scale(-1.0, 1.0);
                  image(foto[i], -(i+1) * (context.depthWidth() + 400) / 5 - i*5, context.depthHeight() + 15, (context.depthWidth() + 400) / 5, 130);
                  popMatrix();
              }
              if (empty[i+5]) {
                  pushMatrix();
                  scale(-1.0, 1.0);
                  image(foto[(i+5)], -(i+1) * (context.depthWidth() + 400) / 5 - i*5, context.depthHeight() + 170, (context.depthWidth() + 400) / 5, 130);
                  popMatrix();
              }            
          }
  
          textFont(fontA12, 16);
          fill(255,255,255);
          textAlign(CENTER);
          for (int i = 0; i<=4; i++) {
              text(i, i * (context.depthWidth() + 400) / 5 + i*5 + 12, context.depthHeight() + 32);
              // image(foto[i], i * (context.depthWidth() + 400) / 5 + i*5, context.depthHeight() + 15, (context.depthWidth() + 400) / 5, 130);
              text((i+5), i * (context.depthWidth() + 400) / 5 + i*5 + 12, context.depthHeight() + 187);
              // image(foto[(i+5)], i * (context.depthWidth() + 400) / 5 + i*5, context.depthHeight() + 170, (context.depthWidth() + 400) / 5, 130);
          }                
      }
    
    
    
    
    // evaluate and draw DTW
    if (foundSkeleton)
    {
          noStroke();             
          fill(0,0,0);
          rect(0, context.depthHeight() + 145, 1070, 20);
          rect(0, context.depthHeight() + 300, 1070, 20);
    }
    
    computeCosts(detectedPlayerCount);
          
    //TODO: this must be also in game mode        
    // current person count
    for (int p = 0; p<detectedPlayerCount; p++) {
        for (int i = 0; i < MAX_GESTURE_COUNT; i++) {
            // compute cost
            if (!empty[i]) {
                    
                fill(255,0,0);
                if (p == 1) fill(0,0,255);
                if ( cost[p][i] <= 0.25 ) fill(0,255,0);
                    
            
                    
                if ( ( cost[p][i] > 0.25 ) && ( cost[p][i] < 0.35 ) ) {
                    float normalized = 10.0 * (cost[p][i] - 0.25);
                    fill(255 * normalized,255 * (1.0-normalized),0);
                    if (p == 1) fill(0,255 * (1.0-normalized),255 * normalized); 
                }
                                    
                if (i < 5) rect(i * (context.depthWidth() + 400) / 5 + i*5, context.depthHeight() + 145 + 10*p, min(1.0, max(0.01, 1.0-cost[p][i])) * ((context.depthWidth() + 400) / 5), 10);
                if (i >= 5) rect((i-5) * (context.depthWidth() + 400) / 5 + (i-5)*5, context.depthHeight() + 300 + 10*p, min(1.0, max(0.01, 1.0-cost[p][i])) * ((context.depthWidth() + 400) / 5), 10);
            }
        }
    }

}
    
void draw()
{
  strokeWeight(0);

  // update the cam
    context.update();
  
    

    if (IS_DEBUG_MODE)  
    {
      pg.beginDraw();
      drawDebug();
      pg.endDraw();
      return;
    }
   
    
    //we are in game mode
    background(bg); 


    // draw player hits
//     strokeWeight(0);
    textFont(fontA32, 32);
    fill(255,255,255,128);
    rect(20,20,screenWidth -20,142);
    fill(0,0,0,255);
    textAlign(CENTER);
    text("Local Hits "+ localPointsSum, 1170 * 0.15f,40);
    textAlign(CENTER);
    text("Remote Hits "+ remotePointsSum, 1170 * 0.85f, 40);
  //   strokeWeight(0); 
     
     
    pg.beginDraw();


      int  detectedPlayerCount = 0;

      drawGameField();

   
    
        
      // draw remote players ?!?
      // trigger ball on remote player throw detection / message
      for (int i = 0; i < remotePlayerNecks.length; ++i) {
        if(!vectorNullOrZero(remotePlayerNecks[i])) {
          PVector neck = new PVector();
          otherKinectToFieldScaled.mult(remotePlayerNecks[i], neck);

            // update player hit counters          
          // if player is hit ?!?
          if (remotePlayerHit[i] > 0) {
            remotePlayerHit[i]--;
          }
          if (remotePlayerFrightened[i] > 0) {
            remotePlayerFrightened[i]--;
          }
            
          drawPlayer(neck, false,remotePlayerHit[i],remotePlayerFrightened[i]); 
        }
      
        // remote throw detection
        if (!vectorNullOrZero(remotePlayerActualHands[i]) &&
          !vectorNullOrZero(remotePlayerOldHands[i])) {
          createBall(remotePlayerOldHands[i], remotePlayerActualHands[i], otherKinectToFieldScaled,false);
          remotePlayerActualHands[i] = null;
          remotePlayerOldHands[i] = null;
        }
      }
    
  
      //TODO: currently the players are switching their ids, it has to be mapped
      // detect and draw local players
      for (int i = 1; i< CHECK_SKELETON_COUNT; i++) {
        if ( (context.isTrackingSkeleton(i)) && (detectedPlayerCount < PLAYER_COUNT) ) {
            PVector neck = evaluateSkeleton(i,detectedPlayerCount);

//            println("old neck "+neck);
              neck.y = 150.0;
            kinectToFieldScaled.mult(neck, neck);
//            println("new neck "+neck);
            
            // save the transformed position   
            localPlayerNecks[detectedPlayerCount] = neck;
            //translate(neck.x * 0.001f, neck.y * 0.001f, -neck.z * 0.001f);
           
            // update player hit counters
            if (localPlayerHit[detectedPlayerCount] > 0) {
              localPlayerHit[detectedPlayerCount]--;
            }
            if (localPlayerFrightened[detectedPlayerCount] > 0) {
              localPlayerFrightened[detectedPlayerCount]--;
            }
            
            drawPlayer(neck, true,localPlayerHit[detectedPlayerCount],localPlayerFrightened[detectedPlayerCount]);
            
            detectedPlayerCount++;
        }
      }
      
      // remove lost or not registered local players
      for (int i=detectedPlayerCount; i < PLAYER_COUNT; i++) {
        localPlayerNecks[i] = null;
      }
      
      // update local player throw counter
      for (int i=0; i < PLAYER_COUNT; i++) {
        if (localPlayerThrowCounter[i] < SUPPRESS_THROW) localPlayerThrowCounter[i]++; 
      }
    
      computeCosts(detectedPlayerCount); // updates only if person > 0
        
      boolean playerLeftHandGestureDetected[] = new boolean[PLAYER_COUNT]; 
      boolean playerRightHandGestureDetected[] = new boolean[PLAYER_COUNT];
      for(int k = 0; k < PLAYER_COUNT; k++) {
        playerLeftHandGestureDetected[k] = false;
        playerRightHandGestureDetected[k] = false;
      }  
        
          
      for (int p = 0; p<detectedPlayerCount; p++) {
        if (localPlayerThrowCounter[p] < SUPPRESS_THROW) {
          //println("throw suppressed for player "+p+" c "+localPlayerThrowCounter[p]);
        }
        else {
          for (int i = 0; i < MAX_GESTURE_COUNT; i++) {
            if (!empty[i]) {
              // check if gesture is accepted and was not accepted before
              if ( ( cost[p][i] < 0.3 ) && ( costLast[p][i] >= 0.3 ) ) {
                 println("Throw!");
                 
                 // reset throw counter
                 localPlayerThrowCounter[p] = 0;
                    
                 //TODO: how to map the detected (in evaluateSkeleton) left or right arm to these [left/right]HandAbsolute
                 //TODO:@LHA: why only the left hand ?
                 // should be: PVector lHA = actualHandPose[p].leftHandAbsolute;
                 PVector lHA = actualHandPose[p].leftHandAbsolute;
                 PVector throwEndPos = new PVector(lHA.x, lHA.y, lHA.z);
                 PVector olHA = oldHandPose[p].leftHandAbsolute;
                 PVector throwStartPos = new PVector(olHA.x, olHA.y, olHA.z);
       
                 // geht davon aus dass die gesten für die linke hand auf 0-4 und die gesten für die rechte hand auf 5-9 liegen
                 if(i < 5) {
                   playerLeftHandGestureDetected[p] = true;
                 } else {
                   playerRightHandGestureDetected[p] = true;
                 }

                 // virtually throw: add local ball
                 createBall(throwStartPos, throwEndPos, kinectToFieldScaled,true);           
                                 
              }
            }
          }
        }  
      }
  
  
  
      // get the current neck position of every player, for osc message
      PVector playerNecks[] = new PVector[4];
      for(int n = 0; n < PLAYER_COUNT; n++) {
        playerNecks[n] = new PVector();
        context.getJointPositionSkeleton(n,SimpleOpenNI.SKEL_NECK,playerNecks[n]);
        if(playerNecks[n] == null) {
          playerNecks[n] = new PVector(0,0,0);
        }
      }

      // get the current and old hand position of every player, for osc message
      PVector playerActualHands[] = new PVector[PLAYER_COUNT];
      PVector playerOldHands[] = new PVector[PLAYER_COUNT];
      
      for(int k = 0; k < PLAYER_COUNT; k++) {
    
          if(playerLeftHandGestureDetected[k]) {
            playerActualHands[k] = ringbufferHand[k].getTheActualPose().leftHandAbsolute;
            playerOldHands[k] = ringbufferHand[k].getTheOlderPose().leftHandAbsolute;
            playerOldHands[k].y = -10.0;
            playerLeftHandGestureDetected[k] = false;
          } else if(playerRightHandGestureDetected[k]) {
            playerActualHands[k] = ringbufferHand[k].getTheActualPose().rightHandAbsolute;
            playerOldHands[k] = ringbufferHand[k].getTheOlderPose().rightHandAbsolute;
            playerOldHands[k].y = -10.0;            
            playerRightHandGestureDetected[k] = false;
          } else {
            playerActualHands[k] = new PVector(0, 0, 0);
            playerOldHands[k] = new PVector(0, 0, 0);
          }
      }

  
      OscMessage updateMessage = new OscMessage("/update"); 

      for(int s = 0; s < PLAYER_COUNT; s++) {
 
        updateMessage.add(playerNecks[s].x);
        updateMessage.add(playerNecks[s].y);
        updateMessage.add(playerNecks[s].z);
 
        updateMessage.add(playerOldHands[s].x);
        updateMessage.add(playerOldHands[s].y);
        updateMessage.add(playerOldHands[s].z);
    
        updateMessage.add(playerActualHands[s].x);
        updateMessage.add(playerActualHands[s].y);
        updateMessage.add(playerActualHands[s].z);
   
      }
      oscP5.send(updateMessage, myRemoteLocation); 
    
      updateBalls();
      updatePlayerHits();    

    // the matrix set in drawGameField()    
    popMatrix();



   
    pg.endDraw();

   
}

// draw the skeleton with the selected joints
PVector evaluateSkeleton(int userId, int detectedPlayerCount)
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
    if (IS_DEBUG_MODE)
    {
      if (detectedPlayerCount < PLAYER_COLORS.length) pg.stroke(PLAYER_COLORS[detectedPlayerCount].x,PLAYER_COLORS[detectedPlayerCount].y,PLAYER_COLORS[detectedPlayerCount].z,255);
    
      warning[detectedPlayerCount] = -1; 

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
          warning[detectedPlayerCount] = 0;
      }
    
      if (jointNeck2D.x >  540) 
      {
          warning[detectedPlayerCount] = 4;
      }
    
      if (jointNeck3D.z > 4000)
      {
          warning[detectedPlayerCount] = 2;
        
          if (jointNeck2D.x < 100) 
          {
              warning[detectedPlayerCount] = 1;
          }
        
          if (jointNeck2D.x > 540) 
          {
              warning[detectedPlayerCount] = 3;
          }
      }
    
      if (jointNeck2D.z <  1500)
      {
          warning[detectedPlayerCount] = 6;
        
          if (jointNeck2D.x < 100) 
          {
              warning[detectedPlayerCount] = 7;
          }
        
          if (jointNeck2D.x > 540) 
          {
              warning[detectedPlayerCount] = 5;
          }
      }
    }
    
    // ==== COPY ===
    // LHA
    // the actual handPose   
    
    actualHandPose[detectedPlayerCount].leftHandAbsolute.x = jointLeftHand3D.x;
    actualHandPose[detectedPlayerCount].leftHandAbsolute.y = jointLeftHand3D.y;
    actualHandPose[detectedPlayerCount].leftHandAbsolute.z = jointLeftHand3D.z;
    
    actualHandPose[detectedPlayerCount].rightHandAbsolute.x = jointRightHand3D.x;
    actualHandPose[detectedPlayerCount].rightHandAbsolute.y = jointRightHand3D.y;
    actualHandPose[detectedPlayerCount].rightHandAbsolute.z = jointRightHand3D.z;

    // add the new hand pose to the ringbuffer for hand poses
    ringbufferHand[detectedPlayerCount].addANewPose(actualHandPose[detectedPlayerCount]);
    
    // the older pose
    oldHandPose[detectedPlayerCount] = ringbufferHand[detectedPlayerCount].getTheOlderPose();
    
    if(oldHandPose == null) println("NO history");
    // ==== COPY ===
    
    // add new pose to ringbuffer
    if (NORMALIZE_SIZE) pose = normalizeSize(pose);
    if (NORMALIZE_ROTATION) pose = normalizeRotation(pose);    
    
    ringbuffer[detectedPlayerCount].fillBuffer( pose );
    
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
    // context.stopTrackingSkeleton(userId);
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

        
        if (NORMALIZE_SIZE_ON_LOAD) move[moveID][j] = normalizeSize( move[moveID][j] );
        if (NORMALIZE_ROTATION_ON_LOAD) move[moveID][j] = normalizeRotation( move[moveID][j]);
      
  } 
}


// -----------------------------------------------------------------
// Keyboard events

void keyPressed()
{
  // key interaction only in debug mode
  if (!IS_DEBUG_MODE) {
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
    IS_DEBUG_MODE = ! IS_DEBUG_MODE;
    println(" show debug "+IS_DEBUG_MODE);
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

