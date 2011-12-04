/* --------------------------------------------------------------------------
* Yeties - A funky kinect snowball fight game 
* --------------------------------------------------------------------------
* prog:  Matthias Wölfel and others (need to be added)
* date:  04/12/2011 (m/d/y)
* ver:   0.1
* ----------------------------------------------------------------------------
*/

// === setup ==================================================
boolean autoPoseDetection = true;
boolean useFullscreen = false;
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
OscBundle myBundle;
OscMessage myMessage;

int displayCost = 1;

int N = 25;
int M = 2*N;

int counter = 0;
int counter2 = 0;
int counterEvent = 0;

int person = 0;

// Relative Array of objects
Pose[][] grid;
Pose[][] move;

PGraphics pg;
PFont fontA32;
PFont fontA12;
PImage[] foto;
PImage[][] warnings;
PImage shapeOfUser;  
PImage kineticspace;

boolean foundSkeleton = false;
boolean switchDisplay = false;
boolean updateDisplay = true;

int steps[] = new int[10];
float speed[] = new float[10];
float cost[][] = new float[2][10];
float costLast[][] = new float[2][10];
boolean empty[] = new boolean[10];

Data data;
RingBuffer[] ringbuffer;

int warning[] = new int[2];

// define the pose object
class Pose
{
    PVector jointLeftShoulderRelative = new PVector();
    PVector jointLeftElbowRelative = new PVector();
    PVector jointLeftHandRelative = new PVector();
  
    PVector jointRightShoulderRelative = new PVector();
    PVector jointRightElbowRelative = new PVector();
    PVector jointRightHandRelative = new PVector();
}

// define the pose object
class RingBuffer
{
    Pose[] poseArray;
    int  startOfBuffer = 0;
    Float d[][][] = new Float[10][N][M];
    int P[][][] = new int[10][N][M];
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
	
	if (weight_left_or_right > 1.0) weight_left_or_right = 1.0;
	if (weight_left_or_right < -1.0) weight_left_or_right = -1.0;
		
	float weight_left = 1.0 - weight_left_or_right;
	float weight_right = 1.0 + weight_left_or_right;
		
	float mse = 0.0;
        
        mse += weight_left * sqrt( pow((move[moveID][j].jointLeftShoulderRelative.x - poseArray[i].jointLeftShoulderRelative.x),2) 
                   + pow((move[moveID][j].jointLeftShoulderRelative.y - poseArray[i].jointLeftShoulderRelative.y),2)
                   + pow((move[moveID][j].jointLeftShoulderRelative.z - poseArray[i].jointLeftShoulderRelative.z),2) );
        mse += weight_left * sqrt( pow((move[moveID][j].jointLeftElbowRelative.x - poseArray[i].jointLeftElbowRelative.x),2) 
                   + pow((move[moveID][j].jointLeftElbowRelative.y - poseArray[i].jointLeftElbowRelative.y),2)
                   + pow((move[moveID][j].jointLeftElbowRelative.z - poseArray[i].jointLeftElbowRelative.z),2) );
        mse += weight_left * sqrt( pow((move[moveID][j].jointLeftHandRelative.x - poseArray[i].jointLeftHandRelative.x),2) 
                   + pow((move[moveID][j].jointLeftHandRelative.y - poseArray[i].jointLeftHandRelative.y),2)
                   + pow((move[moveID][j].jointLeftHandRelative.z - poseArray[i].jointLeftHandRelative.z),2) );
        mse += weight_right * sqrt( pow((move[moveID][j].jointRightShoulderRelative.x - poseArray[i].jointRightShoulderRelative.x),2) 
                   + pow((move[moveID][j].jointRightShoulderRelative.y - poseArray[i].jointRightShoulderRelative.y),2)
                   + pow((move[moveID][j].jointRightShoulderRelative.z - poseArray[i].jointRightShoulderRelative.z),2) );
        mse += weight_right * sqrt( pow((move[moveID][j].jointRightElbowRelative.x - poseArray[i].jointRightElbowRelative.x),2) 
                   + pow((move[moveID][j].jointRightElbowRelative.y - poseArray[i].jointRightElbowRelative.y),2)
                   + pow((move[moveID][j].jointRightElbowRelative.z - poseArray[i].jointRightElbowRelative.z),2) );
        mse += weight_right * sqrt( pow((move[moveID][j].jointRightHandRelative.x - poseArray[i].jointRightHandRelative.x),2) 
                   + pow((move[moveID][j].jointRightHandRelative.y - poseArray[i].jointRightHandRelative.y),2)
                   + pow((move[moveID][j].jointRightHandRelative.z - poseArray[i].jointRightHandRelative.z),2) );
 
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


/* =====================================================================================
    setup
   ===================================================================================== */
void setup()
{
    // context = new SimpleOpenNI(this);
    context = new SimpleOpenNI(this,SimpleOpenNI.RUN_MODE_MULTI_THREADED);
  
    int portToListenTo = 7001; 
    int portToSendTo = 7000;
    String ipAddressToSendTo = "localhost";

    oscP5 = new OscP5(this,portToListenTo);
    myRemoteLocation = new NetAddress(ipAddressToSendTo, portToSendTo);  
    myBundle = new OscBundle();
    myMessage = new OscMessage("/"); 

    grid = new Pose[M][N];
    for (int i = 0; i < M; i++) {
      for (int j = 0; j < N; j++) {
        grid[i][j] = new Pose();
      }
    }

    move = new Pose[10][N];

    ringbuffer = new RingBuffer[2];
    for (int i = 0; i < 2; i++) {
        ringbuffer[i] = new RingBuffer();
    }
    
    data = new Data();
  
    for(int i = 0; i <= 9; i++) {
        for(int j = 0; j < N; j++) move[i][j] = new Pose();
    }

    foto = new PImage[10];
  
    // load the stored data
    for (int i = 0; i <= 9; i++) {
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

    warnings = new PImage[2][8];

    warnings[0][0] = loadImage(dataPath("go_left_red.png"));    
    warnings[0][1] = loadImage(dataPath("go_lf_red.png"));
    warnings[0][2] = loadImage(dataPath("go_front_red.png"));
    warnings[0][3] = loadImage(dataPath("go_rf_red.png"));
    warnings[0][4] = loadImage(dataPath("go_right_red.png"));
    warnings[0][5] = loadImage(dataPath("go_rb_red.png"));
    warnings[0][6] = loadImage(dataPath("go_back_red.png"));
    warnings[0][7] = loadImage(dataPath("go_lb_red.png"));

    warnings[1][0] = loadImage(dataPath("go_left_blue.png"));    
    warnings[1][1] = loadImage(dataPath("go_lf_blue.png"));
    warnings[1][2] = loadImage(dataPath("go_front_blue.png"));
    warnings[1][3] = loadImage(dataPath("go_rf_blue.png"));
    warnings[1][4] = loadImage(dataPath("go_right_blue.png"));
    warnings[1][5] = loadImage(dataPath("go_rb_blue.png"));
    warnings[1][6] = loadImage(dataPath("go_back_blue.png"));
    warnings[1][7] = loadImage(dataPath("go_lb_blue.png"));

    warning[0] = -1;
    warning[1] = -1;

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
 
    background(0,0,0);
    stroke(0,0,255);
    strokeWeight(3);
    smooth();
  
    pg = createGraphics(context.depthWidth(), context.depthHeight(), P2D);
  
    if (!useFullscreen)
    {
        // size(1070, 850, OPENGL); 
        // size(1070, 850); 
        size(1170, 800);
        
        pushMatrix();
        rotate(-PI/2);
        image(kineticspace, -780, 1070);
        popMatrix();
    }
    else
    {
        size(1280, 800);
        // Create the fullscreen object
        // fs = new FullScreen(this); 
  
        // enter fullscreen mode
        // fs.enter(); 
        
        pushMatrix();
        rotate(-PI/2);
        image(kineticspace, -780, 1180);
        popMatrix();
    }
}

void sendOSCEvent(int event, int person)
{
    /* ==============================================================================
        part to define OSC messages 
       ============================================================================== */
    switch(event)
    {
        case 1:
            myMessage = new OscMessage("/layer1/clip1/connect");        
        break; 

        case 2:
            myMessage = new OscMessage("/layer1/clip2/connect");        
        break;                   
        
        case 3:
            if (person == 0)
            {
                myMessage = new OscMessage("/layer2/clip2/connect");
            }
            else
            {
                myMessage = new OscMessage("/layer3/clip3/connect");
            }
        break;
        
        default: 
            myMessage = new OscMessage("/layer1/clip3/connect");        
        break;  
    }    

    int temp = 1;
    myMessage.add(temp);
    myBundle.add(myMessage);
    myMessage.clear();
    oscP5.send(myBundle, myRemoteLocation); 
    myBundle.clear();  
}

void draw()
{
    // update the cam
    context.update();
  
    // draw depthImageMap
    pg.beginDraw();
    pg.image(context.depthImage(),0,0);

    ringbuffer[0].display();
    
    // process the skeleton if it's available
    foundSkeleton = false;
    person = 0;

    for (int i = 1; i<6; i++)
    {
        if ( (context.isTrackingSkeleton(i)) && (person < 2) )
        {
            evaluateSkeleton(i);
            foundSkeleton = true;
            person++;
        }
    }
  
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
  
    pg.endDraw();

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
    else if ((warning[0] >= 0) || (warning[1] >= 0))
    {
        if ((warning[0] >= 0) && (warning[1] < 0))
        {
            image(warnings[0][warning[0]],context.depthWidth()/2-100, context.depthHeight()/2 - 50);
        }
        else if ((warning[1] >= 0) && (warning[0] < 0))
        {
            image(warnings[1][warning[1]],context.depthWidth()/2-100, context.depthHeight()/2 - 50);
        }
        else
        {
            image(warnings[0][warning[0]],context.depthWidth()/2-100-75, context.depthHeight()/2 - 50);
            image(warnings[1][warning[1]],context.depthWidth()/2-100+75, context.depthHeight()/2 - 50);
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
  
    // evaluate and draw DTW
    if (foundSkeleton)
    {
     
        noStroke();             
        fill(0,0,0);
        rect(0, context.depthHeight() + 145, 1070, 20);
        rect(0, context.depthHeight() + 300, 1070, 20);
        
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
                    
                    if ( ( cost[p][i] < 0.3 ) && ( costLast[p][i] >= 0.3 ) )
                    {
                        println("found gesture #" + i + " user #" + p);
                        sendOSCEvent(i, p);                    
                    }   
                }
            }
        }
    }
}


// draw the skeleton with the selected joints
void evaluateSkeleton(int userId)
{
    Pose pose = new Pose();

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

    if (person == 0) pg.stroke(255,0,0,255);
    if (person == 1) pg.stroke(0,0,255,255);
    pg.strokeWeight(5);
    pg.line(jointNeck2D.x,jointNeck2D.y, jointLeftShoulder2D.x,jointLeftShoulder2D.y);
    pg.line(jointLeftShoulder2D.x,jointLeftShoulder2D.y, jointLeftElbow2D.x,jointLeftElbow2D.y);
    pg.line(jointLeftElbow2D.x,jointLeftElbow2D.y, jointLeftHand2D.x,jointLeftHand2D.y);  
    pg.line(jointNeck2D.x,jointNeck2D.y, jointRightShoulder2D.x,jointRightShoulder2D.y);
    pg.line(jointRightShoulder2D.x,jointRightShoulder2D.y, jointRightElbow2D.x,jointRightElbow2D.y);
    pg.line(jointRightElbow2D.x,jointRightElbow2D.y, jointRightHand2D.x,jointRightHand2D.y);

    warning[person] = -1; 
  
    textAlign(CENTER);
    textFont(fontA32, 32);
    fill(255,0,0);
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
  
  
    // add new pose to ringbuffer
    pose = normalizePose( pose );
    ringbuffer[person].fillBuffer( pose );
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
        
        move[moveID][j] = normalizePose( move[moveID][j] );
    } 
}


// -----------------------------------------------------------------
// Keyboard events

void keyPressed()
{  
  IntVector userList = new IntVector();
  int user;
    
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

