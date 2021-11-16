/************************************************************************************
 CSC C85 - UTSC RoboSoccer image processing core

 Implementation of the image processing functionality for robo-soccer.
 This code initializes the web-cam, obtains a rectified playfield from
 the corners specified by the user, detects the bots and ball, and
 processes the user's keyboard input as well as the OpenGL display.

 You *DO NOT* have to look at the code here.

 However, you do want to read imageCapture.h to see what data is stored
 in the blob data structure. You will need to access it (at the very least
 you may want to use the direction vector stored in the bot's and opponent's
 blobs).

 * DO NOT SPEND TIME ON THIS CODE UNTIL YOU HAVE A MOSTLY WORKING
   AI CORE that at the very least can drive the bot toward the ball
   to kick it.

 For the curious - You can learn the following from studying code in this
  file:
 
  - Setting up, initializing, and using a webcam to get video frames
    (courtesy of the folks who created uvccapture)
  - Computing a mapping from points on a plane (the field of play) to
    points on a different plane (the image of the rectified field).
  - Processing an image frame to detect coloured blobs
  - Estimating basic blob properties
  - Updating an OpenGL interactive display

 Image processing code - F. Estrada, Summer 2013
 Updated - F. Estrada, Jul. 2019.

 Code for initializing the camera and grabbing a frame adapted                    
                    from luvcview and uvccapture.                                
                    luvcview and uvccapture by Laurent Pinchart & Michel Xhaard  
		    This code is distributed under GPL V2.0. Therefore:
                                                                                  
 As per the GLP V2.0 license terms. This code is hereby distributed freely. You   
  are free to modify and redistribute this code under the provisions of the      
  GPL V2 or newer. A copy of the license should be included with this code,      
  but should this not be the case, please consult                                 
  http://www.gnu.org/licenses/gpl-2.0.html                                       
************************************************************************************/

#include "imageCapture.h"
#include "svdDynamic.h"
#include "../roboAI.h"
#include <time.h>

// ** DO NOT MODIFY THESE GLOBAL VARIABLE DEFINITIONS **
// OpenGL data
GLuint texture;				// GL texture identifier 
int windowID;               		// Glut window ID (for display)
int Win[2];                 		// window (x,y) size
int sx,sy;				// Image size - set by the webcam init function

// Webcam image data and processed image components
struct vdIn *webcam;			// The input video device
struct image *proc_im;			// Image structure for processing
unsigned char *im;			// The current frame in RGB
unsigned char bigIm[1024*1024*3];	// Big texture to hold our image for OpenGL
unsigned char fieldIm[1024*768*3]; 	// Unwarped field 
unsigned char bgIm[1024*768*3];		// Background image

// Global image processing parameters
int gotbg=0;				// Background acquired flag
int toggleProc;				// Toggles processing on/off
double Mcorners[4][2];			// Manually specified corners for the field
double mcx,mcy;				// Crosshair x and y
int cornerIdx;				// Index for manual corner selection.
double thr=.58;				// Threshold for track detection 
double bgThresh=900;			// Background subtract threshold
double colThresh=.95;			// Saturation threshold
double colAngThresh=.985;		// Colour angle threshold
struct blob *blobs=NULL;		// Blob list for the current frame * DO NOT USE THIS LIST *
double *H = NULL;			// Homography matrix for field rectification
int frameNo=0;				// Frame id
time_t time1,time2;		    	// timing variables
int printFPS=0;				// Flag that controls FPS printout

// Robot-control data
struct RoboAI skynet;			// Bot's AI structure
int DIR_L=0, DIR_R=0, DIR_FWD=0, DIR_BACK=0;	// Toggle flags for manual robot motion
int doAI=0;				// Toggle switch for AI processing
int AIMode = 0;				// AI mode, as per command line
int botCol = 0;				// Own bot's colour as per command line
double adj_Y[2][2];			// Y coordinate adjustment for robot position
int got_Y=0;				// Got Y adjustment
// ** END OF GLOBAL DATA DECLARATIONS ** //

/*********************************************************************
Image processing setup and frame processing loop
**********************************************************************/
int imageCaptureStartup(char *devName, int rx, int ry, int own_col, int AI_mode)
{
 ///////////////////////////////////////////////////////////////////////////////////
 //
 // This is the entry point for the image processing code. It is called by 
 // roboSoccer.c
 //
 // Input args:
 //	- devName: Linux device name for the webcam, usually /dev/video0, or /dev/video1
 //	- rx, ry: Requested image resolution, typically 720, 1280
 //	- own_col: Color of the bot controlled by this program (passed on from command line)
 //	- AI_mode: Penalties, Follow the Ball, or Robo-Soccer (passed on from command line)
 //
 // This function performs the following tasks:
 //  - Initializes the webcam and opens the video input device
 //  - Sets up and opens an OpenGL window for image display
 //  - Initializes the AI data structure
 //  - Calls the image processing main loop
 //  
 // Returns:
 //     - Hopefully it doesn't! (glutMainLoop() exits without returning here)
 //	- But, -1 if there is a problem initializing the video capture device
 //	  or setting up OpenGL
 //
 ///////////////////////////////////////////////////////////////////////////////////
 toggleProc=0;
 memset(&bgIm[0],0,1024*768*3*sizeof(unsigned char));
 AIMode=AI_mode;
 botCol=own_col;

 fprintf(stderr,"Camera initialization!\n");
 // Initialize the webcam
 webcam=initCam(devName,rx,ry);
 if (webcam==NULL)
 {
  fprintf(stderr,"Unable to initialize webcam!\n");
  return -1;
 }
 sx=webcam->width;
 sy=webcam->height;
 fprintf(stderr,"Camera initialized! grabbing frames at %d x %d\n",sx,sy);

 // Done, set up OpenGL and call particle filter loop
 fprintf(stderr,"Entering main loop...\n");
 Win[0]=800;
 Win[1]=800;

 // Get initial time for FPS
 time(&time1);
 
 // Initialize the AI data structure with the requested mode
 setupAI(AIMode,botCol, &skynet);
 memset(&adj_Y[0][0],0,4*sizeof(double));

 initGlut(devName);
 glutMainLoop();

 return 0;
}

void FrameGrabLoop(void)
{
 ///////////////////////////////////////////////////////////////////
 //
 // This is the frame processing loop. It is called once per OpenGL
 // frame update, and does all the heavy lifting:
 //
 // - Obtains a video frame from the web cam
 // - Processes the video frame to extract the play field and 
 //   any blobs therein (see blobDetect2()).
 // - Calls the main AI processing function to allow your bot to
 //   plan and execute its actions based on the video data.
 // - Refreshes the video display - what gets displayed depends
 //   on whether the game is on, or whether the image processing
 //   setup is being carried out.
 //
 ////////////////////////////////////////////////////////////////////
  // OpenGL variables. Do not remove
  static int frame=0;
  char line[1024];
  int ox,oy,i,j;
  double *tmpH;
  unsigned char *big, *tframe;
  struct displayList *dp;
  struct image *t1, *t2, *t3;
  struct image *labIm, *blobIm;
  static int nblobs=0;
  FILE *f;

  /***************************************************
   Grab the current frame from the webcam
  ***************************************************/
  big=&bigIm[0];
  im=getFrame(webcam,sx,sy);
  ox=420;
  oy=1;
  t3=imageFromBuffer(im,sx,sy,3);
  
  /////////////////////////////////////////////////////////////////////////
  // What happens in this loop depends on a global variable that changes in
  // response to user keyboard commands. The variable 'toggleproc' has the
  // following values:
  // - 0 : Normal frame processing loop - Used during the game to get
  //                                      and update blob data.
  // - 1 : Used to load field calibration data (the H matrix and the
  //       background image)
  // - 2 : Used while capturing the 4 corners of the field during the
  //       field calibration stage
  /////////////////////////////////////////////////////////////////////////
  if (toggleProc==1)
  {
   // Load cached H matrix and background image
   f=fopen("Homography.dat","r");
   if (f!=NULL)
   {
    if (H!=NULL) free(H);
    H=(double *)calloc(9,sizeof(double));
    fread(H,9*sizeof(double),1,f);
    fread(&bgIm[0],1024*768*3*sizeof(unsigned char),1,f);
    fclose(f);
    gotbg=1;
    cornerIdx=4;
    fprintf(stderr,"Successfully read background and H matrix from file\n");
   }
   else
    fprintf(stderr,"No calibration data. Please use 'm' to capture corners\n");
   toggleProc=0;
  }
  else if (toggleProc==2)				
  {
    // Field calibration stage. Provide an on-screen crosshair to allow the user to
    // pick the 4 corners of the field. Once the corners are selected it computes
    // the H matrix and obtains a background image.
    if (H!=NULL) {free(H);H=NULL;}

    // Draw a cross-hair at the current cursor location
    drawCross_buf(mcx,mcy,0,255,64,15,im);
    drawCross_buf(mcx-1,mcy,0,255,64,15,im);
    drawCross_buf(mcx+1,mcy,0,255,64,15,im);
    drawCross_buf(mcx,mcy+1,0,255,64,15,im);
    drawCross_buf(mcx,mcy+1,0,255,64,15,im);
  }
  else
  {
   // toggleProc is zero - check if we should compute H and background image
   if (cornerIdx==4&&H==NULL)
   {
    // We have 4 corners but have not yet obtained the H matrix. Compute H and
    // obtain the background image. Cache the background along with the H
    // matrix for future use.
    fprintf(stderr,"Computing homography and acquiring background image\n");
    H=getH();

    // Get background image
    t2=newImage(t3->sx,t3->sy,3);
    for (i=0;i<25;i++)
    {
     tframe=getFrame(webcam,sx,sy);
     t1=imageFromBuffer(im,sx,sy,3);
     pointwise_add(t2,t1);
     deleteImage(t1);
     free(tframe);
    }
    image_scale(t2,1.0/25.0);
    fieldUnwarp(H,t2);
    deleteImage(t2);
    for (j=0;j<1024*768*3;j++) bgIm[j]=fieldIm[j];
    gotbg=1;
    time(&time1);
    frameNo=0;

    // Cache calibration data - Homography + background image
    f=fopen("Homography.dat","w");
    fwrite(H,9*sizeof(double),1,f);
    fwrite(&bgIm[0],1024*768*3*sizeof(unsigned char),1,f);
    fclose(f);
   }
  }

  ////////////////////////////////////////////////////////////////////
  // If we have a homography, detect blobs and call the AI routine
  ////////////////////////////////////////////////////////////////////
  labIm=blobIm=NULL;
  if (H!=NULL) 
  {
   ///////////////////////////////////////////////////////////////////
   //
   // If we have the H matrix, we must do the following:
   // - Rectify the playfield into a rectangle
   // - Perform background subtraction
   // - Detect colour blobs
   // - Call the AI main function to do its work
   // - Display the blobs along with information passed back from
   //   the AI processing code.
   //////////////////////////////////////////////////////////////////
   fieldUnwarp(H,t3);
   bgSubtract2();
   labIm=blobDetect2(fieldIm,1024,768,&blobs,&nblobs);
   if (blobs)
   {
    if (doAI==1) skynet.runAI(&skynet,blobs,NULL);
    else if (doAI==2) skynet.calibrate(&skynet,blobs);
    blobIm=renderBlobs(fieldIm,1024,768,labIm,blobs);
    // Render anything in the display list
    dp=skynet.DPhead;
    while (dp)
    {
      if (dp->type==0)
      {
        drawBox(dp->x1-2,dp->y1-2,dp->x1+2,dp->y1+2,dp->R,dp->G,dp->B,blobIm);
        drawBox(dp->x1-1,dp->y1-1,dp->x1+1,dp->y1+1,dp->R,dp->G,dp->B,blobIm);
        drawBox(dp->x1-0,dp->y1-0,dp->x1+0,dp->y1+0,dp->R,dp->G,dp->B,blobIm);
      }
      else
      {
        drawLine(dp->x1,dp->y1,dp->x2-dp->x1,dp->y2-dp->y1,1,dp->R,dp->G,dp->B,blobIm);
      }
      dp=dp->next;
    }
   }
   deleteImage(labIm);
  }
  
  ////////////////////////////////////////////////////////////////////
  // Render whatever we are going to display onto the texture image
  // buffer used by OpenGL
  //////////////////////////////////////////////////////////////////// 
  if (H==NULL)
  {
   // We still have not computed H. Display the video frame directly
   double ii,jj,dx,dy;
   dx=(double)sx/1023.0;
   dy=(double)sy/767.0;
#pragma omp parallel for schedule(dynamic,16) private(ii,jj,i,j)
   for (j=0;j<768;j++)
    for (i=0;i<1024;i++)
    {
     ii=i*dx;
     jj=j*dy;
     *(big+((i+((j+128)*1024))*3)+0)=*(im+(((int)ii+(((int)jj)*sx))*3)+0);
     *(big+((i+((j+128)*1024))*3)+1)=*(im+(((int)ii+(((int)jj)*sx))*3)+1);
     *(big+((i+((j+128)*1024))*3)+2)=*(im+(((int)ii+(((int)jj)*sx))*3)+2);
    }
  }
  else if (blobIm==NULL)
  {
   // We have calibration from H but no blob image (possible if no
   // agents are on the field at the moment or the image processing
   // thresholds are improperly set.
   // Copy the rectified, background subtracted field image for display
#pragma omp parallel for schedule(dynamic,16) private(i,j)
    for (j=0;j<768;j++)
     for (i=0;i<1024;i++)
     {
      *(big+(((i)+((j+128)*1024))*3)+0)=fieldIm[((i+(j*1024))*3)+0];
      *(big+(((i)+((j+128)*1024))*3)+1)=fieldIm[((i+(j*1024))*3)+1];
      *(big+(((i)+((j+128)*1024))*3)+2)=fieldIm[((i+(j*1024))*3)+2];
     }
  }
  else
  {
   // We have the H matrix and also detected blobs. Display the blob image
#pragma omp parallel for schedule(dynamic,16) private(i,j)
    for (j=0;j<768;j++)
     for (i=0;i<1024;i++)
     {
      *(big+(((i)+((j+128)*1024))*3)+0)=(unsigned char)((*(blobIm->layers[0]+i+(j*blobIm->sx))));
      *(big+(((i)+((j+128)*1024))*3)+1)=(unsigned char)((*(blobIm->layers[1]+i+(j*blobIm->sx))));
      *(big+(((i)+((j+128)*1024))*3)+2)=(unsigned char)((*(blobIm->layers[2]+i+(j*blobIm->sx))));
     }
    deleteImage(blobIm);
  }
  deleteImage(t3);	// Release the image obtained from the webcam for this round

  ///////////////////////////////////////////////////////////////////////////
  // Have OpenGL display our image for this frame
  ///////////////////////////////////////////////////////////////////////////
  // Clear the screen and depth buffers
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glEnable(GL_TEXTURE_2D);
  glDisable(GL_LIGHTING);

  if (frame==0)
  {
   glGenTextures( 1, &texture);
   glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
   glBindTexture( GL_TEXTURE_2D, texture);
   glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
   glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
   glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
   glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
   glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
   glTexImage2D (GL_TEXTURE_2D, 0, GL_RGB, 1024, 1024, 0, GL_RGB, GL_UNSIGNED_BYTE, big);
  }
  else
  {
   glBindTexture(GL_TEXTURE_2D, texture);
   glTexSubImage2D(GL_TEXTURE_2D,0,0,0,1024,1024,GL_RGB,GL_UNSIGNED_BYTE,big);
  }
  // Draw box bounding the viewing area
  glBegin (GL_QUADS);
  glTexCoord2f (0.0, 0.0);
  glVertex3f (0.0, 60.0, 0.0);
  glTexCoord2f (1.0, 0.0);
  glVertex3f (800.0, 60.0, 0.0);
  glTexCoord2f (1.0, 1.0);
  glVertex3f (800.0, 740.0, 0.0);
  glTexCoord2f (0.0, 1.0);
  glVertex3f (0.0, 740.0, 0.0);
  glEnd ();

  // Make sure all OpenGL commands are executed
  glFlush();
  // Swap buffers to enable smooth animation
  glutSwapBuffers();

  // Clean Up - Do all the image processing, AI, and planning before this code
  free(im);				// Release memory used by the current frame
  frame++;

  // Tell glut window to update ls itself
  glutSetWindow(windowID);
  glutPostRedisplay();
}

/////////////////////////////////////////////////////////////////////////////////////
// Field processing functions:
//   - Field un-warping
//   - Homography computation
//   - Background subtraction
/////////////////////////////////////////////////////////////////////////////////////
void fieldUnwarp(double *H, struct image *im)
{
 ////////////////////////////////////////////////////////////////////////////
 //
 // This takes the input frame and rectifies the playfield so that it is
 // rectangular and we can measure distances, directions, and velocities.
 //
 // It requires the homography matrix H computed from 4 user-selected
 // corner points on the input video frames that correspond to the 
 // four corners of the field. Corners must be in the following order:
 // 1 - top-left
 // 2 - top-right
 // 3 - bottom-right
 // 4 - bottom-left
 //
 // This code uses bi-linear interpolation during rectification.
 ////////////////////////////////////////////////////////////////////////////
 int i,j;
 double px,py,pw;
 double dx,dy;
 unsigned char *fi;
 double r1,g1,b1,r2,g2,b2,r3,g3,b3,r4,g4,b4;
 double R,G,B;
 
 if (H==NULL) return;

 fi=&fieldIm[0];
 memset(fi,0,1024*768*3*sizeof(unsigned char));

#pragma omp parallel for schedule(dynamic,16) private(i,j,px,py,pw,dx,dy,R,G,B,r1,g1,b1,r2,g2,b2,r3,g3,b3,r4,g4,b4)
 for (j=1;j<767;j++)
  for (i=1;i<1023;i++)
  {
   // Obtain coordinates for this pixel in the unwarped image
   px=((*(H+0))*i) + ((*(H+1))*j) + (*(H+2));
   py=((*(H+3))*i) + ((*(H+4))*j) + (*(H+5));
   pw=((*(H+6))*i) + ((*(H+7))*j) + (*(H+8));
   px=px/pw;
   py=py/pw;
   dx=px-(int)px;
   dy=py-(int)py;
   if (px>0&&px<im->sx-1&&py>0&&py<im->sy-1)
   {
    R=G=B=0;
    r1=*(im->layers[0]+ ((int)px) + (((int)py) * im->sx));
    g1=*(im->layers[1]+ ((int)px) + (((int)py) * im->sx));
    b1=*(im->layers[2]+ ((int)px) + (((int)py) * im->sx));
    r2=*(im->layers[0]+ ((int)px+1) + (((int)py) * im->sx));
    g2=*(im->layers[1]+ ((int)px+1) + (((int)py) * im->sx));
    b2=*(im->layers[2]+ ((int)px+1) + (((int)py) * im->sx));
    r3=*(im->layers[0]+ ((int)px) + (((int)py+1) * im->sx));
    g3=*(im->layers[1]+ ((int)px) + (((int)py+1) * im->sx));
    b3=*(im->layers[2]+ ((int)px) + (((int)py+1) * im->sx));
    r4=*(im->layers[0]+ ((int)px+1) + (((int)py+1) * im->sx));
    g4=*(im->layers[1]+ ((int)px+1) + (((int)py+1) * im->sx));
    b4=*(im->layers[2]+ ((int)px+1) + (((int)py+1) * im->sx));
    r1=((1.0-dx)*r1)+(dx*r2);
    g1=((1.0-dx)*g1)+(dx*g2);
    b1=((1.0-dx)*b1)+(dx*b2);
    r3=((1.0-dx)*r3)+(dx*r4);
    g3=((1.0-dx)*g3)+(dx*g4);
    b3=((1.0-dx)*b3)+(dx*b4);
    R=((1.0-dy)*r1)+(dy*r3);
    G=((1.0-dy)*g1)+(dy*g3);
    B=((1.0-dy)*b1)+(dy*b3);
    *(fi+((i+(j*1024))*3)+0)=(unsigned char)(R);
    *(fi+((i+(j*1024))*3)+1)=(unsigned char)(G);
    *(fi+((i+(j*1024))*3)+2)=(unsigned char)(B);  
   }
  }

}

double *getH(void)
{
 //////////////////////////////////////////////////////////////////////
 //
 // This function estimates the homography matrix H from the 4 corner
 // coordinates input by the user. These 4 corners are made to 
 // correspond to the 4 corners of the displayed rectangular image.
 //
 //////////////////////////////////////////////////////////////////////
 double cD[4],tcorners[4][2];
 double corners2[4][2]={1.0,1.0,1023.0,1.0,1023.0,767.0,1.0,767.0};
 double A[8][9],B[8][8],b[8],Binv[8][8];
 double *U,*s,*V,*rv1;
 double *H;
 int i,j;

 for (i=0;i<4;i++)
 {
  A[(2*i)+0][0]=0;
  A[(2*i)+0][1]=0;
  A[(2*i)+0][2]=0;
  A[(2*i)+0][3]=-corners2[i][0];
  A[(2*i)+0][4]=-corners2[i][1];
  A[(2*i)+0][5]=-1;
  A[(2*i)+0][6]=corners2[i][0]*Mcorners[i][1];
  A[(2*i)+0][7]=corners2[i][1]*Mcorners[i][1];
  A[(2*i)+0][8]=Mcorners[i][1];
  A[(2*i)+1][0]=corners2[i][0];
  A[(2*i)+1][1]=corners2[i][1];
  A[(2*i)+1][2]=1;
  A[(2*i)+1][3]=0;
  A[(2*i)+1][4]=0;
  A[(2*i)+1][5]=0;
  A[(2*i)+1][6]=-corners2[i][0]*Mcorners[i][0];
  A[(2*i)+1][7]=-corners2[i][1]*Mcorners[i][0];
  A[(2*i)+1][8]=-Mcorners[i][0];
 }

 // Allocate memory for the homography!
 H=(double *)calloc(9,sizeof(double));	// H should be the last row of U

 U=NULL;
 s=NULL;
 V=NULL;
 rv1=NULL;

 SVD(&A[0][0],8,9,&U,&s,&V,&rv1);
 // A note on sizes: A is 8x9, SVD should return U is 8x8, S is 8x9 
 // (or in this case a vector with 9 values), and V is 9x9. However,
 // Tom's code returns an 8x8 U, and a 9x8 V! missing last column

 // Now, because V is not all there, we have to solve for the eigenvector in V
 // that lies along the null-space of A. It has to be orthogonal to the 8
 // eigenvectors we just got in V. It also has 9 entries but fortunately we
 // can fix the last one to 1. That leaves an 8x8 non-homogeneous system we
 // can solve directly for.

 for (i=0;i<8;i++)
  for (j=0; j<8; j++)
   B[i][j]=(*(V+i+(j*8)));
 for (j=0; j<8; j++)
  b[j]=(-(*(V+j+64)));

 free(U);
 free(s);
 free(V);

 // Compute B^-1
 U=NULL;
 s=NULL;
 V=NULL;
 rv1=NULL;
 SVD(&B[0][0],8,8,&U,&s,&V,&rv1);
 InvertMatrix(U,s,V,8,&Binv[0][0]);
 free(U);
 free(s);
 free(V);

 // Finally, compute the eigenvector's first 8 entries
 for (i=0;i<8;i++)
 {
  *(H+i)=0;
  for (j=0; j<8;j++) (*(H+i))+=Binv[i][j]*b[j];
 }
 *(H+8)=1.0;
 
 fprintf(stderr,"Homography martrix:\n");
 fprintf(stderr,"hh=[%f %f %f\n",*(H),*(H+1),*(H+2));
 fprintf(stderr,"%f %f %f\n",*(H+3),*(H+4),*(H+5));
 fprintf(stderr,"%f %f %f\n];\n",*(H+6),*(H+7),*(H+8));
 
 for (i=0;i<4;i++)
 {
  memset(&cD[0],0,4*sizeof(double));
  cD[0]=((*(H+0))*corners2[i][0])+((*(H+1))*corners2[i][1])+((*(H+2)));
  cD[1]=((*(H+3))*corners2[i][0])+((*(H+4))*corners2[i][1])+((*(H+5)));
  cD[2]=((*(H+6))*corners2[i][0])+((*(H+7))*corners2[i][1])+((*(H+8)));
  fprintf(stderr,"Converts (%f,%f) to (%f,%f)\n",corners2[i][0],corners2[i][1],\
          cD[0]/cD[2],cD[1]/cD[2]);
 }
 return(H);
}

void bgSubtract2(void)
{
 ///////////////////////////////////////////////////////////////////////////////
 //
 // This function performs background subtraction.
 //
 // Pixels that are zeroed out include:
 //   - Any whose difference w.r.t. background is less than the specified bgThress (which is
 //     user controlled through the GUI)
 //   - Any whose saturation value is less than a the specified threshold (colThresh, also
 //     controlled via the GUI)
 //
 ///////////////////////////////////////////////////////////////////////////////

 int j,i;
 double r,g,b,R,G,B,dd,mg;
 double H,S,V;
 double imT[1024][768];
 char line[1024];
 FILE *f;
 
 if (!gotbg) return;
#pragma omp parallel for schedule(dynamic,16) private(i,j,r,g,b,R,G,B,dd,mg)
 for (j=0;j<768;j++)
  for (i=0; i<1024; i++)
  {
   r=fieldIm[((i+(j*1024))*3)+0];
   g=fieldIm[((i+(j*1024))*3)+1];
   b=fieldIm[((i+(j*1024))*3)+2];
   R=bgIm[((i+(j*1024))*3)+0];
   G=bgIm[((i+(j*1024))*3)+1];
   B=bgIm[((i+(j*1024))*3)+2];
   
   // HSV conversion
   if (r>g&&r>b) V=r; else if (g>b) V=g; else V=b;
   if (V==0) S=0; else if (r<g&&r<b) S=(V-r)/V; else if (g<b) S=(V-g)/V; else S=(V-b)/V;
   
   imT[i][j]=S;
   
   // Compute magnitude of difference w.r.t. background image
   dd=(r-R)*(r-R);
   dd+=(g-G)*(g-G);
   dd+=(b-B)*(b-B);
   mg=(r+g+b)/3.0;

   // Zero out background pixels and pixels that are not saturated (everything except uniforms/ball)
   if (dd<bgThresh||S<colThresh)
   {  
    fieldIm[((i+(j*1024))*3)+0]=0;
    fieldIm[((i+(j*1024))*3)+1]=0;
    fieldIm[((i+(j*1024))*3)+2]=0;
   }
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////
//
// Blob detection and rendering
//
///////////////////////////////////////////////////////////////////////////////////////////////////
void releaseBlobs(struct blob *blobList)
{
 // Deletes a linked list of blobs
 struct blob *p,*q;
 p=blobList;
 while(p)
 {
  q=p->next;
  free(p);
  p=q;
 }
}

void rgb2hsv(double R, double G, double B, double *H, double *S, double *V)
{
  // Return the HSV components of the input RGB colour, R,G, and B in [0,1]
  double C;
  
  if (R>G&&R>B) *V=R; else if (G>B) *V=G; else *V=B;
  if (R<G&&R<B) C=((*V)-R); else if (G<B) C=((*V)-G); else C=((*V)-B); 
  if (C==0) *H=0;      // Actually it is undefined, but this should never be allowed to happen
  else if (*V==R) *H=(G-B)/C;
  else if (*V==G) *H=((B-R)/C)+2.0;
  else *H=((R-G)/C)+4.0;
  *H=(*H)*60;
  *H=fmod(*H,360);
  *H=2*PI*(*H)/360.0;
  if (*V==0) *S=0;  // Once more, this should not happen
  else *S=C/(*V);
}

struct image *blobDetect2(unsigned char *fgIm, int sx, int sy, struct blob **blob_list, int *nblobs)
{
 /////////////////////////////////////////////////////////////////////////////////////////////////
 //
 // This function does blob detection on the rectified field image (after background subtraction)
 // and generates:
 // - A label image, with a unique label for pixels in each blob (sx * sy * 1 layer)
 // - A list of blob data structures with suitably estimated values (though note that some of
 //   the blob data values are filled-in by the AI code later on)
 // - The number of blobs found
 // 
 // NOTE 1: This function will ignore tiny blobs
 // NOTE 2: The list of blobs is created from scratch for each frame - blobs do not persist
 /////////////////////////////////////////////////////////////////////////////////////////////////

 static int pixStack[1024*768*2];
 int i,j,x,y;
 int mix,miy,mx,my;
 int lab;
 double R,G,B;
 double H,S,V,C;
 double Hx,Hy;
 double tH,tS,tV,tC;
 double tHx,tHy;
 double Hacc,Sacc,Vacc;
 double Ra,Ga,Ba;
 double xc,yc, oy1, oy2;
 int pixcnt;
 int *stack;
 int stackPtr;
 struct image *labIm, *tmpIm, *tmpIm2;
 struct blob *bl;
 double cov[2][2],T,D,L1,L2;
 char line[1024];
 struct kernel *kern;

 kern=GaussKernel(2);
 
 // Clear any previous list of blobs
 if (*(blob_list)!=NULL)
 {
  releaseBlobs(*(blob_list));
  *(blob_list)=NULL;
 }

 // Assumed: Pixels in the input fgIm that have non-zero RGB values are foreground
 labIm=newImage(sx,sy,1);				// 1-layer labels image
 tmpIm=imageFromBuffer(fgIm,sx,sy,3);

 // Filter background subtracted, saturation thresholded map to make smoother blobs
 tmpIm2=convolve_x(tmpIm,kern);
 deleteImage(tmpIm);
 tmpIm=convolve_y(tmpIm2,kern);
 deleteImage(tmpIm2);

 stack=&pixStack[0];
 lab=1;		
 memset(stack,0,1024*768*2*sizeof(int));

 // Visit each pixel and try to grow a blob from it if it has a non-zero value - this uses simple floodfill
 for (j=0;j<sy;j++)
  for (i=0;i<sx;i++)
  {
   R=*(tmpIm->layers[0]+i+(j*sx));
   G=*(tmpIm->layers[1]+i+(j*sx));
   B=*(tmpIm->layers[2]+i+(j*sx));
   Hacc=0;
   Sacc=0;
   Vacc=0;
      
   if (R+G+B>0)    // Found unlabeled pixel
   {       
    // Obtain HSV components and color vector
    rgb2hsv(R/255.0,G/255.0,B/255.0,&H,&S,&V);
    Hx=cos(H);
    Hy=sin(H);
    
    stackPtr=1;
    *(stack+(2*stackPtr))=i;
    *(stack+(2*stackPtr)+1)=j;
    *(tmpIm->layers[0]+i+(j*sx)) = -(*(tmpIm->layers[0]+i+(j*sx)));
    *(tmpIm->layers[1]+i+(j*sx)) = -(*(tmpIm->layers[1]+i+(j*sx)));
    *(tmpIm->layers[2]+i+(j*sx)) = -(*(tmpIm->layers[2]+i+(j*sx)));
    Ra=0;
    Ga=0;
    Ba=0;
    xc=0;
    yc=0;
    mix=10000;
    miy=10000;
    mx=-10000;
    my=-10000;
    pixcnt=0;
    while (stackPtr>0)
    {
     if (stackPtr>=1024*768) fprintf(stderr," *** Busted the stack!\n");
     x=*(stack+(2*stackPtr));
     y=*(stack+(2*stackPtr)+1);
     stackPtr--;
     *(labIm->layers[0]+x+(y*labIm->sx))=lab;
     Ra-=*(tmpIm->layers[0]+x+(y*tmpIm->sx));
     Ga-=*(tmpIm->layers[1]+x+(y*tmpIm->sx));
     Ba-=*(tmpIm->layers[2]+x+(y*tmpIm->sx));
     xc+=x;
     yc+=y;
     pixcnt++;
     *(tmpIm->layers[0]+x+(y*sx)) = 0;
     *(tmpIm->layers[1]+x+(y*sx)) = 0;
     *(tmpIm->layers[2]+x+(y*sx)) = 0;
     if (mix>x) mix=x;
     if (miy>y) miy=y;
     if (mx<x) mx=x;
     if (my<y) my=y;
     // Check neighbours
     if (y>0)
     {
      R=*(tmpIm->layers[0]+x+((y-1)*sx)); 
      G=*(tmpIm->layers[1]+x+((y-1)*sx)); 
      B=*(tmpIm->layers[2]+x+((y-1)*sx)); 
      if (R+G+B>0)
      {
       rgb2hsv(R/255.0,G/255.0,B/255.0,&tH,&tS,&tV);
       tHx=cos(tH);
       tHy=sin(tH);
       Hacc+=tH;
       Sacc+=tS;
       Vacc+=tV;

       if (fabs((tHx*Hx)+(tHy*Hy))>colAngThresh)
       {
        stackPtr++;
        *(stack+(2*stackPtr))=x;
        *(stack+(2*stackPtr)+1)=y-1;
        *(tmpIm->layers[0]+x+((y-1)*sx)) *= -1.0;
        *(tmpIm->layers[1]+x+((y-1)*sx)) *= -1.0;
        *(tmpIm->layers[2]+x+((y-1)*sx)) *= -1.0;
       }

      }   
     }
     if (x<sx-1)
     {
      R=*(tmpIm->layers[0]+x+1+(y*sx)); 
      G=*(tmpIm->layers[1]+x+1+(y*sx)); 
      B=*(tmpIm->layers[2]+x+1+(y*sx)); 
      if (R+G+B>0)
      {
       rgb2hsv(R/255.0,G/255.0,B/255.0,&tH,&tS,&tV);
       tHx=cos(tH);
       tHy=sin(tH);
       Hacc+=tH;
       Sacc+=tS;
       Vacc+=tV;

       if (fabs((tHx*Hx)+(tHy*Hy))>colAngThresh)
       {
        stackPtr++;
        *(stack+(2*stackPtr))=x+1;
        *(stack+(2*stackPtr)+1)=y;
        *(tmpIm->layers[0]+x+1+(y*sx)) *= -1.0;
        *(tmpIm->layers[1]+x+1+(y*sx)) *= -1.0;
        *(tmpIm->layers[2]+x+1+(y*sx)) *= -1.0;
       }
      }
     }
     if (y<sy-1)
     {
      R=*(tmpIm->layers[0]+x+((y+1)*sx)); 
      G=*(tmpIm->layers[1]+x+((y+1)*sx)); 
      B=*(tmpIm->layers[2]+x+((y+1)*sx)); 
      if (R+G+B>0)
      {
       rgb2hsv(R/255.0,G/255.0,B/255.0,&tH,&tS,&tV);
       tHx=cos(tH);
       tHy=sin(tH);
       Hacc+=tH;
       Sacc+=tS;
       Vacc+=tV;

       if (fabs((tHx*Hx)+(tHy*Hy))>colAngThresh)
       {
        stackPtr++;
        *(stack+(2*stackPtr))=x;
        *(stack+(2*stackPtr)+1)=y+1;
        *(tmpIm->layers[0]+x+((y+1)*sx)) *= -1.0;
        *(tmpIm->layers[1]+x+((y+1)*sx)) *= -1.0;
        *(tmpIm->layers[2]+x+((y+1)*sx)) *= -1.0;
       }
      }
     }
     if (x>0)
     {
      R=*(tmpIm->layers[0]+x-1+(y*sx)); 
      G=*(tmpIm->layers[1]+x-1+(y*sx)); 
      B=*(tmpIm->layers[2]+x-1+(y*sx)); 
      if (R+G+B>0)
      {
       rgb2hsv(R/255.0,G/255.0,B/255.0,&tH,&tS,&tV);
       tHx=cos(tH);
       tHy=sin(tH);
       Hacc+=tH;
       Sacc+=tS;
       Vacc+=tV;

       if (fabs((tHx*Hx)+(tHy*Hy))>colAngThresh)
       {
        stackPtr++;
        *(stack+(2*stackPtr))=x-1;
        *(stack+(2*stackPtr)+1)=y;
        *(tmpIm->layers[0]+x-1+(y*sx)) *= -1.0;
        *(tmpIm->layers[1]+x-1+(y*sx)) *= -1.0;
        *(tmpIm->layers[2]+x-1+(y*sx)) *= -1.0;
       }
      }
     }
    }	// End while
    
    if (pixcnt>250)         // Ignore blobs smaller than 250 pixels in size
    {
     // If the size of the blob is greater than a small threshold, inset it in the
     // blob list and fill-out the blob data structure.
     Ra/=pixcnt;            // Average RGB values
     Ga/=pixcnt;
     Ba/=pixcnt;
     xc/=pixcnt;            // Centroid of the blob
     yc/=pixcnt;
     Hacc/=pixcnt;          // Average HSV values
     Sacc/=pixcnt;
     Vacc/=pixcnt;
     bl=(struct blob *)calloc(1,sizeof(struct blob));
     bl->label=lab;         // Label for this blob (a unique int ID, not related to color)
     bl->vx=0;              // Velocity and motion vectors *THESE ARE UPDATED BY THE AI*
     bl->vy=0;
     bl->mx=0;
     bl->my=0;
     bl->cx=xc;             // Blob center location
     bl->cy=yc;
     bl->size=pixcnt;       // Size in pixels
     bl->x1=mix;            // Bounding box (top-left -> bottom-right)
     bl->y1=miy;
     bl->x2=mx;
     bl->y2=my;
     bl->R=Ra;              // Blob RGB color
     bl->G=Ga;
     bl->B=Ba;
     bl->H=Hacc;            // Blob HSV color
     bl->S=Sacc;
     bl->V=Vacc;
     bl->next=NULL;
     bl->idtype=0;          // Set by the AI to indicate if this blob is an agent, and which agent it is

     // Insert into linked list of detected blobs.
     if (*(blob_list)==NULL) *(blob_list)=bl;
     else {bl->next=(*(blob_list))->next; (*(blob_list))->next=bl;}
    }
    lab++;
   }    // End if
  }   // End for i

 deleteImage(tmpIm);

 // Count number of blobs found
 bl=*blob_list;
 *(nblobs)=0;
 while (bl!=NULL)
 {
  *nblobs=(*nblobs) + 1;  
  bl=bl->next;
 }

 // Compute blob direction for each blob
 bl=*blob_list;
 while (bl!=NULL)
 {
  memset(&cov[0][0],0,4*sizeof(double));
  for (j=bl->y1;j<=bl->y2;j++)
   for (i=bl->x1;i<=bl->x2;i++)
    if (*(labIm->layers[0]+i+(j*labIm->sx))==bl->label)
    {
     xc=i-bl->cx;
     yc=j-bl->cy;
     cov[0][0]+=(xc*xc);
     cov[1][1]+=(yc*yc);
     cov[0][1]+=(yc*xc);
     cov[1][0]+=(xc*yc);
    }
  cov[0][0]/=bl->size;
  cov[0][1]/=bl->size;
  cov[1][0]/=bl->size;
  cov[1][1]/=bl->size;
  T=cov[0][0]+cov[1][1];
  D=(cov[0][0]*cov[1][1])-(cov[1][0]*cov[0][1]);
  L1=(.5*T)+sqrt(((T*T)/4)-D);
  L2=(.5*T)-sqrt(((T*T)/4)-D);
  if (fabs(L1)>fabs(L2))
  {
   bl->dx=L1-cov[1][1];
   bl->dy=cov[1][0];
  }
  else
  {
   bl->dx=L2-cov[1][1];
   bl->dy=cov[1][0];
  } 
  T=sqrt((bl->dx*bl->dx)+(bl->dy*bl->dy));
  bl->dx/=T;
  bl->dy/=T;

  // Finally, if we have offset correction data, store it in the blob
  if (got_Y==3)
   memcpy(&bl->adj_Y[0][0],&adj_Y[0][0],4*sizeof(double));
  else
   memset(&bl->adj_Y[0][0],0,4*sizeof(double));

  bl=bl->next;
 }
 
 deleteKernel(kern);
 
 return(labIm);
} 

struct image *renderBlobs(unsigned char *fgIm, int sx, int sy, struct image *labels, struct blob *list)
{
 //////////////////////////////////////////////////////////////////////////////////////////////
 //
 // This function renders an image with the blobs and geometric information provided by the
 // AI code (provided the AI is toggled-on and has estimated the corresponding values for
 // the blobs).
 //
 // - It renders each blob as a uniform-colored region
 // - It draws bounding boxes
 // - It draws crosshairs at the centers of blobs (or perspective-corrected crosshairs
 //   indicating the estimated location of the bots)
 // - It draws heading and orientation vectors
 //
 // NOTE: This function is also in charge of updating the calibration data for perspective
 //       projection error correction while the user is calibrating for Y offset error.
 //////////////////////////////////////////////////////////////////////////////////////////////

 int i,j;
 struct blob *p;
 struct image *blobIm;
 double cR,cG,cB;
 double *labRGB;
 int maxLab,lab;
 FILE *f;

 if (list==NULL) return(NULL);

 // Find maximum label for this round
 maxLab=-1;
 p=list;
 while (p!=NULL) {if (p->label>maxLab) maxLab=p->label; p=p->next;}

 labRGB=(double *)calloc(3*(maxLab+1),sizeof(double));
 if (labRGB==NULL) return(NULL);
 p=list;
 while (p!=NULL)
 {
  if (*(labRGB+((p->label)*3)+0)==0)
  {
   *(labRGB+((p->label)*3)+0)=p->R;
   *(labRGB+((p->label)*3)+1)=p->G;
   *(labRGB+((p->label)*3)+2)=p->B;
  }
  p=p->next;
 }

 blobIm=imageFromBuffer(fgIm,sx,sy,3);
 // Uniform coloured blobs - colour is average colour of the corresponding region in the image  
#pragma omp parallel for schedule(dynamic,32) private(i,j)
 for (j=0;j<sy;j++)
  for (i=0;i<sx;i++)
  {
   if (*(labels->layers[0]+i+(j*labels->sx))!=0)
   {
    lab=*(labels->layers[0]+i+(j*labels->sx));
    *(blobIm->layers[0]+i+(j*blobIm->sx))=*(labRGB+(3*lab)+0);
    *(blobIm->layers[1]+i+(j*blobIm->sx))=*(labRGB+(3*lab)+1);
    *(blobIm->layers[2]+i+(j*blobIm->sx))=*(labRGB+(3*lab)+2);
   }
  }

 // Draw bounding boxes & cross-hairs 
 p=list;
 while (p!=NULL)
 {
  if (p->idtype>0)
  {
   if (p->idtype==3)		// This blob is the ball
   {
    cR=255.0;
    cG=255.0;
    cB=32.0;
    drawLine(p->cx-1,p->cy,p->mx,p->my,55,cR,cG,cB,blobIm);
    drawLine(p->cx,p->cy-1,p->mx,p->my,55,cR,cG,cB,blobIm);
    drawLine(p->cx+1,p->cy,p->mx,p->my,55,cR,cG,cB,blobIm);
    drawLine(p->cx,p->cy+1,p->mx,p->my,55,cR,cG,cB,blobIm);
    drawLine(p->cx,p->cy,p->mx,p->my,55,cR,cG,cB,blobIm);
    cR=32.0;
    cG=32.0;
    cB=255.0;
   }
   else if (p->idtype==1)	// This blob is our own bot
   {
    cR=255.0;
    cG=255.0;
    cB=32.0;
    drawLine(p->cx-1,p->cy,p->mx,p->my,55,cR,cG,cB,blobIm);
    drawLine(p->cx,p->cy-1,p->mx,p->my,55,cR,cG,cB,blobIm);
    drawLine(p->cx+1,p->cy,p->mx,p->my,55,cR,cG,cB,blobIm);
    drawLine(p->cx,p->cy+1,p->mx,p->my,55,cR,cG,cB,blobIm);
    drawLine(p->cx,p->cy,p->mx,p->my,55,cR,cG,cB,blobIm);
    cR=32.0;
    cG=255.0;
    cB=32.0;
    if (got_Y==1&&fabs(adj_Y[0][0])<.1)
     adj_Y[0][0]=p->cy;
    if (got_Y==2&&fabs(adj_Y[1][0])<.1)
     adj_Y[1][0]=p->cy;
   }
   else if (p->idtype==2)	// This blob is the opponent
   {
    cR=255.0;
    cG=255.0;
    cB=32.0;
    drawLine(p->cx-1,p->cy,p->mx,p->my,55,cR,cG,cB,blobIm);
    drawLine(p->cx,p->cy-1,p->mx,p->my,55,cR,cG,cB,blobIm);
    drawLine(p->cx+1,p->cy,p->mx,p->my,55,cR,cG,cB,blobIm);
    drawLine(p->cx,p->cy+1,p->mx,p->my,55,cR,cG,cB,blobIm);
    drawLine(p->cx,p->cy,p->mx,p->my,55,cR,cG,cB,blobIm);
    cR=255.0;
    cG=32.0;
    cB=32.0;
    if (got_Y==1&&fabs(adj_Y[0][1])<.1)
     adj_Y[0][1]=p->cy;
    if (got_Y==2&&fabs(adj_Y[1][1])<.1)
     adj_Y[1][1]=p->cy;
   }
   drawBox(p->x1-1,p->y1,p->x2-1,p->y2,cR,cG,cB,blobIm);
   drawBox(p->x1+1,p->y1,p->x2+1,p->y2,cR,cG,cB,blobIm);
   drawBox(p->x1,p->y1-1,p->x2,p->y2-1,cR,cG,cB,blobIm);
   drawBox(p->x1,p->y1+1,p->x2,p->y2+1,cR,cG,cB,blobIm);
   drawBox(p->x1,p->y1,p->x2,p->y2,cR,cG,cB,blobIm);

   drawCross(p->cx-1,p->cy,64,64,64,15,blobIm);
   drawCross(p->cx+1,p->cy,64,64,64,15,blobIm);
   drawCross(p->cx,p->cy-1,128,128,128,15,blobIm);
   drawCross(p->cx,p->cy+1,128,128,128,15,blobIm);
   drawCross(p->cx,p->cy,255,255,255,15,blobIm);

   drawLine(p->cx-1,p->cy,p->dx,p->dy,55,255,255,255,blobIm);
   drawLine(p->cx,p->cy-1,p->dx,p->dy,55,255,255,255,blobIm);
   drawLine(p->cx+1,p->cy,p->dx,p->dy,55,255,255,255,blobIm);
   drawLine(p->cx,p->cy+1,p->dx,p->dy,55,255,255,255,blobIm);
   drawLine(p->cx,p->cy,p->dx,p->dy,55,255,255,255,blobIm);
  }
  p=p->next;
 }

 if (got_Y==2)
 {
  fprintf(stderr,"Storing offset data. Own bot:%f,%f ; opponent: %f,%f\n",adj_Y[0][0],adj_Y[1][0],adj_Y[0][1],adj_Y[1][1]);
  f=fopen("offsets.dat","w");
  fwrite(&adj_Y[0][0],4*sizeof(double),1,f);
  fclose(f);
  got_Y=3;				// Complete! we have offset calibration data
  doAI=0;				// End calibration loop
 }

 free(labRGB);
 return(blobIm);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// A few miscellaneous drawing functions to overlay stuff onto images
// We can draw: lines, boxes, and crosses onto images stored in an image data structure
//              crosses onto an image stored as a framebuffer
////////////////////////////////////////////////////////////////////////////////////////////////////////////
void drawLine(int x1, int y1, double vx, double vy, double scale, double R, double G, double B, struct image *dst)
{
 int i,j;
 double len;
 double l;

 len=sqrt((vx*vx)+(vy*vy));
 vx=vx/len;
 vy=vy/len;

 for (l=0;l<=len*scale;l++)
 {
  i=(int)(x1+(l*vx));
  j=(int)(y1+(l*vy));
  if (i>=0&&i<dst->sx&&j>=0&&j<dst->sy)
  {
   *(dst->layers[0]+i+(j*dst->sx))=R;
   *(dst->layers[1]+i+(j*dst->sx))=G;
   *(dst->layers[2]+i+(j*dst->sx))=B;
  }
 }

}
 
void drawBox(int x1, int y1, int x2, int y2, double R, double G, double B, struct image *dst)
{
 int i,j;

 for (i=x1;i<=x2;i++)
 {
  if (i>=0&&i<dst->sx)
  {
   if (y1>=0&&y1<dst->sy)
   {
    *(dst->layers[0]+i+(y1*dst->sx))=R;
    *(dst->layers[1]+i+(y1*dst->sx))=G;
    *(dst->layers[2]+i+(y1*dst->sx))=B;
   }
   if (y2>=0&&y2<dst->sy)
   {
    *(dst->layers[0]+i+(y2*dst->sx))=R;
    *(dst->layers[1]+i+(y2*dst->sx))=G;
    *(dst->layers[2]+i+(y2*dst->sx))=B;
   }
  } 
 }
 for (j=y1;j<=y2;j++)
 {
  if (j>=0&&j<dst->sy)
  {
   if (x1>=0&&x1<dst->sx)
   {
    *(dst->layers[0]+x1+(j*dst->sx))=R;
    *(dst->layers[1]+x1+(j*dst->sx))=G;
    *(dst->layers[2]+x1+(j*dst->sx))=B;
   }
   if (x2>=0&&x2<dst->sx)
   {
    *(dst->layers[0]+x2+(j*dst->sx))=R;
    *(dst->layers[1]+x2+(j*dst->sx))=G;
    *(dst->layers[2]+x2+(j*dst->sx))=B;
   }
  }
 }

}

void drawCross(int mcx, int mcy, double R, double G, double B, int len, struct image *dst)
{
 int i,j;

 for (j=-len; j<len; j++)
  if (mcy+j>=0&&mcy+j<dst->sy&&j!=0) 
  {
   *(dst->layers[0]+mcx+((mcy+j)*dst->sx))=R;
   *(dst->layers[1]+mcx+((mcy+j)*dst->sx))=G;
   *(dst->layers[2]+mcx+((mcy+j)*dst->sx))=B;
  }
 for (i=-len; i<len; i++)
  if (mcx+i>=0&&mcx+i<dst->sx&&i!=0)
  {
   *(dst->layers[0]+mcx+i+(mcy*dst->sx))=R;
   *(dst->layers[1]+mcx+i+(mcy*dst->sx))=G;
   *(dst->layers[2]+mcx+i+(mcy*dst->sx))=B;
  }
}

void drawCross_buf(int mcx, int mcy, double R, double G, double B, int len, unsigned char *dst)
{
 int i,j;

 for (j=-len; j<len; j++)
  if (mcy+j>=0&&mcy+j<sy&&j!=0) 
  {
   *(dst+((mcx+((mcy+j)*sx))*3)+0)=R;
   *(dst+((mcx+((mcy+j)*sx))*3)+1)=G;
   *(dst+((mcx+((mcy+j)*sx))*3)+2)=B;
  }
 for (i=-len; i<len; i++)
  if (mcx+i>=0&&mcx+i<sx&&i!=0)
  {
   *(dst+((mcx+i+(mcy*sx))*3)+0)=R;
   *(dst+((mcx+i+(mcy*sx))*3)+1)=G;
   *(dst+((mcx+i+(mcy*sx))*3)+2)=B;
  }
}


/*********************************************************************
End of frame processing functions
**********************************************************************/

/*********************************************************************
 OpenGL display setup.
*********************************************************************/
// Initialize glut and create a window with the specified caption
void initGlut(char* winName)
{
 //////////////////////////////////////////////////////////////////////
 //
 // This function sets up the OpenGL display window, and informs GLUT
 // of the names of the callbacks to be used to refresh the display
 // for each frame, to handle window changes, and to handle keyboard
 // input.
 //
 //////////////////////////////////////////////////////////////////////

 // Set video mode: double-buffered, color, depth-buffered
 glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);

 // Create window
 glutInitWindowPosition (0, 0);
 glutInitWindowSize(Win[0],Win[1]);
 windowID = glutCreateWindow(winName);

 // Setup callback functions to handle window-related events.
 // In particular, OpenGL has to be informed of which functions
 // to call when the image needs to be refreshed, and when the
 // image window is being resized.
 glutReshapeFunc(WindowReshape);   // Call WindowReshape whenever window resized
 glutDisplayFunc(FrameGrabLoop);   // Main display function is also the main loop
 glutKeyboardFunc(kbHandler);
}

void kbHandler(unsigned char key, int x, int y)
{ 
 /////////////////////////////////////////////////////////////////
 //
 // This is the GLUT keyboard handler. It is a callback triggered by
 // keyboard input while the OpenGL windows is in-focus. The
 // function then performs the function required for each 
 // of the valid keyboard commands.
 //
 /////////////////////////////////////////////////////////////////
 FILE *f;
 struct displayList *q;
 // Exit!
 if (key=='q') 
 {
  BT_all_stop(0);
  releaseBlobs(blobs);
  deleteImage(proc_im);
  glDeleteTextures(1,&texture);
  closeCam(webcam);
  while (skynet.DPhead!=NULL)
  {
    q=skynet.DPhead->next;
    free(skynet.DPhead);
    skynet.DPhead=q;
  }
  exit(0);
 }

 // Toggle AI processing on/off
 if (key=='t') if (doAI==1) doAI=0; else if (doAI==0) doAI=1;		// Ignores doAI=2 (calibration)
 if (key=='r') {setupAI(AIMode,botCol,&skynet); doAI=0;}		// Resets the state of the AI (may need full reset)

 // Controls for recording the corners of the playing field
 if (key=='z')
 {
  // Start/stop calibration loop. User will have to place robot at center, press 'x'
  // then place robot at bottom and press 'x' again.
  fprintf(stderr,"Offset calibration started\n");
  if (doAI==0) doAI=2; else if (doAI==2) doAI=0;
 }
 if (key=='x')
 {
  // Record Y offsets first at center then at bottom of the field
  // adj_Y[0][0] and adj_Y[1][0] will contain Y offsets for green bot
  // adj_Y[0][1] and adj_Y[1][1] will contain Y offsets for red bot
  if (got_Y==0) {got_Y=1; fprintf(stderr,"Capturing Y offset at center field\n");}
  else if (got_Y==1) {got_Y=2; fprintf(stderr,"Capturing Y offset at bottom for both robots\n");}
 }
 if (key=='c')
 {
  // Read offset calibration from file
  f=fopen("offsets.dat","r");
  if (f!=NULL)
  {
   fread(&adj_Y[0][0],4*sizeof(double),1,f);
   fclose(f);
   fprintf(stderr,"Read offset data from file\n");
   got_Y=3;
   if (doAI==2) doAI=0;
  }
  else fprintf(stderr,"No calibration data, please capture offsets first\n");
 }
 if (key=='g')		// Use cached background and H matrix
 {
  toggleProc=1;
 }
 if (key=='m')		// Manually select corners
 {
  memset(&Mcorners[0][0],0,8*sizeof(double));
  cornerIdx=0;
  toggleProc=2;		// Indicates manual corner detection active  
  mcx=340;
  mcy=340;
 }
 if (key=='d'&&toggleProc==2) if (mcx<sx-1) mcx++;
 if (key=='a'&&toggleProc==2) if (mcx>1) mcx--;
 if (key=='w'&&toggleProc==2) if (mcy>1) mcy--;
 if (key=='s'&&toggleProc==2) if (mcy<sy-1) mcy++;
 if (key=='D'&&toggleProc==2) if (mcx<sx-6) mcx+=5;
 if (key=='A'&&toggleProc==2) if (mcx>5) mcx-=5;
 if (key=='W'&&toggleProc==2) if (mcy>5) mcy-=5;
 if (key=='S'&&toggleProc==2) if (mcy<sy-6) mcy+=5;
 if (key==' '&&toggleProc==2) 
 {
  fprintf(stderr,"Recorded corner %d at %f,%f\n",cornerIdx,mcx,mcy);
  Mcorners[cornerIdx][0]=mcx;
  Mcorners[cornerIdx][1]=mcy;
  cornerIdx++;
  if (cornerIdx==4) toggleProc=0;	// Done!
 }

 // Image processing controls
 if (key=='<') {bgThresh-=50;fprintf(stderr,"BG subtract threshold now at %f\n",bgThresh);}
 if (key=='>') {bgThresh+=50;fprintf(stderr,"BG subtract threshold now at %f\n",bgThresh);}
 if (key=='{'&&colAngThresh>.5) {colAngThresh-=.01;fprintf(stderr,"Colour Angle Threshold now at %f\n",colAngThresh);}
 if (key=='}'&&colAngThresh<.99) {colAngThresh+=.01;fprintf(stderr,"Colour Angle Threshold now at %f\n",colAngThresh);}
 if (key=='['&&colThresh>=0.05) {colThresh-=.05;fprintf(stderr,"Saturation threshold now at %f\n",colThresh);}
 if (key==']'&&colThresh<=0.95) {colThresh+=.05;fprintf(stderr,"Saturation threshold now at %f\n",colThresh);}
 if (key=='f') {if (printFPS==0) printFPS=1; else printFPS=0;}

 // NXT robot manual override
 if (key=='i') {if (DIR_FWD==0) {DIR_FWD=1; DIR_L=0; DIR_R=0; DIR_BACK=0; BT_drive(LEFT_MOTOR, RIGHT_MOTOR,75);} else {DIR_FWD=0; BT_all_stop(0);}}
 if (key=='j') {if (DIR_L==0) {DIR_L=1; DIR_R=0; DIR_FWD=0; DIR_BACK=0; BT_turn(LEFT_MOTOR, 50, RIGHT_MOTOR, -50);} else {DIR_L=0; BT_all_stop(0);}}
 if (key=='l') {if (DIR_R==0) {DIR_R=1; DIR_L=0; DIR_FWD=0; DIR_BACK=0; BT_turn(LEFT_MOTOR, -50, RIGHT_MOTOR, 50);} else {DIR_R=0; BT_all_stop(0);}}
 if (key=='k') {if (DIR_BACK==0) {DIR_BACK=1; DIR_L=0; DIR_R=0; DIR_FWD=0; BT_drive(LEFT_MOTOR, RIGHT_MOTOR, -75);} else {DIR_BACK=0; BT_all_stop(0);}}
 if (key=='o') {BT_all_stop(0);doAI=0;}	// <-- Important!
}

void WindowReshape(int w, int h)
{
 ///////////////////////////////////////////////////////////////////////
 //
 // This function handles changes in the geometry of the OpenGL window
 // and ensures OpenGL renders to the correct window dimensions
 //
 ///////////////////////////////////////////////////////////////////////
 glMatrixMode(GL_PROJECTION);
 glLoadIdentity();			// Initialize with identity matrix
 gluOrtho2D(0, 800, 800, 0);
 glViewport(0,0,w,h);
 Win[0] = w;
 Win[1] = h;
}
/*********************************************************************
End of OpenGL display setup
*********************************************************************/

/*********************************************************************
Camera initialization, frame grab, and frame conversion. 
*********************************************************************/
unsigned char *yuyv_to_rgb (struct vdIn *vd, int sx, int sy)
{
  ///////////////////////////////////////////////////////////////////
  // The camera's video frame comes in a format called yuyv, this
  // means that 2 pixel RGB values are encoded as 4 bytes, two 
  // luminance samples (the two y's), and two colour samples (u and v).
  // To use the frame, we have to convert each set of 4 yuyv samples
  // into two RGB triplets. 
  //
  // The input is a video frame data structure, and the size of the
  // image (sx,sy).
  //
  // Returns a pointer to the newly allocated RGB image buffer, if
  // something goes wrong it returns NULL.
  //
  // Derived from compress_yuyv_to_jpeg() in uvccapture.c
  ///////////////////////////////////////////////////////////////////
  unsigned char *frame_buffer, *yuyv;
  int ii;
  unsigned char *ptr;

  frame_buffer = (unsigned char *)calloc (vd->height*vd->width * 3, sizeof(unsigned char));
  if (!frame_buffer)
  {
   fprintf(stderr,"yuyv_to_rgb(): Can not allocate memory for frame buffer.\n");
   return(NULL);
  }

#pragma omp parallel for schedule(dynamic,32) private(ii,ptr,yuyv)
  for (ii=0;ii<vd->height;ii++)
  {
   int x;
   ptr=frame_buffer+((ii*vd->width)*3);
   yuyv=vd->framebuffer+((ii*vd->width)*2);
   for (x = 0; x < vd->width; x+=2) 
   {
    int r, g, b;
    int y, u, v;
    y = yuyv[0] << 8;
    u = yuyv[1] - 128;
    v = yuyv[3] - 128;

    r = (y + (359 * v)) >> 8;
    g = (y - (88 * u) - (183 * v)) >> 8;
    b = (y + (454 * u)) >> 8;

    *(ptr++) = (r > 255) ? 255 : ((r < 0) ? 0 : r);
    *(ptr++) = (g > 255) ? 255 : ((g < 0) ? 0 : g);
    *(ptr++) = (b > 255) ? 255 : ((b < 0) ? 0 : b);

    y = yuyv[2] << 8;
    r = (y + (359 * v)) >> 8;
    g = (y - (88 * u) - (183 * v)) >> 8;
    b = (y + (454 * u)) >> 8;

    *(ptr++) = (r > 255) ? 255 : ((r < 0) ? 0 : r);
    *(ptr++) = (g > 255) ? 255 : ((g < 0) ? 0 : g);
    *(ptr++) = (b > 255) ? 255 : ((b < 0) ? 0 : b);
    yuyv += 4;
   }   // End for x  
  }  
  return (frame_buffer);
}
    
struct vdIn *initCam(const char *videodevice, int width, int height)
{
 /* 
    Camera initialization - sets up the camera communication, image
    format, fps, and other camera parmeters. Derived from uvccapture.c
    Derived from the uvccapture code.
 */
	int status;
	unsigned char *p = NULL;
	int hwaccel = 0;
	const char *mode = NULL;
	int format = V4L2_PIX_FMT_YUYV;
	int i;					
	int grabmethod = 1;
	int fps = 30;
	unsigned char frmrate = 0;
	char *avifilename = NULL;
	int queryformats = 0;
	int querycontrols = 0;
	int readconfigfile = 0;
	char *separateur;
	char *sizestring = NULL;
	char *fpsstring  = NULL;
	int enableRawStreamCapture = 0;
	int enableRawFrameCapture = 0;
	struct vdIn *videoIn;
	FILE *file;

	printf("roboSoccer 1.0.2015\n\n");

	if (avifilename == NULL || *avifilename == 0) {
		avifilename = "video.avi";
	}
	videoIn = (struct vdIn *) calloc(1, sizeof(struct vdIn));

	if (init_videoIn
			(videoIn, (char *) videodevice, width, height, fps, format,
			 grabmethod, avifilename) < 0)
		return(NULL);
	return(videoIn);		// Successfully opened a video device
}

unsigned char *getFrame(struct vdIn *videoIn, int sx, int sy)
{
 /*
   Grab a single frame from the camera. Derived from uvccapture.c
 */
 // NOTE - Maybe here we read until there's no more? clear the buffer?
 
	unsigned char *frame;
        double dtime;

	// Grab a frame from the video device
	if (uvcGrab(videoIn) < 0) {
		printf("Error grabbing\n");
		return(NULL);
	}
        frame=yuyv_to_rgb(videoIn, sx, sy); 
        videoIn->getPict = 0;

	// Print FPS if needed.
        frameNo++;
        time(&time2);
        dtime=difftime(time2,time1);
        if (printFPS) fprintf(stderr,"FPS= %f\n",(double)frameNo/dtime);
         
        return(frame);
}

void closeCam(struct vdIn *videoIn)
{
	close_v4l2(videoIn);
	free(videoIn);
}

/*********************************************************************
End of Webcam manipulation code
*********************************************************************/

