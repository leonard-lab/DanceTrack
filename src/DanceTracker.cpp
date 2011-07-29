#include "DanceTracker.h"

#include <fstream>

#include "BiCC.h"

/* default parameter values */
const unsigned int DEFAULT_BG_THRESH = 200;
const double DEFAULT_MIN_BLOB_PERIMETER = 10;   
const double DEFAULT_MIN_BLOB_AREA = 10;        
const double DEFAULT_MAX_BLOB_PERIMETER = 1000; 
const double DEFAULT_MAX_BLOB_AREA = 1000;
const unsigned int DEFAULT_SEARCH_AREA_PADDING = 100;

const unsigned int DEFAULT_HTHRESH_LOW = 28;
const unsigned int DEFAULT_HTHRESH_HIGH = 57;
const unsigned int DEFAULT_STHRESH_LOW = 13;
const unsigned int DEFAULT_STHRESH_HIGH = 255;
const unsigned int DEFAULT_VTHRESH_LOW = 204;
const unsigned int DEFAULT_VTHRESH_HIGH = 255;

/* default color to draw blobs */
const MT_Color DEFAULT_BLOB_COLOR = MT_Red;

#define AT_GET "===============================\n"
#define AT_printf(...) printf(AT_GET "=="); printf(__VA_ARGS__); printf(AT_GET);

static void spit_mat(const std::vector<unsigned int>& m, unsigned int rows, unsigned int cols, FILE* f)
{
    for(unsigned int i = 0; i < rows; i++)
    {
        for(unsigned int j = 0; j < cols; j++)
        {
            fprintf(f, "%d ", (int) m[i*cols + j]);
        }
        fprintf(f, "\n");
    }
}

void printw(float x, float y, const char* str)
{

  GLvoid *font_style = GLUT_BITMAP_HELVETICA_12;

  //  Specify the raster position for pixel operations.
  glRasterPos3f (x, y, 0);

  //  Draw the characters one by one
  for (unsigned int i = 0; str[i] != '\0'; i++)
  glutBitmapCharacter(font_style, str[i]);

}


void Dance_Segmenter::usePrevious(DSGYA_Blob* obj, unsigned int i)
{
    DSGYA_Segmenter::usePrevious(obj, i);
    m_pTracker->notifyNoMeasurement(i);
}


static void dance_dynamics(const CvMat* x_k,
                           const CvMat* u_k,
                           const CvMat* v_k,
                           CvMat* x_kplus1)
{
    /* Assuming that the time-step is one frame rather than e.g.
     * 33 msec - I take the actual time step into account in
     * analysis. */
    double dT = 1.0;

    /* cvGetReal2D is useful to get an element of a 2D matrix
     * cvGetReal2D(x_k, i, j) gets the (i,k)^th element
     * Note that x_k is a 4x1 matrix here */
    double x = cvGetReal2D(x_k, 0, 0);
    double y = cvGetReal2D(x_k, 1, 0);
    double vx = cvGetReal2D(x_k, 2, 0); /* heading [rad] */
    double vy = cvGetReal2D(x_k, 3, 0); /* speed */

    /* position(t + 1) = position(t) + dT*velocity */
    x += dT*vx;
    y += dT*vy;

    /* works just like cvGetReal2D */
    cvSetReal2D(x_kplus1, 0, 0, x);
    cvSetReal2D(x_kplus1, 1, 0, y);
    cvSetReal2D(x_kplus1, 2, 0, vx);
    cvSetReal2D(x_kplus1, 3, 0, vy);

    /* this allows v_k to be a NULL pointer, in which case
     * this step is skipped */
    if(v_k)
    {
        /* simple additive noise: x(t+1) <- x(t+1) + noise */
        cvAdd(x_kplus1, v_k, x_kplus1);
    }
    
}

static void dance_measurement(const CvMat* x_k,
                              const CvMat* n_k,
                              CvMat* z_k)
{
    cvSetReal2D(z_k, 0, 0, cvGetReal2D(x_k, 0, 0));
    cvSetReal2D(z_k, 1, 0, cvGetReal2D(x_k, 1, 0));

    /* as above, skip this step when n_k is null */
    if(n_k)
    {
        cvAdd(z_k, n_k, z_k);
    }
}

static void constrain_state(CvMat* x_k,
                            CvMat* X_p,
                            IplImage* frame)
{
    double x = cvGetReal2D(x_k, 0, 0);
    double y = cvGetReal2D(x_k, 1, 0);
    double vx = cvGetReal2D(x_k, 2, 0);
    double vy = cvGetReal2D(x_k, 3, 0);

    double x_p = cvGetReal2D(X_p, 0, 0);
    double y_p = cvGetReal2D(X_p, 1, 0);
    double vx_p = cvGetReal2D(X_p, 2, 0);
    double vy_p = cvGetReal2D(X_p, 3, 0);

    /* MT_CLAMP(x, a, b) =
     *    x, if a <= x <= b,
     *    a, if x < a,
     *    b, if x > b
     */
    x = MT_CLAMP(x, 0, frame->width);
    y = MT_CLAMP(y, 0, frame->height);
    vx = MT_CLAMP(vx, -10, 10);
    vy = MT_CLAMP(vy, -10, 10);    

    /* MT_isnan(x) returns true if x is NaN */
    if(MT_isnan(x))
    {
        if(!MT_isnan(x_p))
        {
            x = x_p;
        }
        else
        {
            x = 0;
        }
    }
    if(MT_isnan(y))
    {
        if(!MT_isnan(y_p))
        {
            y = y_p;
        }
        else
        {
            y = 0;
        }
    }
    if(MT_isnan(vx))
    {
        if(!MT_isnan(vx_p))
        {
            vx = vx_p;
        }
        else
        {
            vx = 0.1;
        }
    }
    if(MT_isnan(vy))
    {
        if(!MT_isnan(vy_p))
        {
            vy = vy_p;
        }
        else
        {
            vy = 0.1;            
        }
    }
    
    cvSetReal2D(x_k, 0, 0, x);
    cvSetReal2D(x_k, 1, 0, y);
    cvSetReal2D(x_k, 2, 0, vx);
    cvSetReal2D(x_k, 3, 0, vy);
}


/* helper function.  Returns false if any element of M is either NaN
 * or larger in magnitude than max_val */
bool CvMatIsOk(const CvMat* M, double max_val = 1e10)
{
    double v;
    for(unsigned int i = 0; i < M->rows; i++)
    {
        for(unsigned int j = 0; j < M->cols; j++)
        {
            v = cvGetReal2D(M, i, j);
            if(MT_isnan(v) || fabs(v) > max_val)
            {
                return false;
            }
        }
    }
    return true;
}


/**********************************************************************
 * Tracker Class
 *********************************************************************/

/* Constructor - this gets called when we create a new instance of
 * this class.  It should initialize all of the member variables and
 * do whatever memory allocation is necessary. */
DanceTracker::DanceTracker(IplImage* ProtoFrame, unsigned int n_obj)
    : MT_TrackerBase(ProtoFrame),
      m_iBlobValThresh(DEFAULT_BG_THRESH),
	  m_iBlobAreaThreshLow(DEFAULT_MIN_BLOB_AREA),
	  m_iBlobAreaThreshHigh(DEFAULT_MAX_BLOB_AREA),
      m_dOverlapFactor(1.0),
	  m_iSearchAreaPadding(DEFAULT_SEARCH_AREA_PADDING),      
      m_iStartFrame(-1),
      m_iStopFrame(-1),
      m_pGYBlobber(NULL),
      m_vpUKF(n_obj, NULL),
      m_dSigmaPosition(DEFAULT_SIGMA_POSITION),
      m_dSigmaHeading(DEFAULT_SIGMA_HEADING),
      m_dSigmaSpeed(DEFAULT_SIGMA_SPEED),
      m_dSigmaPositionMeas(DEFAULT_SIGMA_POSITION_MEAS),
      m_dSigmaHeadingMeas(DEFAULT_SIGMA_HEADING_MEAS),
      m_dPrevSigmaPosition(0),
      m_dPrevSigmaHeading(0),
      m_dPrevSigmaSpeed(0),
      m_dPrevSigmaPositionMeas(0),
      m_dPrevSigmaHeadingMeas(0),
      m_bShowBlobs(true),
      m_bShowTracking(true),
      m_bShowInitialBlobs(true),
      m_bShowAssociations(true),
      m_dDt(0),
      m_iFrameCounter(0),
      m_iNObj(n_obj),
      m_iFrameHeight(0),
	  m_iFrameWidth(0),
      m_YABlobber()
{
    doInit(ProtoFrame);
}

/* Destructor - basically the opposite of the destructor, gets called
 * whenever an object is deleted or goes out of scope.  It should
 * de-allocate any memory that was allocated */
DanceTracker::~DanceTracker()
{

    if(m_pGYBlobber)
    {
        delete m_pGYBlobber;
    }

    /* cvReleaseMat deallocates matrices given to it */
    cvReleaseMat(&m_px0);
    cvReleaseMat(&m_pz);
    cvReleaseMat(&m_pQ);
    cvReleaseMat(&m_pR);

    /* MT_UKFFree frees up memory used by the UKF */
    for(unsigned int i = 0; i < m_iNObj; i++)
    {
        MT_UKFFree(&m_vpUKF[i]);
    }
}

/* this is the main initialization function and gets called by the
 * constructor.  It does memory allocation, etc. */
void DanceTracker::doInit(IplImage* ProtoFrame)
{
    /* Not using the built-in tracked objects functions - setting
     * this pointer to NULL will ensure that the appropriate code is
     * disabled  */
    m_pTrackedObjects = NULL;

    /* It's always a good idea to initialize pointers to NULL so that
       other pieces of code can use e.g. if(p) to check for allocation */
    m_pThreshFrame = NULL;
    m_pHSVFrame = NULL;
    m_pHFrame = NULL;
    m_pSFrame = NULL;
    m_pVFrame = NULL;
    m_pTempFrame1 = NULL;
    m_pTempFrame2 = NULL;

    /* Call the base class's doInit method.
     * This initializes variables to safe values and
     * calls doTrain to set the background frame,
     * frame sizes, and calls createFrames(). */
    MT_TrackerBase::doInit(ProtoFrame);
    
    /* grab the frame height */
    m_iFrameHeight = ProtoFrame->height;
	m_iFrameWidth = ProtoFrame->width;

    m_iHThresh_Low = DEFAULT_HTHRESH_LOW;
    m_iHThresh_High = DEFAULT_HTHRESH_HIGH;
    m_iSThresh_Low = DEFAULT_STHRESH_LOW;
    m_iSThresh_High = DEFAULT_STHRESH_HIGH;
    m_iVThresh_Low = DEFAULT_VTHRESH_LOW;
    m_iVThresh_High = DEFAULT_VTHRESH_HIGH;
    m_iBGThresh = DEFAULT_BG_THRESH;

    /* resize all of our vectors. note that the std::vector object
       deallocates memory on its own, so we won't have to do that later */
    m_vdBlobs_X.resize(m_iNObj);
    m_vdBlobs_Y.resize(m_iNObj);          
    m_vdBlobs_Area.resize(m_iNObj);
    m_vdBlobs_Orientation.resize(m_iNObj);
    m_vdBlobs_MajorAxis.resize(m_iNObj);  
    m_vdBlobs_MinorAxis.resize(m_iNObj);  
    m_vdBlobs_Speed.resize(m_iNObj);
    m_viMatchAssignments.resize(m_iNObj);
    m_vdTracked_X.resize(m_iNObj);
    m_vdTracked_Y.resize(m_iNObj);
    m_vdTracked_Vx.resize(m_iNObj);
    m_vdTracked_Vy.resize(m_iNObj);
    m_vbNoMeasurement.resize(m_iNObj, false);

    m_vInitBlobs.resize(0);
    m_viAssignments.resize(0);

    m_vdHistories_X.resize(m_iNObj);
    m_vdHistories_Y.resize(m_iNObj);
    /* there is an X and Y history for each object.  They need to be
       initially zero in length so that we can fill them up with real
       data as it becomes available. */
    for(unsigned int i = 0; i < m_iNObj; i++)
    {
        m_vdHistories_X[i].resize(0);
        m_vdHistories_Y[i].resize(0);        
    }

    /* grab the frame height */
    m_iFrameHeight = ProtoFrame->height;

    /* sets up the frames that are available in the "view" menu */
    m_pTrackerFrameGroup = new MT_TrackerFrameGroup();
    m_pTrackerFrameGroup->pushFrame(&m_pHSVFrame,    "HSV Frame");
    m_pTrackerFrameGroup->pushFrame(&m_pHFrame,    "H Frame");
    m_pTrackerFrameGroup->pushFrame(&m_pSFrame,    "S Frame");
    m_pTrackerFrameGroup->pushFrame(&m_pVFrame,    "V Frame");
    m_pTrackerFrameGroup->pushFrame(&m_pTempFrame1, "HSV Thresh");
    m_pTrackerFrameGroup->pushFrame(&m_pTempFrame2, "BG Thresh");    
    m_pTrackerFrameGroup->pushFrame(&m_pThreshFrame,    "Threshold Frame");


    /* Data group and Data report setup.
     *
     * This is how the tracker class interacts with the GUI.  When the
     * tracker is initialized, the GUI pulls the contents of
     * m_vDataGroups and automatically creates dialog boxes enabling
     * the user to view/edit these values.
     *
     * An MT_DataGroup is a list of parameters and can be read/write -
     * e.g. minimum blob size, or drawing color.
     * An MT_DataReport is a list of vectors of numbers and is
     * read-only.  e.g. a report of the found blobs positions.
     */
    
    /* set up the parameter groups for parameter modification, etc. */
    /* first group is for blob tracking parameters */
    MT_DataGroup* dg_blob = new MT_DataGroup("Blob Tracking Parameters");
    dg_blob->AddUInt("H min", &m_iHThresh_Low);
	dg_blob->AddUInt("H max", &m_iHThresh_High);
	dg_blob->AddUInt("S min", &m_iSThresh_Low);
	dg_blob->AddUInt("S max", &m_iSThresh_High);
	dg_blob->AddUInt("V min", &m_iVThresh_Low);
	dg_blob->AddUInt("V max", &m_iVThresh_High);

    dg_blob->AddUInt("BG Thresh", &m_iBGThresh);
    
    dg_blob->AddUInt("Min Blob Area",
                    &m_iBlobAreaThreshLow,
                    MT_DATA_READWRITE,
                    0);
    dg_blob->AddUInt("Max Blob Area",
                    &m_iBlobAreaThreshHigh,
                    MT_DATA_READWRITE,
                    0);

    dg_blob->AddDouble("Overlap Factor",
                       &m_dOverlapFactor,
                       MT_DATA_READWRITE,
                       0.1);
    
	dg_blob->AddUInt("Search Area Padding",
		             &m_iSearchAreaPadding,
					 MT_DATA_READWRITE,
					 0);
    dg_blob->AddDouble("Position Disturbance Sigma",
                       &m_dSigmaPosition,
                       MT_DATA_READWRITE,
                       0);
    dg_blob->AddDouble("Speed Disturbance Sigma",
                       &m_dSigmaSpeed,
                       MT_DATA_READWRITE,
                       0);
    dg_blob->AddDouble("Position Measurement Sigma",
                       &m_dSigmaPositionMeas,
                       MT_DATA_READWRITE,
                       0);

    MT_DataGroup* dg_draw = new MT_DataGroup("Drawing Options");
    dg_draw->AddBool("Show blob ellipses", &m_bShowBlobs);
    dg_draw->AddBool("Show tracking arrows", &m_bShowTracking);
    dg_draw->AddBool("Show first pass blob ellipses", &m_bShowInitialBlobs);
    dg_draw->AddBool("Show associations", &m_bShowAssociations);    
    
    /* now stuff the parameter groups into m_vDataGroups, which
     * MT_TrackerBase will automagically report to the GUI */
    m_vDataGroups.resize(0);
    m_vDataGroups.push_back(dg_blob);
    m_vDataGroups.push_back(dg_draw);
    
    MT_DataReport* dr_tracked = new MT_DataReport("Tracked data");
    dr_tracked->AddDouble("X", &m_vdTracked_X);
    dr_tracked->AddDouble("Y", &m_vdTracked_Y);
    dr_tracked->AddDouble("Vx", &m_vdTracked_Vx);
    dr_tracked->AddDouble("Vy", &m_vdTracked_Vy);
    m_vDataReports.resize(0);
    m_vDataReports.push_back(dr_tracked);

}

/* This gets called by MT_TrackerBase::doInit.  I use it here more to
 * allocate other bits of memory - a lot of this could actually go
 * into doInit, but there's no real harm */
void DanceTracker::createFrames()
{
    m_SearchArea = cvRect(0,0,0,0);
        
    /* this makes sure that the BG_frame is created */
    MT_TrackerBase::createFrames();

    /* Create the Thresholder and Blobber objects */
    m_pGYBlobber = new GYBlobber(m_iNObj);
    /* Initialize the Hungarian Matcher */
    m_HungarianMatcher.doInit(m_iNObj);

    /* Initialize some of the matrices used for tracking */
    unsigned int nstate = 4;
    unsigned int nmeas = 2;
    m_pQ = cvCreateMat(nstate, nstate, CV_64FC1);
    m_pR = cvCreateMat(nmeas, nmeas, CV_64FC1);
    cvSetIdentity(m_pQ);
    cvSetIdentity(m_pR);
    m_px0 = cvCreateMat(nstate, 1, CV_64FC1);
    m_pz = cvCreateMat(nmeas, 1, CV_64FC1);

    /* Create the UKF objects */
    m_vpUKF.resize(m_iNObj);
    for(unsigned int i = 0; i < m_iNObj; i++)
    {
        m_vpUKF[i] = MT_UKFInit(nstate, nmeas, 0.1); /* 0.1 is a
                                                        paramater */
    }
    
    if(m_pHSVFrame)
    {
        cvReleaseImage(&m_pHSVFrame);
        cvReleaseImage(&m_pHFrame);
        cvReleaseImage(&m_pSFrame);
        cvReleaseImage(&m_pVFrame);
        cvReleaseImage(&m_pTempFrame1);
        cvReleaseImage(&m_pTempFrame2);        
    }    

    printf("Createframes with %d %d\n", m_iFrameWidth, m_iFrameHeight);
    m_pHSVFrame = cvCreateImage(cvSize(m_iFrameWidth, m_iFrameHeight), IPL_DEPTH_8U, 3);
    m_pHFrame = cvCreateImage(cvSize(m_iFrameWidth, m_iFrameHeight), IPL_DEPTH_8U, 1);
    m_pSFrame = cvCreateImage(cvSize(m_iFrameWidth, m_iFrameHeight), IPL_DEPTH_8U, 1);
    m_pVFrame = cvCreateImage(cvSize(m_iFrameWidth, m_iFrameHeight), IPL_DEPTH_8U, 1);
    m_pTempFrame1 = cvCreateImage(cvSize(m_iFrameWidth, m_iFrameHeight), IPL_DEPTH_8U, 1);
    m_pTempFrame2 = cvCreateImage(cvSize(m_iFrameWidth, m_iFrameHeight), IPL_DEPTH_8U, 1);
    m_pThreshFrame = cvCreateImage(cvSize(m_iFrameWidth, m_iFrameHeight), IPL_DEPTH_8U, 1);    

} /* endof createFrames */

/* this gets called during the destructor */
void DanceTracker::releaseFrames()
{
    if(m_pHSVFrame)
    {
        cvReleaseImage(&m_pHSVFrame);
        cvReleaseImage(&m_pHFrame);
        cvReleaseImage(&m_pSFrame);
        cvReleaseImage(&m_pVFrame);
        cvReleaseImage(&m_pTempFrame1);
        cvReleaseImage(&m_pTempFrame2);        
    }    
    
    /* Nothing really to do here.  BG_Frame is managed by the base
     * class and the GS, Diff, and Thresh frames are managed by
     * m_pGSThresholder */
}

/* Initialize a data file for output.  Gets called from the GUI or
 * command line */
void DanceTracker::initDataFile()
{
    /* An XDF can store parameters in XML format and manages data
       output to other files that are keyed in the master XML file.

       These can then be read by e.g. MATLAB */
    char v[10];
    sprintf(v, "%d", m_iStartFrame);
    m_XDF.writeParameterToXML("Starting Frame", v);
    sprintf(v, "%d", m_iStopFrame);
    m_XDF.writeParameterToXML("Stopping Frame", v);

/*    m_XDF.addDataStream("Blob X", "blob_x.dat");
    m_XDF.addDataStream("Blob Y", "blob_y.dat");
    m_XDF.addDataStream("Blob Area", "blob_area.dat");
    m_XDF.addDataStream("Blob Orientation", "blob_orientation.dat");
    m_XDF.addDataStream("Blob Major Axis", "blob_major.dat");
    m_XDF.addDataStream("Blob Minor Axis", "blob_minor.dat");*/

    m_XDF.addDataStream("Frame Number", "frame_num.dat");
    m_XDF.addDataStream("Tracked X", "tracked_x.dat");
    m_XDF.addDataStream("Tracked Y", "tracked_y.dat");    
    m_XDF.addDataStream("Tracked Heading", "tracked_heading.dat");    
    m_XDF.addDataStream("Tracked Speed", "tracked_speed.dat");
    
    MT_TrackerBase::initDataFile();
}

/* a helper function to write data to a file */
void DanceTracker::writeData()
{

    /* the XDF object handles writing data to the right files - all we
       have to do is pass the data as vectors */
/*    m_XDF.writeData("Blob X"           , m_vdBlobs_X); 
    m_XDF.writeData("Blob Y"           , m_vdBlobs_Y); 
    m_XDF.writeData("Blob Area"        , m_vdBlobs_Area); 
    m_XDF.writeData("Blob Orientation" , m_vdBlobs_Orientation); 
    m_XDF.writeData("Blob Major Axis"  , m_vdBlobs_MajorAxis); 
    m_XDF.writeData("Blob Minor Axis"  , m_vdBlobs_MinorAxis); */
    
    m_vdTimeNow.resize(1);
    m_vdTimeNow[0] = m_iFrameCounter;
    m_XDF.writeData("Frame Number"     , m_vdTimeNow);
    m_XDF.writeData("Tracked X"        , m_vdTracked_X); 
    m_XDF.writeData("Tracked Y"        , m_vdTracked_Y); 
    m_XDF.writeData("Tracked Heading"  , m_vdTracked_Vx); 
    m_XDF.writeData("Tracked Speed"    , m_vdTracked_Vy); 
}

void DanceTracker::HSVSplit(IplImage* frame)
{
    cvCvtColor(frame, m_pHSVFrame, CV_RGB2HSV);
    cvSplit(m_pHSVFrame, 
            m_pHFrame, 
            m_pSFrame, 
            m_pVFrame,
            NULL);
}

/* Main tracking function - gets called by MT_TrackerFrameBase every
 * time step when the application is not paused. */
void DanceTracker::doTracking(IplImage* frame)
{
    /* time-keeping, if necessary
     * NOTE this is not necessary for keeping track of frame rate */
    static double t_prev = MT_getTimeSec();
    double t_now = MT_getTimeSec();
    m_dDt = t_now - t_prev;
    t_prev = t_now;

    /* keeping track of the frame number, if necessary */
    m_iFrameCounter++;

    /* This checks every time step to see if the UKF parameters have
       changed and modifies the UKF structures accordingly.  This will
       also get called the first time through b/c the "Prev" values get
       set to zero initially.   There may be a more efficient way to do
       this, but because the values need to be embedded into the CvMat
       objects I'm not sure how else to do it. */ 
    if(
        m_dSigmaPosition != m_dPrevSigmaPosition ||
        m_dSigmaSpeed != m_dPrevSigmaSpeed ||
        m_dSigmaPositionMeas != m_dPrevSigmaPositionMeas
        )
    {
        /* these are the diagonal entries of the "Q" matrix, which
           represents the variances of the process noise.  They're
           modeled here as being independent and uncorrellated. */
        cvSetReal2D(m_pQ, 0, 0, m_dSigmaPosition*m_dSigmaPosition);
        cvSetReal2D(m_pQ, 1, 1, m_dSigmaPosition*m_dSigmaPosition);
        cvSetReal2D(m_pQ, 2, 2, m_dSigmaHeading*m_dSigmaSpeed);
        cvSetReal2D(m_pQ, 3, 3, m_dSigmaSpeed*m_dSigmaSpeed);        

        /* these are the diagonal entries of the "R matrix, also
           assumed to be uncorrellated. */
        cvSetReal2D(m_pR, 0, 0, m_dSigmaPositionMeas*m_dSigmaPositionMeas);
        cvSetReal2D(m_pR, 1, 1, m_dSigmaPositionMeas*m_dSigmaPositionMeas);

        /* this step actually copies the Q and R matrices to the UKF
           and makes sure that it's internals are properly initialized -
           it's set up to handle the fact that the sizes of these
           matrices could have changed. */
        for(unsigned int i = 0; i < m_iNObj; i++)
        {
            MT_UKFCopyQR(m_vpUKF[i], m_pQ, m_pR);
        }
    }


    HSVSplit(frame);

    cvThreshold(m_pHFrame, m_pThreshFrame, m_iHThresh_Low, 255, CV_THRESH_BINARY);
    cvThreshold(m_pHFrame, m_pTempFrame1, m_iHThresh_High, 255, CV_THRESH_BINARY_INV);
    cvAnd(m_pThreshFrame, m_pTempFrame1, m_pThreshFrame);

    cvThreshold(m_pSFrame, m_pTempFrame1, m_iSThresh_Low, 255, CV_THRESH_BINARY);
    cvThreshold(m_pSFrame, m_pTempFrame2, m_iSThresh_High, 255, CV_THRESH_BINARY_INV);
    cvAnd(m_pTempFrame1, m_pTempFrame2, m_pTempFrame1);
    cvAnd(m_pThreshFrame, m_pTempFrame1, m_pThreshFrame);

    cvThreshold(m_pVFrame, m_pTempFrame1, m_iVThresh_Low, 255, CV_THRESH_BINARY);
    cvThreshold(m_pVFrame, m_pTempFrame2, m_iVThresh_High, 255, CV_THRESH_BINARY_INV);
    cvAnd(m_pTempFrame1, m_pTempFrame2, m_pTempFrame1);
    cvAnd(m_pThreshFrame, m_pTempFrame1, m_pTempFrame1);

    cvSub(BG_frame, m_pVFrame, m_pTempFrame2);
    cvThreshold(m_pTempFrame2, m_pTempFrame2, m_iBGThresh, 255, CV_THRESH_BINARY);

    cvOr(m_pTempFrame1, m_pTempFrame2, m_pThreshFrame);
    cvSmooth(m_pThreshFrame, m_pThreshFrame, CV_MEDIAN, 3);
    
    if(ROI_frame)
    {
        cvAnd(m_pThreshFrame, ROI_frame, m_pThreshFrame);
    }
                

/*    std::vector<YABlob> yblobs = m_YABlobber.FindBlobs(m_pThreshFrame,
                                                       5,
                                                       m_iBlobAreaThreshLow,
                                                       NO_MAX,
                                                       m_iBlobAreaThreshHigh);


    int ny = yblobs.size();
    m_vdBlobs_X.resize(ny);
    m_vdBlobs_Y.resize(ny);
    m_vdBlobs_Orientation.resize(ny);
    for(unsigned int i = 0; i < yblobs.size(); i++)
    {
        m_vdBlobs_X[i] = yblobs[i].COMx;
        m_vdBlobs_Y[i] = yblobs[i].COMy;
        m_vdBlobs_Orientation[i] = 0;
        }*/

    m_vbNoMeasurement.assign(m_iNObj, false);
    
    Dance_Segmenter segmenter(this);
    segmenter.setDebugFile(stdout);
    segmenter.m_iMinBlobPerimeter = 1;
    segmenter.m_iMinBlobArea = m_iBlobAreaThreshLow;
    segmenter.m_iMaxBlobArea = m_iBlobAreaThreshHigh;
    segmenter.m_dOverlapFactor = m_dOverlapFactor;

    if(m_iFrameCounter <= 1)
    {
        std::ifstream in_file;
        in_file.open("initials.dat");
        double x, y;

        m_vBlobs.resize(0);
        m_vBlobs = readDSGYABlobsFromFile("initials.dat");

        m_vInitBlobs.resize(0);
        m_viAssignments.resize(0);

/*        m_vBlobs = segmenter.segmentFirstFrame(m_pThreshFrame,
          m_iNObj); */
    }
    else
    {
        writeDSGYABlobsToFile(m_vBlobs, "blobs-in.dat");
        writeDSGYABlobsToFile(m_vPredictedBlobs, "predicted-in.dat");
        
        bool use_prediction = true;
        m_vBlobs = segmenter.doSegmentation(m_pThreshFrame,
                                            use_prediction ? m_vPredictedBlobs : m_vBlobs);

        m_viAssignments = segmenter.getAssignmentVector(&m_iAssignmentRows, &m_iAssignmentCols);
        m_vInitBlobs = segmenter.getInitialBlobs();
    }

    /* prediction is done below - this makes sure the predicted blobs
       are OK no matter what */
    m_vPredictedBlobs = m_vBlobs;
    
    unsigned int sc = 0;
    bool same_frame = false;
    for(unsigned int i = 0; i < m_vBlobs.size(); i++)
    {
        if(m_vdBlobs_X[i] == m_vBlobs[i].m_dXCenter)
        {
            sc++;
        }
        m_vdBlobs_X[i] = m_vBlobs[i].m_dXCenter;
        m_vdBlobs_Y[i] = m_vBlobs[i].m_dYCenter;
        m_vdBlobs_Orientation[i] = m_vBlobs[i].m_dOrientation;
    }

    same_frame = (sc >= m_iNObj - 2);
    if(same_frame)
    {
        return;
    }

    /* Tracking / UKF / State Estimation
     *
     * Now that we've got the mapping of which measurement goes with
     * which object, we need to feed the measurements into the UKF in
     * order to obtain a state estimate.
     *
     * This is a loop over each object we're tracking. 
     */

    for(unsigned int i = 0; i< m_iNObj; i++)
    {
    
        /* we could throw out a measurement and use the blob
           state as an estimate for various reasons.  On the first
           frame we want to set the initial state, so we flag the
           measurement as invalid */
        bool invalid_meas =  m_vbNoMeasurement[i];
        bool need_state = m_iFrameCounter == 1;
        
        /* if any state is NaN, reset the UKF
         * This shouldn't happen anymore, but it's a decent safety
         * check.  It could probably be omitted if we want to
         * optimize for speed... */
        if(m_iFrameCounter > 1 &&
           (!CvMatIsOk(m_vpUKF[i]->x) ||
            !CvMatIsOk(m_vpUKF[i]->P)))
        {
            MT_UKFFree(&(m_vpUKF[i]));
            m_vpUKF[i] = MT_UKFInit(4, 2, 0.1);
            MT_UKFCopyQR(m_vpUKF[i], m_pQ, m_pR);
            need_state = true;
        }

        if(need_state)
        {
            cvSetReal2D(m_px0, 0, 0, m_vdBlobs_X[i]);
            cvSetReal2D(m_px0, 1, 0, m_vdBlobs_Y[i]);
            cvSetReal2D(m_px0, 2, 0, 0);
            cvSetReal2D(m_px0, 3, 0, 0);
            MT_UKFSetState(m_vpUKF[i], m_px0);
        }
    
        /* if we're going to accept this measurement */
        if(!invalid_meas)
        {
            /* UKF prediction step, note we use function pointers to
               the fish_dynamics and fish_measurement functions defined
               above.  The final parameter would be for the control input
               vector, which we don't use here so we pass a NULL pointer */
            MT_UKFPredict(m_vpUKF[i],
                          &dance_dynamics,
                          &dance_measurement,
                          NULL);
    
            /* finally, set the measurement vector z */
            cvSetReal2D(m_pz, 0, 0, m_vdBlobs_X[i]);
            cvSetReal2D(m_pz, 1, 0, m_vdBlobs_Y[i]);

            MT_UKFSetMeasurement(m_vpUKF[i], m_pz);
    
            /* then do the UKF correction step, which accounts for the
               measurement */
            MT_UKFCorrect(m_vpUKF[i]);
    
    
        }
        else  
        {
            /* use the predicted state */
            CvMat* xp = m_vpUKF[i]->x1;
            MT_UKFSetState(m_vpUKF[i], xp);
        }
        
        /* then constrain the state if necessary - see function
         * definition above */
        constrain_state(m_vpUKF[i]->x, m_vpUKF[i]->x1, frame);            
    
        /* grab the state estimate and store it in variables that will
           make it convenient to save it to a file. */
        CvMat* x = m_vpUKF[i]->x;
    
        m_vdTracked_X[i] = cvGetReal2D(x, 0, 0);
        m_vdTracked_Y[i] = cvGetReal2D(x, 1, 0);
        m_vdTracked_Vx[i] = cvGetReal2D(x, 2, 0);
        m_vdTracked_Vy[i] = cvGetReal2D(x, 3, 0);

        /* take the tracked positions as the blob centers */
        m_vBlobs[i].m_dXCenter = m_vdTracked_X[i];
        m_vBlobs[i].m_dYCenter = m_vdTracked_Y[i];

        /* predict blob locations */
        CvMat* xp = m_vpUKF[i]->x1;
        m_vPredictedBlobs[i].m_dXCenter = cvGetReal2D(xp, 0, 0);
        m_vPredictedBlobs[i].m_dYCenter = cvGetReal2D(xp, 1, 0);        
    
        /* If we wanted the predicted state, this would be how to get
           it */
        /* CvMat* xp = m_vpUKF[i]->x1; */
    }

    writeDSGYABlobsToFile(m_vBlobs, "blobs-out.dat");
    writeDSGYABlobsToFile(m_vPredictedBlobs, "predicted-out.dat");
    
    /* write data to file */
    writeData();

}

/* Drawing function - gets called by the GUI
 * All of the drawing is done with OpenGL */
void DanceTracker::doGLDrawing(int flags)
{
    /* MT_R3 is a 3-vector class, used here for convenience */
    MT_R3 blobcenter;

    /* m_bShowBlobs is a boolean in the "Drawing Options" data group.
       If the user selects "Drawing Options" from the "Tracking" menu,
       a dialog will pop up with a check box labeled "Show blob arrows",
       and its state will be linked (via a pointer) to the value of this
       variable */
    if(m_bShowBlobs)
    {
        for (unsigned int i = 0; i < MT_MIN(m_vdBlobs_X.size(), m_vBlobs.size()); i++)
        {

            /* note that we have to subtract y from the frame_height
               to account for the window coordinates originating from the
               bottom left but the image coordinates originating from
               the top left */
            blobcenter.setx(m_vdBlobs_X[i]);
            blobcenter.sety(m_iFrameHeight - m_vdBlobs_Y[i]);
            blobcenter.setz( 0 );

            double M, m;
            M = MT_MAX(m_vBlobs[i].m_dMajorAxis, 0.01);
            m = MT_MAX(m_vBlobs[i].m_dMinorAxis, 0.01);
            
            MT_DrawEllipse(blobcenter,
                           M,
                           m,
                           m_vBlobs[i].m_dOrientation,
                           MT_Primaries[i % MT_NPrimaries]);

            /* draws an arrow using OpenGL */
            /* MT_DrawArrow(blobcenter,  // center of the base of the arrow
             *              20.0,        // arrow length (pixels)
             *              MT_DEG2RAD*m_vdBlobs_Orientation[i], // arrow angle
             *              MT_Primaries[i % MT_NPrimaries], // color
             *              1.0 // fixes the arrow width
             *     );     */
        }
    }

    /* essentially the same as above, but with the tracked positions
       instead of the blob positions */
    char s[50];
    if(m_bShowTracking)
    {
        for (unsigned int i = 0; i < m_vdTracked_X.size(); i++)
        {

            blobcenter.setx(m_vdTracked_X[i]);
            blobcenter.sety(m_iFrameHeight - m_vdTracked_Y[i]);
            blobcenter.setz( 0 );

            sprintf(s, "%d", i);
            printw(blobcenter.getx(), blobcenter.gety(), s);

            double th = atan2(-m_vdTracked_Vy[i], m_vdTracked_Vx[i]);

            MT_DrawArrow(blobcenter,
                         10.0, 
                         th,
                         MT_Primaries[i % MT_NPrimaries],
                         0.5 
                );    
        }
    }

    double x1, y1, x2, y2;

    if(m_bShowInitialBlobs)
    {
        glLineStipple(1, 0x00FF);
        glEnable(GL_LINE_STIPPLE);
        for(unsigned int i = 0; i < m_vInitBlobs.size(); i++)
        {
            blobcenter.setx(m_vInitBlobs[i].COMx);
            blobcenter.sety(m_iFrameHeight - m_vInitBlobs[i].COMy);
            blobcenter.setz( 0 );

            double M, m;
            M = MT_MAX(m_vInitBlobs[i].major_axis, 0.01);
            m = MT_MAX(m_vInitBlobs[i].minor_axis, 0.01);
            
            MT_DrawEllipse(blobcenter,
                           M,
                           m,
                           m_vInitBlobs[i].orientation,
                           MT_Green);
        
        }
        glDisable(GL_LINE_STIPPLE);
    }

    if(m_bShowAssociations)
    {
        glLineStipple(1, 0x00FF);
        glEnable(GL_LINE_STIPPLE);
        
        if(m_vInitBlobs.size() > 0 && m_viAssignments.size() > 0)
        {
            unsigned int max_label = BiCC::getNumberOfLabels(m_viAssignments);
            for(unsigned int c = 0; c <  max_label; c++)
            {
                for(unsigned int i = 0; i < m_iAssignmentRows; i++)
                {
                    x1 = m_vdBlobs_X[i];
                    y1 = m_iFrameHeight - m_vdBlobs_Y[i];
                    for(unsigned int j = 0; j < m_iAssignmentCols; j++)
                    {
                        if(m_viAssignments[i] == c && m_viAssignments[m_iAssignmentRows + j] == c)
                        {
                            x2 = m_vInitBlobs[j].COMx;
                            y2 = m_iFrameHeight - m_vInitBlobs[j].COMy;
                            MT_DrawLineFromTo(x1, y1, x2, y2, MT_Red);
                        }
                    }
                }
            }
        }
        glDisable(GL_LINE_STIPPLE);        
    }

//	MT_DrawRectangle(m_SearchArea.x, m_iFrameHeight - m_SearchArea.y - m_SearchArea.height, m_SearchArea.width, m_SearchArea.height);

}

