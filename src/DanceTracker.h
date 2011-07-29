#ifndef DANCETRACKER_H
#define DANCETRACKER_H

#include "MT_Core.h"
#include "MT_Tracking.h"

#include "DSGYA_Segmenter.h"

const double DEFAULT_SIGMA_POSITION = 4.0; /* pixels */
const double DEFAULT_SIGMA_HEADING = 0.26; /* rad ~= 15 deg */
const double DEFAULT_SIGMA_SPEED = 1.0; /* pixels/frame */
const double DEFAULT_SIGMA_POSITION_MEAS = 1.0; /* pixels */
const double DEFAULT_SIGMA_HEADING_MEAS = 0.087; /* rad ~= 5 deg */

class DanceTracker;

class Dance_Segmenter : public DSGYA_Segmenter
{
public:
    Dance_Segmenter(DanceTracker* tracker)
        : m_pTracker(tracker), DSGYA_Segmenter() {};
    ~Dance_Segmenter(){};

    void usePrevious(DSGYA_Blob* obj, unsigned int i);
    
private:
    DanceTracker* m_pTracker;
};

typedef std::vector<double> t_p_history;

class DanceTracker : public MT_TrackerBase
{
private:
    /* frames */
    IplImage* m_pHSVFrame;
    IplImage* m_pHFrame;
    IplImage* m_pSFrame;
    IplImage* m_pVFrame;
    IplImage* m_pTempFrame1;
    IplImage* m_pTempFrame2;    
    
    IplImage* m_pThreshFrame;  /* Thresholded frame */

    /* blobber parameters */

    unsigned int m_iHThresh_Low;
    unsigned int m_iHThresh_High;
    unsigned int m_iSThresh_Low;
	unsigned int m_iSThresh_High;
    unsigned int m_iVThresh_Low;	
	unsigned int m_iVThresh_High;

    unsigned int m_iBGThresh;
    
    unsigned int m_iBlobValThresh;
	unsigned int m_iBlobAreaThreshLow;
	unsigned int m_iBlobAreaThreshHigh;

    double m_dOverlapFactor;

    /* only used to add to XDF */
    int m_iStartFrame;
    int m_iStopFrame;

    CvRect m_SearchArea;
	unsigned int m_iSearchAreaPadding;


    GYBlobber* m_pGYBlobber;
    YABlobber m_YABlobber;
    std::vector<MT_UKF_struct*> m_vpUKF;
    CvMat* m_pQ;
    CvMat* m_pR;
    CvMat* m_px0;
    CvMat* m_pz;
    double m_dSigmaPosition;
    double m_dSigmaHeading;
    double m_dSigmaSpeed;
    double m_dSigmaPositionMeas;
    double m_dSigmaHeadingMeas;
    
    double m_dPrevSigmaPosition;
    double m_dPrevSigmaHeading;
    double m_dPrevSigmaSpeed;
    double m_dPrevSigmaPositionMeas;
    double m_dPrevSigmaHeadingMeas;

    bool m_bShowBlobs;
    bool m_bShowTracking;
    bool m_bShowInitialBlobs;
    bool m_bShowAssociations;

    /* operational variables */
    double m_dDt;
    int m_iFrameCounter;
    int m_iNObj;

    std::vector<DSGYA_Blob> m_vBlobs;
    std::vector<DSGYA_Blob> m_vPredictedBlobs;    

    std::vector<double> m_vdBlobs_X;
    std::vector<double> m_vdBlobs_Y; 
    std::vector<double> m_vdBlobs_Area; 
    std::vector<double> m_vdBlobs_Orientation;
    std::vector<double> m_vdBlobs_MajorAxis;
    std::vector<double> m_vdBlobs_MinorAxis;
    std::vector<double> m_vdBlobs_Speed;

    std::vector<bool> m_vbNoMeasurement;

    std::vector<double> m_vdTimeNow;
    std::vector<double> m_vdTracked_X;
    std::vector<double> m_vdTracked_Y;
    std::vector<double> m_vdTracked_Vx;
    std::vector<double> m_vdTracked_Vy;

    std::vector<t_p_history> m_vdHistories_X;
    std::vector<t_p_history> m_vdHistories_Y;

    unsigned int m_iFrameHeight;
	unsigned int m_iFrameWidth;

    std::vector<unsigned int> m_viAssignments;
    std::vector<YABlob> m_vInitBlobs;
    unsigned int m_iAssignmentRows;
    unsigned int m_iAssignmentCols;    
    
public:
    /* constructor */
    DanceTracker(IplImage* ProtoFrame, unsigned int n_obj);
    /* destructor - note we need the virtual destructor */
    virtual ~DanceTracker();
    
    /* Initialization */
    void doInit(IplImage* ProtoFrame);

    /* Memory allocation / deallocation */
    void createFrames();
    void releaseFrames();

    void HSVSplit(IplImage* frame);

    void setDiffThresh(int thresh){m_iBlobValThresh = thresh;};
    void setStartStopFrames(int start_frame, int stop_frame)
    {m_iStartFrame = start_frame; m_iStopFrame = stop_frame;};

    void doTrain(IplImage* frame){
        m_iFrameWidth = frame->width;
        m_iFrameHeight = frame->height;        
        MT_TrackerBase::doTrain(frame);
    };

    void initDataFile();
    void writeData();

    /* Main tracking functions */
    void doTracking(IplImage* frame);

    void doGLDrawing(int flags);

    void notifyNoMeasurement(unsigned int i)
    {
        m_vbNoMeasurement[i] = true;
    }
    

};

#endif /* DANCETRACKER_H */
