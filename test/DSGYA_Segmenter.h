#ifndef DSGYA_SEGMENTER_H
#define DSGYA_SEGMENTER_H

//#include "MT/MT_Tracking/trackers/YA/YABlobber.h"

//#include "DSGY_Blobber.h"

class YABlobber;
class IplImage;

class DSGYA_Blob
{
    DSGYA_Blob();
    DSGYA_Blob(double x,
               double y,
               double xx,
               double xy,
               double yy,
               double m,
               double M,
               double a,
               double o);

    double m_dXCenter;
    double m_dYCenter;
    double m_dXXMoment;
    double m_dXYMoment;
    double m_dYYMoment;
    double m_dMinorAxis;
    double m_dMajorAxis;
    double m_dArea;
    double m_dOrientation;
}

class DSGYA_Segmenter
{
public:
    DSGYA_Segmenter();

    std::vector<DSGYA_Blob> doSegmentation(IplImage* I,
                                           std::vector<DSGYA_Blob> in_blobs);

    unsigned int m_iMinBlobArea;
    unsigned int m_iMinBlobPerimeter;
    unsigned int m_iMaxBlobArea;
    unsigned int m_iMaxBlobPerimeter;
    
private:
    unsigned int m_iFrameWidth;
    unsigned int m_iFrameHeight;
    IplImage* m_pBlobFrame;

    YABlobber m_YABlobber;
};

#endif // DSGYA_SEGMENTER_H
