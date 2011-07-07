#ifndef DSGYBLOBBER_H
#define DSGYBLOBBER_H

/* DSGYBlobber.h
 *
 * D. Swain, dan.t.swain@gmail.com, 3/30/11
 *
 * Modified from GYBlobber - Removed first step of guessing
 * number of blobs.  Now set this manually.
 *
 */

// OpenCV for underlying CV algorithms and structures
#ifdef MT_HAVE_OPENCV_FRAMEWORK
#include <OpenCV/OpenCV.h>
#else
#include <cv.h>
#include <highgui.h>
#endif

#include "MT/MT_Tracking/trackers/GY/GYBlobs.h"
#include "MT/MT_Tracking/trackers/GY/MixGaussians.h"

const int GY_DEFAULT_AREA_THRESH_LOW = 10;
const int GY_DEFAULT_AREA_THRESH_HIGH = 1000;

class YABlob;
void DSGY_PaintYABlobIntoImage(const YABlob& blob,
                               IplImage* image,
                               CvScalar color = cvScalar(255));

class DSGYBlobber
{
public:
    DSGYBlobber(unsigned int num_obj);
    void setTestOut(FILE* fp);

    void doBlobFinding(IplImage* thresh_image);
    void doSegmentation(int num_to_find);

    void setInitials(std::vector<double> x_c,
                     std::vector<double> y_c,
                     std::vector<double> xx,
                     std::vector<double> xy,
                     std::vector<double> yy);
    std::vector<GYBlob> findBlobs(IplImage* thresh_image, int num_to_find);

    int m_iBlob_area_thresh_low;
    int m_iBlob_area_thresh_high;

    void setNumberOfObjects(unsigned int num_obj);
    unsigned int getNumberOfObjects() const {return m_iNObj;};

    void setSearchArea(CvRect new_search_area);
    void resetSearchArea();

protected:
    
private:
    CvRect m_SearchArea;
    
    std::vector<RawBlobPtr> m_RawBlobData;
    std::vector<GYBlob> m_CurrentBlobs;
    std::vector<GYBlob> m_OldBlobs;

    bool m_bHasHistory;

    unsigned int m_iNObj;
    MixGaussians m_Gaussians;

    FILE* m_pTestFile;
    
};

#endif /* DSGYBLOBBER_H */    
