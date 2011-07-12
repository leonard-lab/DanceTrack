#include "DSGYA_Segmenter.h"

#include "MT/MT_Tracking/cv/MT_HungarianMatcher.h"
#include "DSGYBlobber.h"
#include "BiCC.h"

#define DEBUG_OUT(...) if(m_pDebugFile){fprintf(m_pDebugFile, __VA_ARGS__); fflush(m_pDebugFile);}

void spit_mat(unsigned int* m, unsigned int rows, unsigned int cols, FILE* f)
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

// objs_this_comp = remove_indeces_from_vector(objs_this_comp,
// matched_objs);
template <class T>
std::vector<T> remove_indeces_from_vector(const std::vector<T>& in_vector,
                                          const std::vector<unsigned int> indeces)
{
    std::vector<T> out_vector(0);

    for(unsigned int i = 0; i < in_vector.size(); i++)
    {
        bool skip = false;
        for(unsigned int j = 0; j < indeces.size(); j++)
        {
            if(i == indeces[j])
            {
                skip = true;
                break;
            }
        }
        if(!skip)
        {
            out_vector.push_back(in_vector[i]);
        }
    }

    return out_vector;
}

bool one_to_one_match(int k,
                      unsigned int* label,
                      int nobjs, int nblobs,
                      unsigned int* ko,
                      unsigned int* kb)
{
    unsigned int matches = 0;
    for(unsigned int i = 0; i < nobjs; i++)
    {
        for(unsigned int j = 0; j < nblobs; j++)
        {
            if(label[i] == label[j + nobjs] && label[i] == k)
            {
                matches++;
                *ko = i;
                *kb = j;
            }
        }
    }

    return (matches == 1);
}

DSGYA_Blob::DSGYA_Blob()
    : m_dXCenter(0),
      m_dYCenter(0),
      m_dXXMoment(0),
      m_dXYMoment(0),
      m_dYYMoment(0),
      m_dMinorAxis(0),
      m_dMajorAxis(0),
      m_dArea(0),
      m_dOrientation(0),
      m_dRhoContrib(0)
{
}

DSGYA_Blob::DSGYA_Blob(double x,
                       double y,
                       double xx,
                       double xy,
                       double yy,
                       double m,
                       double M,
                       double a,
                       double o)
    : m_dXCenter(x),
      m_dYCenter(y),
      m_dXXMoment(xx),
      m_dXYMoment(xy),
      m_dYYMoment(yy),
      m_dMinorAxis(m),
      m_dMajorAxis(M),
      m_dArea(a),
      m_dOrientation(o),
      m_dRhoContrib(0)
{
}

DSGYA_Blob::DSGYA_Blob(const YABlob& in_blob)
    : m_dXCenter     (in_blob.COMx),
      m_dYCenter     (in_blob.COMy),
      m_dXXMoment    (in_blob.XX), 
      m_dXYMoment    (in_blob.XY), 
      m_dYYMoment    (in_blob.YY), 
      m_dMinorAxis   (in_blob.minor_axis),
      m_dMajorAxis   (in_blob.major_axis),
      m_dArea        (in_blob.area),
      m_dOrientation (in_blob.orientation),
      m_dRhoContrib(0)
{
}

DSGYA_Blob::DSGYA_Blob(const GYBlob& in_blob)
    : m_dXCenter     (in_blob.m_dXCentre),
      m_dYCenter     (in_blob.m_dYCentre),
      m_dXXMoment    (in_blob.m_dXXMoment),
      m_dXYMoment    (in_blob.m_dXYMoment),
      m_dYYMoment    (in_blob.m_dYYMoment),
      m_dMinorAxis   (in_blob.m_dMinorAxis),
      m_dMajorAxis   (in_blob.m_dMajorAxis),
      m_dArea        (in_blob.m_dArea),
      m_dOrientation (in_blob.m_dOrientation),
      m_dRhoContrib(0)
{
}


DSGYA_Segmenter::DSGYA_Segmenter()
    : m_iMinBlobArea(1),      
      m_iMinBlobPerimeter(1), 
      m_iMaxBlobArea(-1),      
      m_iMaxBlobPerimeter(-1),
      m_dOverlapFactor(1.0),
      m_iFrameWidth(0),
      m_iFrameHeight(0),
      m_pBlobFrame(NULL),
      m_pDebugFile(NULL)
{
}

DSGYA_Segmenter::~DSGYA_Segmenter()
{
    if(m_pBlobFrame){cvReleaseImage(&m_pBlobFrame);}
}

void DSGYA_Segmenter::setDebugFile(FILE* file)
{
    m_pDebugFile = file;
}

bool DSGYA_Segmenter::areAdjacent(DSGYA_Blob* obj, const YABlob& blob)
{
    if(obj->m_dRhoContrib <= 0)
    {
        obj->m_dRhoContrib = obj->m_dMajorAxis;
    }
    double dx = obj->m_dXCenter - blob.COMx;
    double dy = obj->m_dYCenter - blob.COMy;
    double rho = m_dOverlapFactor*(obj->m_dRhoContrib + blob.major_axis);
    return dx*dx + dy*dy < rho*rho;
}

bool DSGYA_Segmenter::areOverlapping(DSGYA_Blob* obj, const YABlob& blob)
{
    double dx = obj->m_dXCenter - blob.COMx;
    double dy = obj->m_dYCenter - blob.COMy;
    double rho = blob.minor_axis;
    return dx*dx + dy*dy < rho*rho;
}

void DSGYA_Segmenter::usePrevious(DSGYA_Blob* obj, unsigned int i)
{
    obj->m_dRhoContrib += 0.25*obj->m_dMajorAxis;
    // do nothing since we already copied it
    // but this function is here in case we want to e.g. use the dynamics
    // or just flag that this blob was not found
}

std::vector<YABlob> DSGYA_Segmenter::filterFirstBlobs(const std::vector<YABlob>& in_blobs)
{
    std::vector<YABlob> out_blobs(0);

    for(unsigned int i = 0; i < in_blobs.size(); i++)
    {
        if(in_blobs[i].minor_axis/in_blobs[i].major_axis > 0.5)
        {
            out_blobs.push_back(in_blobs[i]);
        }
    }

    return out_blobs;
}

std::vector<DSGYA_Blob> DSGYA_Segmenter::segmentFirstFrame(const IplImage* I,
                                                           unsigned int num_objs)
{
    DEBUG_OUT("Segmenting first frame\n");
    std::vector<DSGYA_Blob> out_blobs(num_objs);    

    m_iFrameWidth = I->width;
    m_iFrameHeight = I->height;
    m_pBlobFrame = cvCloneImage(I);

    m_YABlobber.m_bCopySequences = true;    
    std::vector<YABlob> yblobs = m_YABlobber.FindBlobs(m_pBlobFrame,
                                                       m_iMinBlobPerimeter,
                                                       m_iMinBlobArea,
                                                       m_iMaxBlobPerimeter,
                                                       m_iMaxBlobArea);

    yblobs = filterFirstBlobs(yblobs);

    DEBUG_OUT("Found %d initial blobs\n", (int) yblobs.size());

    if(yblobs.size() == 0)
    {
        fprintf(stderr, "Error:  No blobs found!\n");
        return out_blobs;
    }
    
    if(yblobs.size() == num_objs)
    {
        DEBUG_OUT("One-to-one correspondence: no EMMG step\n");
        for(unsigned int i = 0; i < num_objs; i++)
        {
            out_blobs[i] = DSGYA_Blob(yblobs[i]);
        }
        return out_blobs;
    }

    cvZero(m_pBlobFrame);
    for(unsigned int k = 0; k < yblobs.size(); k++)
    {
        DEBUG_OUT("Painting blob %d\n", k);
        DSGY_PaintYABlobIntoImage(yblobs[k],  m_pBlobFrame);
    }
    
    DSGYBlobber blobber(num_objs);
    blobber.setTestOut(m_pDebugFile);
    std::vector<GYBlob> blobs = blobber.findBlobs(m_pBlobFrame, num_objs);

    for(unsigned int k = 0; k < num_objs; k++)
    {
        out_blobs[k] = DSGYA_Blob(blobs[k]);
    }

    cvReleaseImage(&m_pBlobFrame);
    return out_blobs;

}


std::vector<DSGYA_Blob> DSGYA_Segmenter::doSegmentation(const IplImage* I,
                                                        const std::vector<DSGYA_Blob>& in_blobs)
{
    DEBUG_OUT("DSGYA_Segmenter start doSegmentation\n");
    
    std::vector<DSGYA_Blob> out_blobs(in_blobs);

    m_iFrameWidth = I->width;
    m_iFrameHeight = I->height;
    m_pBlobFrame = cvCloneImage(I);

    /* we need to copy the blob sequences so that we can later
     * determine which pixels were in the blob */
    m_YABlobber.m_bCopySequences = true;

    DEBUG_OUT("Connected Component Step\n");
    std::vector<YABlob> yblobs = m_YABlobber.FindBlobs(m_pBlobFrame,
                                                       m_iMinBlobPerimeter,
                                                       m_iMinBlobArea,
                                                       m_iMaxBlobPerimeter,
                                                       m_iMaxBlobArea);

    unsigned int rows = in_blobs.size();
    unsigned int cols = yblobs.size();
    unsigned int* adj = BiCC_CreateMat(rows, cols);
    unsigned int* label_M = BiCC_CreateMat(rows, cols);
    unsigned int* label = BiCC_CreateMat(1, rows + cols);

    DEBUG_OUT("Found %d blobs for %d objects\n", rows, cols);
    
    for(unsigned int i = 0; i < rows; i++)
    {
        for(unsigned int j = 0; j < cols; j++)
        {
            adj[i*cols + j] = areAdjacent(&out_blobs[i], yblobs[j]);
        }
    }

    DEBUG_OUT("Adjacency matrix:\n");
    if(m_pDebugFile)
    {
        spit_mat(adj, rows, cols, m_pDebugFile);
    }

    int n_cc = BiCC_Do(adj, rows, cols, label, label_M);

    DEBUG_OUT("Found %d components, label matrix:\n", n_cc);
    if(m_pDebugFile)
    {
        spit_mat(label_M, rows, cols, m_pDebugFile);
    }
    DEBUG_OUT("Label vector:\n");
    if(m_pDebugFile)
    {
        spit_mat(label, 1, rows + cols, m_pDebugFile);
    }

    std::vector<unsigned int> objs_this_comp(0);
    std::vector<unsigned int> blobs_this_comp(0);
    unsigned int nblobs;
    unsigned int nobjs;
    for(unsigned int i = 1; i <= min(n_cc, (int) rows); i++)
    {
        objs_this_comp.resize(0);
        blobs_this_comp.resize(0);
        for(unsigned int k = 0; k < rows+cols; k++)
        {
            if(label[k] == i)
            {
                if(k < rows)
                {
                    objs_this_comp.push_back(k);
                }
                else
                {
                    blobs_this_comp.push_back(k-rows);
                }
            }
        }

        nobjs = objs_this_comp.size();
        nblobs = blobs_this_comp.size();

        DEBUG_OUT("Component %d has %d objects\n", nobjs, nblobs);

        if(nobjs > 0)
        {
            if(nblobs == 0)
            {
                DEBUG_OUT("Object %d has no blobs\n", objs_this_comp[0]);
                /* no blobs */
                usePrevious(&out_blobs[objs_this_comp[0]], objs_this_comp[0]);
            }
            else if(nobjs == 1 && nblobs == 1)
            {
                /* 1-1 correspondence */
                unsigned int ox = objs_this_comp[0];
                YABlob* p_b = &yblobs[blobs_this_comp[0]];

                DEBUG_OUT("Object %d has a one-to-one match with blob %d\n",
                          objs_this_comp[0], blobs_this_comp[0]);

                out_blobs[ox].m_dXCenter = p_b->COMx;
                out_blobs[ox].m_dYCenter = p_b->COMy;
                out_blobs[ox].m_dXXMoment = p_b->XX;
                out_blobs[ox].m_dXYMoment = p_b->XY;
                out_blobs[ox].m_dYYMoment = p_b->YY;
                out_blobs[ox].m_dArea = p_b->area;
                out_blobs[ox].m_dOrientation = p_b->orientation;
                out_blobs[ox].m_dMajorAxis = p_b->major_axis;
                out_blobs[ox].m_dMinorAxis = p_b->minor_axis;                                
            }
            else if(nobjs == nblobs)
            {
                /* 1-1 correspondence, but needs sorting */
                MT_HungarianMatcher matcher;
                matcher.doInit(nobjs, nblobs);
                for(unsigned int ko = 0; ko < nobjs; ko++)
                {
                    unsigned int ox = objs_this_comp[ko];
                    for(unsigned int kb = 0; kb < nblobs; kb++)
                    {
                        YABlob* p_b = &yblobs[blobs_this_comp[kb]];
                        double dx = (p_b->COMx - out_blobs[ox].m_dXCenter);
                        double dy = (p_b->COMy - out_blobs[ox].m_dYCenter);
                        matcher.setValue(ko, kb, dx*dx + dy*dy);
                    }
                }
                std::vector<int> assignments(nobjs);
                matcher.doMatch(&assignments);
                for(unsigned int ko = 0; ko < nobjs; ko++)
                {
                    unsigned int ox = objs_this_comp[ko];
                    unsigned int bx = blobs_this_comp[assignments[ko]];
                    YABlob* p_b = &yblobs[bx];
                    out_blobs[ox].m_dXCenter = p_b->COMx;
                    out_blobs[ox].m_dYCenter = p_b->COMy;
                    out_blobs[ox].m_dXXMoment = p_b->XX;
                    out_blobs[ox].m_dXYMoment = p_b->XY;
                    out_blobs[ox].m_dYYMoment = p_b->YY;
                    out_blobs[ox].m_dArea = p_b->area;
                    out_blobs[ox].m_dOrientation = p_b->orientation;
                    out_blobs[ox].m_dMajorAxis = p_b->major_axis;
                    out_blobs[ox].m_dMinorAxis = p_b->minor_axis;                                
                }
            }
            else
            {
                DEBUG_OUT("Label %d corresponds to %d objects and %d blobs\n", i, nobjs, nblobs);

                /* try to extract as many individual blobs as possible */
                /* DEBUG_OUT("Trying to extract individual blobs\n");
                 * unsigned int* o_adj = BiCC_CreateMat(nobjs, nblobs);
                 * unsigned int* o_label_M = BiCC_CreateMat(nobjs, nblobs);
                 * unsigned int* o_label = BiCC_CreateMat(1, nobjs + nblobs);
                 * 
                 * for(unsigned int ko = 0; ko < nobjs; ko++)
                 * {
                 *     for(unsigned int kb = 0; kb < nblobs; kb++)
                 *     {
                 *         o_adj[ko*nobjs + kb] = areOverlapping(&out_blobs[objs_this_comp[ko]],
                 *                                               yblobs[blobs_this_comp[kb]]);
                 *     }
                 * }
                 * 
                 * DEBUG_OUT("Overlap matrix:\n");
                 * if(m_pDebugFile)
                 * {
                 *     spit_mat(o_adj, nobjs, nblobs, m_pDebugFile);
                 * }
                 * 
                 * int o_n_cc = BiCC_Do(o_adj, nobjs, nblobs, o_label, o_label_M);
                 * 
                 * DEBUG_OUT("Found %d components, label matrix:\n", o_n_cc);
                 * if(m_pDebugFile)
                 * {
                 *     spit_mat(o_label_M, nobjs, nblobs, m_pDebugFile);
                 * }
                 * 
                 * DEBUG_OUT("Label vector:\n");
                 * if(m_pDebugFile)
                 * {
                 *     spit_mat(o_label, 1, nobjs + nblobs, m_pDebugFile);
                 * }
                 * 
                 * unsigned int ko, kb;
                 * std::vector<unsigned int> matched_objs(0);
                 * std::vector<unsigned int> matched_blobs(0);
                 * for(unsigned int kk = 1; kk <= o_n_cc; kk++)
                 * {
                 *     if(one_to_one_match(kk, o_label, nobjs, nblobs, &ko, &kb))
                 *     {
                 *         DEBUG_OUT("Obj %d and blob %d match\n", ko, kb);
                 *         matched_objs.push_back(ko);
                 *         matched_blobs.push_back(kb);
                 *     }
                 * }
                 * 
                 * printf("Die?\n");  fflush(stdout);
                 * BiCC_ReleaseMat(o_adj);
                 * BiCC_ReleaseMat(o_label_M);
                 * BiCC_ReleaseMat(o_label);
                 * printf("Nope\n");  fflush(stdout);
                 * 
                 * for(unsigned int k = 0; k < matched_objs.size(); k++)
                 * {
                 *     ko = matched_objs[k];
                 *     kb = matched_blobs[k];
                 *     unsigned int ox = objs_this_comp[ko];
                 *     YABlob* p_b = &yblobs[blobs_this_comp[kb]];
                 *     out_blobs[ox].m_dXCenter = p_b->COMx;
                 *     out_blobs[ox].m_dYCenter = p_b->COMy;
                 *     out_blobs[ox].m_dXXMoment = p_b->XX;
                 *     out_blobs[ox].m_dXYMoment = p_b->XY;
                 *     out_blobs[ox].m_dYYMoment = p_b->YY;
                 *     out_blobs[ox].m_dArea = p_b->area;
                 *     out_blobs[ox].m_dOrientation = p_b->orientation;
                 *     out_blobs[ox].m_dMajorAxis = p_b->major_axis;
                 *     out_blobs[ox].m_dMinorAxis = p_b->minor_axis;                                
                 * }
                 * 
                 * /\* need to remove matched objects/blobs *\/
                 * objs_this_comp = remove_indeces_from_vector(objs_this_comp, matched_objs);
                 * blobs_this_comp = remove_indeces_from_vector(blobs_this_comp, matched_blobs);
                 * nobjs = objs_this_comp.size();
                 * nblobs = blobs_this_comp.size(); */

                if(nblobs != 0 && nobjs != 0)
                {
                
                    /* not a 1-1 relationship -> use EMMG */
                    cvZero(m_pBlobFrame);
                    for(unsigned int k = 0; k < nblobs; k++)
                    {
                        DEBUG_OUT("\tPainting in blob %d\n", blobs_this_comp[k]);
                        DSGY_PaintYABlobIntoImage(yblobs[blobs_this_comp[k]],
                                                  m_pBlobFrame);
                    }

                    std::vector<double> x(nobjs);
                    std::vector<double> y(nobjs);
                    std::vector<double> xx(nobjs);
                    std::vector<double> xy(nobjs);
                    std::vector<double> yy(nobjs);
                    unsigned int ox;
                    for(unsigned int k = 0; k < nobjs; k++)
                    {
                        ox = objs_this_comp[k];
                        if(out_blobs[ox].m_dXCenter == 0 && out_blobs[ox].m_dYCenter == 0)
                        {
                            printf("Warning:  Zero blob\n");
                            x[k] = yblobs[blobs_this_comp[0]].COMx + MT_frandr(-10, 10);
                            y[k] = yblobs[blobs_this_comp[0]].COMy + MT_frandr(-10, 10);
                        }
                        else
                        {
                            x[k] = out_blobs[ox].m_dXCenter;
                            y[k] = out_blobs[ox].m_dYCenter;
                        }
                        xx[k] = 1; //MT_MAX(out_blobs[ox].m_dXXMoment, 1);
                        xy[k] = 0; //out_blobs[ox].m_dXYMoment;
                        yy[k] = 1; //MT_MAX(out_blobs[ox].m_dYYMoment, 1);
                        DEBUG_OUT("\tSetting initials %f %f %f %f %f\n", x[k], y[k], xx[k], xy[k], yy[k]);
                    }

                    DSGYBlobber blobber(nobjs);
                    blobber.setTestOut(m_pDebugFile);
                    blobber.setInitials(x, y, xx, xy, yy);
                    std::vector<GYBlob> blobs = blobber.findBlobs(m_pBlobFrame, nobjs, 3);

                    for(unsigned int k = 0; k < nobjs; k++)
                    {
                        ox = objs_this_comp[k];
                        if(blobs[k].m_dXXMoment > 0 &&
                           blobs[k].m_dYYMoment > 0)
                        {
                            out_blobs[ox].m_dXCenter = blobs[k].m_dXCentre;
                            out_blobs[ox].m_dYCenter = blobs[k].m_dYCentre;
                            out_blobs[ox].m_dXXMoment = blobs[k].m_dXXMoment;
                            out_blobs[ox].m_dXYMoment = blobs[k].m_dXYMoment;
                            out_blobs[ox].m_dYYMoment = blobs[k].m_dYYMoment;
                            out_blobs[ox].m_dArea = blobs[k].m_dArea;
                            out_blobs[ox].m_dOrientation = blobs[k].m_dOrientation;
                            out_blobs[ox].m_dMajorAxis = blobs[k].m_dMajorAxis;
                            out_blobs[ox].m_dMinorAxis = blobs[k].m_dMinorAxis;
                        }
                    }
                }
            }
        }
        
    }

    BiCC_ReleaseMat(adj);
    BiCC_ReleaseMat(label);
    BiCC_ReleaseMat(label_M);

    cvReleaseImage(&m_pBlobFrame);
        
    return out_blobs;
             
}
