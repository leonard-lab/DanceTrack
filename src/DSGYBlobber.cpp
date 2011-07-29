#include "DSGYBlobber.h"
#include "MT/MT_Tracking/trackers/YA/YABlobber.h"

static int IndexMaxDiff(double* array1, int* array2, int arraylength);
static int IndexMaxDiff(int* array1, double* array2, int arraylength);
static int IndexMin(double* array, int arraylength);
static int ipow(int& a, int& n);

#define TEST_OUT(...) if(m_pTestFile){fprintf(m_pTestFile, __VA_ARGS__); fflush(m_pTestFile);}

void DSGY_PaintYABlobIntoImage(const YABlob& blob,
                               IplImage* image,
                               CvScalar color)
{
    if(blob.sequence)
    {
        cvDrawContours(image,
                       blob.sequence,
                       color,
                       color,
                       -1,
                       CV_FILLED,
                       8);
    }
}


DSGYBlobber::DSGYBlobber(unsigned int num_obj)
    : m_iBlob_area_thresh_low(GY_DEFAULT_AREA_THRESH_LOW),
      m_iBlob_area_thresh_high(GY_DEFAULT_AREA_THRESH_HIGH),
      m_SearchArea(cvRect(0,0,0,0)),
      m_bHasHistory(false),      
      m_iNObj(num_obj),
      m_pTestFile(NULL)
{
    setNumberOfObjects(num_obj);
}

void DSGYBlobber::setTestOut(FILE* fp)
{
    m_pTestFile = fp;
    m_Gaussians.setDebugFile(fp);
}

void DSGYBlobber::setNumberOfObjects(unsigned int num_obj)
{
    m_iNObj = num_obj;
    m_bHasHistory = false;
    m_RawBlobData.resize(0);
    m_CurrentBlobs.resize(m_iNObj);
    m_OldBlobs.resize(m_iNObj);
}

void DSGYBlobber::setSearchArea(CvRect new_search_area)
{
    m_SearchArea = new_search_area;
}

void DSGYBlobber::resetSearchArea()
{
    m_SearchArea = cvRect(0, 0, 0, 0);
}


void DSGYBlobber::setInitials(std::vector<double> x_c,
                              std::vector<double> y_c,
                              std::vector<double> xx,
                              std::vector<double> xy,
                              std::vector<double> yy)
{
    m_Gaussians.ClearDists();
    
    MT_Vector2 mean;
    MT_Matrix2x2 cov;
    for(unsigned int i = 0; i < x_c.size(); i++)
    {
        mean.data[0] = x_c[i];
        mean.data[1] = y_c[i];
        cov.data[0] = xx[i];
        cov.data[1] = xy[i];
        cov.data[2] = xy[i];
        cov.data[3] = yy[i];

        m_Gaussians.AddDist(mean, cov);

    }
    m_bHasHistory = true;
}



std::vector<GYBlob> DSGYBlobber::findBlobs(IplImage* thresh_image,
                                           int num_to_find,
                                           int max_iters)
{
    TEST_OUT("============= DSGYBlobber::findBlobs: =============\n");
    TEST_OUT("\tImage:  (%d x %d x %d)",
             thresh_image->width,
             thresh_image->height,
             thresh_image->nChannels);
    TEST_OUT(", %d blobs expected\n", num_to_find);        
    
    doBlobFinding(thresh_image);
    doSegmentation(num_to_find, max_iters);

    TEST_OUT("Found %d blobs:\n", (int) m_CurrentBlobs.size());
    if(m_pTestFile)
    {
        for(unsigned int i = 0; i < m_CurrentBlobs.size(); i++)
        {
            TEST_OUT("\t%d:  (x,y) = (%f, %f), orientation = %f,\n"
                     "\t\t axes = %f, %f\n"
                     "\t\t moments = %f, %f, %f\n",
                     i,
                     m_CurrentBlobs[i].m_dXCentre,
                     m_CurrentBlobs[i].m_dYCentre,
                     m_CurrentBlobs[i].m_dOrientation,
                     m_CurrentBlobs[i].m_dMinorAxis,
                     m_CurrentBlobs[i].m_dMajorAxis,
                     m_CurrentBlobs[i].m_dXXMoment,
                     m_CurrentBlobs[i].m_dXYMoment,
                     m_CurrentBlobs[i].m_dYYMoment
                );
        }
    }
    TEST_OUT(" -> Done\n");
    
    return m_CurrentBlobs;
}

void DSGYBlobber::doBlobFinding(IplImage* thresh_frame)
{
    int i, j;

    TEST_OUT("Blob Finding\n");
    /* automatically set the search area to the whole frame if it
     * doesn't make sense. */
    if(m_SearchArea.width <= 0 ||
        m_SearchArea.height <= 0 ||
        m_SearchArea.x < 0 ||
        m_SearchArea.y < 0 ||
        m_SearchArea.x + m_SearchArea.width >= thresh_frame->width ||
        m_SearchArea.y + m_SearchArea.height >= thresh_frame->height)
    {
        m_SearchArea = cvRect(0, 0, thresh_frame->width, thresh_frame->height);
    }
    TEST_OUT("\tSearch Area: (x, y, w, h) = (%d, %d, %d, %d)\n",
             m_SearchArea.x,
             m_SearchArea.y,
             m_SearchArea.width,
             m_SearchArea.height);

    /* Add every pixel to the raw blob data */
    TEST_OUT("\tAdding raw blob data\n");
    m_RawBlobData.resize(0);
    m_RawBlobData.push_back(RawBlobPtr(new GYRawBlob(300)));
    uchar p;
    for(j = 0; j < m_SearchArea.height; j++)
    {
        for(i = 0; i < m_SearchArea.width; i++)
        {
            p = ((uchar*)(thresh_frame->imageData
                          + thresh_frame->widthStep*(j + m_SearchArea.y)))[i + m_SearchArea.x];
            if(p == 255)
            {
                m_RawBlobData[0]->AddPoint(cvPoint(i + m_SearchArea.x,
                                                   j + m_SearchArea.y));
            }
        }
    }

    TEST_OUT("\tFound %d pixels in search area\n", m_RawBlobData[0]->GetNumPixels());
    
    // if there are no pixels in the search area, try resizing it
    if (m_RawBlobData[0]->GetNumPixels() == 0)
    {
        TEST_OUT("\tNo pixels found in the search area!")
            //m_bHasHistory = false;
        resetSearchArea();
        doBlobFinding(thresh_frame);
    }

    TEST_OUT(" -> Done\n");

}       // end function


/* This function takes the raw data from doBlobFinding and extracts the parameters of the individual
   blobs. It first calculates how many blobs are in each raw blob. For each raw blob that corresponds
   to a single blob, the parameters are directly measured. For each raw blob that corresponds to
   multiple blobs, an expectation maximisation algorithm is run to fit multiple ellipses to the pixel
   data. Then the individual blobs are extracted and their properties measured. */
void DSGYBlobber::doSegmentation(int num_to_find, int max_iters)
{
    TEST_OUT("EMMG\n");
    
    int i, j, k, index_max;

    int currentblob = 0;
    /* int numpixels; */
    int numinrawblob;
    std::vector<CvPoint> PixelList;
    PixelList.resize(0);
    double XXMoment, YYMoment, XYMoment, Delta, A, theta, headfraction, slope;

    numinrawblob = num_to_find;
    
    TEST_OUT("\tInitialize Distributions\n");
    if(!m_bHasHistory)
    {
        TEST_OUT("\tInitializing with uniform blobs\n");
        //m_Gaussians = MixGaussians(num_to_find, m_SearchArea);
        m_Gaussians.CoverBox(num_to_find, m_SearchArea);
    }
    else
    {
        TEST_OUT("\tAlready have %d distributions\n", m_Gaussians.GetNumDists());
        if(num_to_find > m_Gaussians.GetNumDists())
        {
            TEST_OUT("\tAdding extra distributions\n");
            m_Gaussians.CoverBox(num_to_find - m_Gaussians.GetNumDists(),
                                 m_SearchArea);
        }
    }

#if 0    
    if (!m_bHasHistory)
    {
        /* If we have no blob history, we make our initial estimate by
         * distributing the mixture evenly over the bounding box around
         * the raw blob */
        FittedBlobs = MixGaussians(numinrawblob, m_RawBlobData[i]->GetBoundingBox());
    }
    else
    {
        /* If we have a history, we want to take the blobs that are
           closest to the raw blob as our estimates */
        CvRect RawBoundingBox = m_RawBlobData[i]->GetBoundingBox();
        double xcentre = (double) RawBoundingBox.x + ((double) RawBoundingBox.width)/2.0 - 0.5;
        double ycentre = (double) RawBoundingBox.y + ((double) RawBoundingBox.height)/2.0 - 0.5;
        double maxdimension;
        if (RawBoundingBox.width > RawBoundingBox.height)
        {
            maxdimension = (double) RawBoundingBox.width;
        }
        else
        {
            maxdimension = (double) RawBoundingBox.height;
        }

        double* distances;
        distances = new double[m_iNObj];
        int closestblob;
        MT_Vector2 GuessMean;
        MT_Matrix2x2 GuessCovariance;
        double sigma1, sigma2, phi, cp, sp;

        // Calculate the distances of each blob to the centre of the bounding box
        for (k = 0 ; k < (int) m_iNObj ; k++)
        {
            distances[k] = sqrt(pow(xcentre - m_OldBlobs[k].m_dXCentre, 2) + pow(ycentre - m_OldBlobs[k].m_dYCentre, 2));

            // For blobs outside the bounding box, we add additional weight to their distances
            if((m_OldBlobs[k].m_dXCentre < RawBoundingBox.x) || (m_OldBlobs[k].m_dXCentre > (RawBoundingBox.x + RawBoundingBox.width - 1)) || (m_OldBlobs[k].m_dYCentre < RawBoundingBox.y) || (m_OldBlobs[k].m_dYCentre > (RawBoundingBox.y + RawBoundingBox.height - 1)))
            {
                distances[k] += maxdimension;
            }
        }

        // Find the closest blob, calculate a mean and covariance matrix then add these to the mixture model
        for (k = 0 ; k < numinrawblob ; k++)
        {
            closestblob = IndexMin(distances, m_iNObj);
            GuessMean.data[0] = m_OldBlobs[closestblob].m_dXCentre;
            GuessMean.data[1] = m_OldBlobs[closestblob].m_dYCentre;

            // If GuessMean is outside the bounding box, shift it inside
            if(GuessMean.data[0] < RawBoundingBox.x)
            {
                GuessMean.data[0] = (double) RawBoundingBox.x;
            }
            if(GuessMean.data[0] > RawBoundingBox.x + RawBoundingBox.width - 1)
            {
                GuessMean.data[0] = (double) (RawBoundingBox.x + RawBoundingBox.width - 1);
            }
            if(GuessMean.data[1] < RawBoundingBox.y)
            {
                GuessMean.data[1] = (double) RawBoundingBox.y;
            }
            if(GuessMean.data[1] > RawBoundingBox.y + RawBoundingBox.height - 1)
            {
                GuessMean.data[1] = (double) (RawBoundingBox.y + RawBoundingBox.height - 1);
            }

            // If the previous measured blob had a reasonable size, use it to generate the
            // covariance matrix. Otherwise, use default values
            if (m_OldBlobs[closestblob].m_dMajorAxis > 8)
            {
                sigma1 = pow(m_OldBlobs[closestblob].m_dMajorAxis/1.95, 2);
            }
            else
            {
                sigma1 = pow(8.0/1.95, 2);
            }

            if (m_OldBlobs[closestblob].m_dMinorAxis > 1)
            {
                sigma2 = pow(m_OldBlobs[closestblob].m_dMinorAxis/1.95, 2);
            }
            else
            {
                sigma2 = pow(1.0/1.95, 2);
            }

            phi = MT_DEG2RAD*m_OldBlobs[closestblob].m_dOrientation;
            cp = cos(phi);
            sp = -sin(phi);

            GuessCovariance.data[0] = sigma1*cp*cp + sigma2*sp*sp;
            GuessCovariance.data[1] = sp*cp*(sigma1 - sigma2);
            GuessCovariance.data[2] = sp*cp*(sigma1 - sigma2);
            GuessCovariance.data[3] = sigma1*sp*sp + sigma2*cp*cp;

            FittedBlobs.AddDist(GuessMean, GuessCovariance);

            // Make the distance very large so the next loop will find another blob
            distances[closestblob] = 1E20;
        }               // end for (k = 0 ; k < numinrawblob ; k++)

        delete[] distances;             // release memory
    }           // end else
#endif


   //////////////////////////////////////////////////////////////////////

    /* Create a vector to hold the distribution allocations of each pixel in the raw blob.
       For a pixel assigned to distributions d_1, d_2, d_3, ..., the allocation number will
       be d_1 + n*d_2 + n^2*d_3 + ..., where n is the total number of distributions 
       (i.e. n = numinrawblob) */
    TEST_OUT("\tInitialize Allocations\n");
    std::vector<int> PixelAllocation;
    PixelAllocation.resize(m_RawBlobData[0]->GetNumPixels());

    // Run the expectation maximisation algorithm
    TEST_OUT("\tRunning EMMG algorithm\n");
    /*m_Gaussians.m_dMaxEccentricity = 1.1;*/
    /*m_Gaussians.m_dMaxDistance = 10.0;*/
    /*m_Gaussians.m_dMaxSizePercentChange = 10.0;*/
    m_Gaussians.EMMG(m_RawBlobData[0],
                     PixelAllocation,
                     max_iters);

   //////////////////////////////////////////////////////////////////////

    // We now want a new vector of raw blobs for each extracted
    // individual blob
    TEST_OUT("\tExtracting blob information\n");
    std::vector<RawBlobPtr> ExtractedBlobs;
    ExtractedBlobs.resize(0);
    for (k = 0 ; k < numinrawblob ; k++)
    {
        RawBlobPtr rbp(new GYRawBlob(m_RawBlobData[0]->GetNumPixels()));        // Make sure the new raw blobs have space for enough pixels - faster running at expense of more initial memory
        ExtractedBlobs.push_back(rbp);
    }

   //////////////////////////////////////////////////////////////////////

    // Run through the pixels from the original raw blob and assign them to their allocated new blobs
    PixelList.resize(m_RawBlobData[0]->GetNumPixels());
    m_RawBlobData[0]->GetPixelList(PixelList);
    for (k = 0 ; k < m_RawBlobData[0]->GetNumPixels() ; k++)
    {
        int dists = PixelAllocation[k];
        int p = 1;
        for(unsigned int i = 0; i < numinrawblob; i++)
        {
            if(dists & p)
            {
                ExtractedBlobs[i]->AddPoint(PixelList[k]);
            }
            p *= 2;
        }
        
        /* int allocated;
         * int numdists = 1;
         * int dist_number = PixelAllocation[k] % numinrawblob;
         * if(dist_number < 0)
         * {
         *     continue;
         * }
         * 
         * ExtractedBlobs[dist_number]->AddPoint(PixelList[k]);
         * allocated = dist_number;
         * 
         * while (allocated != PixelAllocation[k])
         * {
         *     dist_number = ((PixelAllocation[k] - allocated)/(ipow(numinrawblob, numdists))) % numinrawblob;
         *     ExtractedBlobs[dist_number]->AddPoint(PixelList[k]);
         *     allocated += dist_number*ipow(numinrawblob, numdists);
         *     numdists++;
         * } */
    }           // end for (k = 0 ; k < m_RawBlobData[i]->GetNumPixels() ; k++)



   //////////////////////////////////////////////////////////////////////


    // Loop through the new blobs and extract their parameters
    for (k = 0 ; k < numinrawblob ; k++)
    {
        m_CurrentBlobs[k].m_dArea = ExtractedBlobs[k]->GetArea();
        m_CurrentBlobs[k].m_dXCentre = ExtractedBlobs[k]->GetXCentre();
        m_CurrentBlobs[k].m_dYCentre = ExtractedBlobs[k]->GetYCentre();
        m_CurrentBlobs[k].m_dXXMoment = ExtractedBlobs[k]->GetXXMoment();
        m_CurrentBlobs[k].m_dXYMoment = ExtractedBlobs[k]->GetXYMoment();
        m_CurrentBlobs[k].m_dYYMoment = ExtractedBlobs[k]->GetYYMoment();            

        // Calculating ellipse (semi) major and minor axes from the moments
        XXMoment = ExtractedBlobs[k]->GetXXMoment();
        YYMoment = ExtractedBlobs[k]->GetYYMoment();
        XYMoment = ExtractedBlobs[k]->GetXYMoment();

        Delta = sqrt(4*pow(XYMoment, 2) + pow(XXMoment - YYMoment, 2));
        A = pow(16*pow(M_PI, 2)*(XXMoment*YYMoment - pow(XYMoment, 2)), 0.25);

        if (A == 0)             // This will happen if all the pixels lie along a line
        {
            A = 1;
        }

        double M, m;
        M = sqrt((2*(XXMoment + YYMoment + Delta))/A);
        m = sqrt((2*(XXMoment + YYMoment - Delta))/A);        
        m_CurrentBlobs[k].m_dMajorAxis = M;
        m_CurrentBlobs[k].m_dMinorAxis = m;

        // Calculating the orientation angle from the moments. Note that after this
        // calculation, theta will be between -90 and 90
        theta = 0.5*atan2(2*XYMoment, XXMoment - YYMoment)*MT_RAD2DEG;

        // Estimate the correct orientation of the blob by assigning the side with the 
        // most pixels as the front. We determine the two 'sides' of the blob by 
        // drawing a line through the centroid, perpendicular to theta
        PixelList.resize(ExtractedBlobs[k]->GetNumPixels());
        ExtractedBlobs[k]->GetPixelList(PixelList);

        if (theta == 0)
        {
            headfraction = 0.0;
            for (j = 0; j < (int) PixelList.size() ; j++)
            {
                if (PixelList[j].x > m_CurrentBlobs[k].m_dXCentre)
                {
                    headfraction += 1.0;
                }
            }
            headfraction /= (double) PixelList.size();
        }               // end if (theta == 0)
        else
        {
            slope = tan((theta - 90.0)*MT_DEG2RAD);
            headfraction = 0.0;

            if (theta > 0)
            {
                for (j = 0 ; j < (int) PixelList.size() ; j++)
                {
                    if (PixelList[j].y > (slope*(PixelList[j].x - m_CurrentBlobs[k].m_dXCentre) + m_CurrentBlobs[k].m_dYCentre))
                    {
                        headfraction += 1.0;
                    }
                }
                headfraction /= (double) PixelList.size();
            }           // end if (theta > 0)
            else
            {
                for (j = 0 ; j < (int) PixelList.size() ; j++)
                {
                    if (PixelList[j].y < (slope*(PixelList[j].x - m_CurrentBlobs[k].m_dXCentre) + m_CurrentBlobs[k].m_dYCentre))
                    {
                        headfraction += 1.0;
                    }
                }
                headfraction /= (double) PixelList.size();
            }           // end else
        }               // end else

        // If headfraction is less than 0.5, we have the wrong orientation
        if (headfraction < 0.5)
        {
            theta = theta - 180.0;
        }
        // So now theta could be between -270 and 90

        if (theta < -180)
        {
            theta = theta + 360.0;
        }
        // Now we have theta between -180 and 180

        m_CurrentBlobs[k].m_dOrientation = -theta;

        currentblob++;
    }           // end for (k = 0 ; k < numinrawblob ; k++)

    // Record the current set of blobs so we can make estimates next time (if needed)
    m_OldBlobs = m_CurrentBlobs;
    //m_bHasHistory = true;

    TEST_OUT(" -> Done\n");

    // Cleanup
    /*if(numrawblobs)
    {
        delete[] RawBlobSizes;
        delete[] BlobSizes;
        }*/
}       // end function

/* Minor functions for finding indices for max and min values, and integer powers of integers */
static int IndexMaxDiff(double* array1, int* array2, int arraylength)
{
    int i;
    int index_max = 0;
    double currentdiff = array1[0] - (double) array2[0];

    for (i = 1 ; i < arraylength ; i++)
    {
        if ((array1[i] - (double) array2[i]) > currentdiff)
        {
            currentdiff = array1[i] - (double) array2[i];
            index_max = i;
        }
    }

    return index_max;
}       // end function


static int IndexMaxDiff(int* array1, double* array2, int arraylength)
{
    int i;
    int index_max = 0;
    double currentdiff = (double) array1[0] - array2[0];

    for (i = 1 ; i < arraylength ; i++)
    {
        if (((double) array1[i] - array2[i]) > currentdiff)
        {
            currentdiff = (double) array1[i] - array2[i];
            index_max = i;
        }
    }

    return index_max;
}       // end function


static int IndexMin(double* array, int arraylength)
{
    int i;
    int index_min = 0;
    double currentmin = array[0];

    for (i = 1 ; i < arraylength ; i++)
    {
        if (array[i] < currentmin)
        {
            currentmin = array[i];
            index_min = i;
        }
    }

    return index_min;
}       // end function


static int ipow(int& a, int& n)
{
    if (n == 0)
    {
        return 1;
    }
    else if (n > 0)
    {
        int val = 1;
        for (int i = 0; i < n ; i++)
        {
            val *= a;
        }
        return val;
    }
    else if ((a == 1) || ((a == 2) && (n == -1)))
    {
        return 1;
    }
    else
    {
        return 0;
    }
}       // end function

