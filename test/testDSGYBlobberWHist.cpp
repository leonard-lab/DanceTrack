#include <iostream>
#include <fstream>

#include "DSGYBlobber.h"
#include "MT/MT_Tracking/trackers/YA/YABlobber.h"
#include "MT/MT_Tracking/cv/MT_HungarianMatcher.h"

const int STATUS_OK = 0;
const int STATUS_NOT_OK = 1;

int main(int argc, char** argv)
{
    int status = STATUS_OK;
    IplImage* I = NULL;
    const char* image_name = "test_ellipse.bmp";
    const char* initials_name = "test_initials.dat";
    unsigned int num_blobs = 2;
    const char* out_file_name = "test_out.dat";

    if(argc >= 2)
    {
        num_blobs = atoi(argv[1]);
    }

    int num_init = num_blobs;
    if(argc >= 3)
    {
        num_init = atoi(argv[2]);
    }

    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> xx;
    std::vector<double> xy;
    std::vector<double> yy;
    double x_i, y_i, xx_i, xy_i, yy_i;

    std::ifstream in_file;
    in_file.open(initials_name);
    if(!in_file.good())
    {
        std::cerr << "Unable to open " << initials_name << std::endl;
        return 1;
    }

    for(unsigned int i = 0; i < num_init; i++)
    {
        in_file >> x_i;
        in_file >> y_i;
        in_file >> xx_i;
        in_file >> xy_i;
        in_file >> yy_i;
        x.push_back(x_i);
        y.push_back(y_i);
        xx.push_back(xx_i);
        xy.push_back(xy_i);
        yy.push_back(yy_i);
        std::cout << "Data Line:  " << x[i] << " " << y[i] << " " <<
            xx[i] << " " << xy[i] << " " << yy[i] << std::endl;
    }

    std::cout << "Loading image." << std::endl;
    I = cvLoadImage(image_name, CV_LOAD_IMAGE_GRAYSCALE);
    if(!I)
    {
        std::cerr << "ERROR:  Unable to load \"" << image_name << "\"" << std::endl;
        return STATUS_NOT_OK;
    }

    std::cout << "Initializing the YABlobber." << std::endl;

    YABlobber yblobber;
    IplImage* I2 = cvCloneImage(I);
    yblobber.m_bCopySequences = true;

    std::cout << "Blobbing." << std::endl;
    std::vector<YABlob> yblobs = yblobber.FindBlobs(I2, 1, 1, -1, -1);
    
    std::cout << "Found " << yblobs.size() << " YABlobs" << std::endl;
    std::cout << "Initializing Hungarian matcher" << std::endl;
    MT_HungarianMatcher hm;
//    hm.doInit(x.size(), yblobs.size());
    hm.doInit(yblobs.size(), x.size());
    std::vector<int> match_results(yblobs.size());

    double d2;
    std::cout << "Filling match table" << std::endl;
    for(unsigned int i = 0; i < yblobs.size(); i++)
    {
        for(unsigned int j = 0; j < x.size(); j++)
        {
            d2 = (yblobs[i].COMx - x[j])*(yblobs[i].COMx - x[j]) +
                (yblobs[i].COMy - y[j])*(yblobs[i].COMy - y[j]);
            std::cout << "dist(" << i << ", " << j << "): " << d2 << std::endl;
            hm.setValue(i, j, d2);
        }
    }

    std::cout << "Run Hungarian matcher" << std::endl;
    hm.doMatch(&match_results);

    std::cout << "Match results: " << std::endl;
    for(unsigned int i = 0; i < match_results.size(); i++)
    {
        std::cout << "\t" <<  i << ":  " << match_results[i] << std::endl;
    }
    
    std::cout << "Painting the blob" << std::endl;
    cvZero(I2);
    DSGY_PaintYABlobIntoImage(yblobs[0], I2);

    std::cout << "Initializing the DSGYlobber." << std::endl;
    
    DSGYBlobber blobber(num_blobs);
    blobber.setTestOut(stdout);    
    blobber.setInitials(x, y, xx, xy, yy);

    std::cout << "Blobbing." << std::endl;
    std::vector<GYBlob> blobs;
    blobs = blobber.findBlobs(I2, num_blobs);

    std::cout << "Blobbing done." << std::endl;

    std::ofstream out_file;
    out_file.open(out_file_name);

    for(unsigned int i = 0; i < num_blobs; i++)
    {
        out_file << blobs[i].m_dXCentre << " " <<
            blobs[i].m_dYCentre << " " <<
            blobs[i].m_dOrientation << " " <<
            blobs[i].m_dArea << " " <<
            blobs[i].m_dMajorAxis << " " <<
            blobs[i].m_dMinorAxis << std::endl;
    }

    return status;
}
