#include <iostream>
#include <fstream>

#include "DSGYBlobber.h"

const int STATUS_OK = 0;
const int STATUS_NOT_OK = 1;

int main(int argc, char** argv)
{
    int status = STATUS_OK;
    IplImage* I = NULL;
    const char* image_name = "test_ellipse.bmp";
    unsigned int num_blobs = 2;
    const char* out_file_name = "test_out.dat";

    if(argc == 2)
    {
        num_blobs = atoi(argv[1]);
    }

    std::cout << "Loading image." << std::endl;
    I = cvLoadImage(image_name, CV_LOAD_IMAGE_GRAYSCALE);
    if(!I)
    {
        std::cerr << "ERROR:  Unable to load \"" << image_name << "\"" << std::endl;
        return STATUS_NOT_OK;
    }

    std::cout << "Initializing the blobber." << std::endl;
    DSGYBlobber blobber(num_blobs);
    blobber.setTestOut(stdout);

    std::cout << "Blobbing." << std::endl;
    std::vector<GYBlob> blobs;
    blobs = blobber.findBlobs(I, num_blobs);

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
