#include <iostream>
#include <fstream>

#include "DSGYA_Segmenter.h"

const int STATUS_OK = 0;
const int STATUS_NOT_OK = 1;

void spit_mat(unsigned int* m, unsigned int rows, unsigned int cols)
{
    for(unsigned int i = 0; i < rows; i++)
    {
        for(unsigned int j = 0; j < cols; j++)
        {
            std::cout << (int) m[i*cols + j] << " ";
        }
        std::cout << std::endl;
    }
}

int main(int argc, char** argv)
{
    int status = STATUS_OK;
    IplImage* I = NULL;
    const char* image_name = "test_ellipse.bmp";
    const char* initials_name = "test_initials.dat";
    unsigned int num_blobs = 2;
    bool treat_as_first = false;
    const char* out_file_name = "test_out.dat";

    if(argc >= 2)
    {
        num_blobs = atoi(argv[1]);
    }

    if(argc >= 3)
    {
        treat_as_first = true;
    }

    std::vector<DSGYA_Blob> blobs(num_blobs);

    if(!treat_as_first)
    {
        std::ifstream in_file;
        in_file.open(initials_name);
        if(!in_file.good())
        {
            std::cerr << "Unable to open " << initials_name << std::endl;
            return 1;
        }

        double x_i, y_i, xx_i, xy_i, yy_i, M_i, m_i, a_i, o_i;
        for(unsigned int i = 0; i < num_blobs; i++)
        {
            in_file >> x_i;
            in_file >> y_i;
            in_file >> xx_i;
            in_file >> xy_i;
            in_file >> yy_i;
            in_file >> m_i;
            in_file >> M_i;
            in_file >> a_i;
            in_file >> o_i;
            blobs[i] = DSGYA_Blob(x_i, y_i, xx_i, xy_i, yy_i, m_i, M_i, a_i, o_i);
        }
    }

    std::cout << "Loading image." << std::endl;
    I = cvLoadImage(image_name, CV_LOAD_IMAGE_GRAYSCALE);
    if(!I)
    {
        std::cerr << "ERROR:  Unable to load \"" << image_name << "\"" << std::endl;
        return STATUS_NOT_OK;
    }

    std::cout << "Running segmenter" << std::endl;
    DSGYA_Segmenter segmenter;
    segmenter.m_iMinBlobPerimeter = 5;
    segmenter.m_iMinBlobArea = 5;    
    segmenter.setDebugFile(stdout);
    if(treat_as_first)
    {
        blobs = segmenter.segmentFirstFrame(I, num_blobs);
    }
    else
    {
        blobs = segmenter.doSegmentation(I, blobs);        
    }
    std::cout << "Done.  Writing output" << std::endl;    

    std::ofstream out_file;
    out_file.open(out_file_name);

    for(unsigned int i = 0; i < num_blobs; i++)
    {
        out_file << blobs[i].m_dXCenter << " " <<
            blobs[i].m_dYCenter << " " <<     
            blobs[i].m_dXXMoment << " " <<    
            blobs[i].m_dXYMoment << " " <<    
            blobs[i].m_dYYMoment << " " <<                
            blobs[i].m_dMinorAxis << " " <<   
            blobs[i].m_dMajorAxis << " " <<   
            blobs[i].m_dArea << " " <<        
            blobs[i].m_dOrientation << std::endl;
    }

    cvReleaseImage(&I);

    return status;
}
