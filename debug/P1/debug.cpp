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

    const char* image_name = "debug.bmp";
    const char* input_blobs_name = "debug_in.dat";
    const char* params_file = "debug_params.dat";

    unsigned int num_blobs = atoi(argv[1]);
    
    const char* out_file_name = "debug_out.dat";

    std::cout << "Loading image." << std::endl;
    I = cvLoadImage(image_name, CV_LOAD_IMAGE_GRAYSCALE);
    if(!I)
    {
        std::cerr << "ERROR:  Unable to load \"" << image_name << "\"" << std::endl;
        return STATUS_NOT_OK;
    }

    FILE* fp = fopen(params_file, "r");
    std::vector<double> p = MT_ReadDoublesToEndOfLine(fp);
    fclose(fp);
    
    std::vector<DSGYA_Blob> in_blobs = readDSGYABlobsFromFile(input_blobs_name);
    std::cout << "Running segmenter" << std::endl;
    
    DSGYA_Segmenter segmenter;
    segmenter.m_iMinBlobPerimeter = p[0];
    segmenter.m_iMinBlobArea = p[1];
    segmenter.m_iMaxBlobArea = p[2];
    segmenter.m_dOverlapFactor = p[3];
    segmenter.setDebugFile(stdout);
    
    std::vector<DSGYA_Blob> out_blobs = segmenter.doSegmentation(I, in_blobs);        
    
    std::cout << "Done.  Writing output" << std::endl;    

    writeDSGYABlobsToFile(out_blobs, out_file_name);

    cvReleaseImage(&I);

    return status;
}
