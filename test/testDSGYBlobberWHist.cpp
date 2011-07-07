#include <iostream>
#include <fstream>

#include "DSGYBlobber.h"
#include "BiCC.h"
#include "MT/MT_Tracking/trackers/YA/YABlobber.h"
#include "MT/MT_Tracking/cv/MT_HungarianMatcher.h"

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
    std::vector<double> o;    
    std::vector<double> xx;
    std::vector<double> xy;
    std::vector<double> yy;
    std::vector<double> a;
    std::vector<double> m;
    std::vector<double> M;    
    double x_i, y_i, o_i, xx_i, xy_i, yy_i;

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
        o.push_back(o_i);
        xx.push_back(xx_i);
        xy.push_back(xy_i);
        yy.push_back(yy_i);
        a.push_back(0);
        m.push_back(0);
        M.push_back(0);        
        std::cout << "Data Line:  " << x[i] << " " << y[i] << " " <<
            xx[i] << " " << xy[i] << " " << yy[i] << std::endl;
    }

    std::vector<double> x_o(x);
    std::vector<double> y_o(y);
    std::vector<double> o_o(o);
    std::vector<double> a_o(a);
    std::vector<double> m_o(m);
    std::vector<double> M_o(M);    
    std::vector<double> xx_o(xx);
    std::vector<double> xy_o(xy);
    std::vector<double> yy_o(yy);

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

    unsigned int rows = x.size();
    unsigned int cols = yblobs.size();
    unsigned int* adj = BiCC_CreateMat(rows, cols);
    unsigned int* label_M = BiCC_CreateMat(rows, cols);
    unsigned int* label = BiCC_CreateMat(1, rows + cols);

    double dx;
    double dy;
    double rho;
    for(unsigned int i_prior = 0; i_prior < rows; i_prior++)    
    {
        for(unsigned int j_blob = 0; j_blob < cols; j_blob++)
        {
            dx = x[i_prior] - yblobs[j_blob].COMx;
            dy = y[i_prior] - yblobs[j_blob].COMy;
            rho = 1.25*(max(xx[i_prior], yy[i_prior])
                        + (yblobs[j_blob].major_axis)*(yblobs[j_blob].major_axis));
            std::cout << i_prior << ", " << j_blob << ": "
                      << dx << " " << dy << " " << rho << std::endl;
            if(dx*dx + dy*dy < rho)
            {
                adj[i_prior*cols+j_blob] = 1;
            }
        }
    }
    std::cout << "Adjacency matrix: " << std::endl;
    spit_mat(adj, rows, cols);

    std::cout << "Performing BiCC" << std::endl;
    int n_cc = BiCC_Do(adj, rows, cols, label, label_M);
    std::cout << "Found " << n_cc << " components.  Labels:" << std::endl;
    spit_mat(label_M, rows, cols);
    spit_mat(label, 1, rows+cols);

    std::vector<unsigned int> priors_this_comp(0);
    std::vector<unsigned int> blobs_this_comp(0);
    for(unsigned int i = 1; i <= min(n_cc, (int) rows); i++)
    {
        priors_this_comp.resize(0);
        blobs_this_comp.resize(0);
        for(unsigned int k = 0; k < rows+cols; k++)
        {
            if(label[k] == i)
            {
                if(k < rows)
                {
                    priors_this_comp.push_back(k);
                }
                else
                {
                    blobs_this_comp.push_back(k-rows);
                }
            }
        }

        unsigned int nblobs = blobs_this_comp.size();
        unsigned int nobj = priors_this_comp.size();
        
        std::cout << "In the " << i << "th component: " << std::endl;
        std::cout << "Priors:  ";
        for(unsigned int k = 0; k < nobj; k++)
        {
            std::cout << priors_this_comp[k] << " ";
        }
        std::cout << std::endl << "Blobs: ";
        for(unsigned int k = 0; k < nblobs; k++)
        {
            std::cout << blobs_this_comp[k] << " ";
        }
        std::cout << std::endl;

        if(nobj > 0)
        {
            if(nblobs == 0)
            {
                std::cout << "No blobs found, using previous state" << std::endl;
                /* shouldn't need to do anything since we copied the
                   states */
            }
            else if(nobj == 1 && nblobs == 1)
            {
                std::cout << "One-to-one relationship, taking info from raw blob" << std::endl;
                unsigned int px = priors_this_comp[0];
                YABlob* p_b = &yblobs[blobs_this_comp[0]];
                
                x_o[px] = p_b->COMx;
                y_o[px] = p_b->COMy;
                o_o[px] = p_b->orientation;
                a_o[px] = p_b->area;
                m_o[px] = p_b->minor_axis;
                M_o[px] = p_b->major_axis;                
                xx_o[px] = p_b->XX;
                xy_o[px] = p_b->XY;
                yy_o[px] = p_b->YY;                
            }
            else
            {
                std::cout << "Not a one-to-one relationship, using EMMG" << std::endl;

                std::cout << "Painting the blobs" << std::endl;
                cvZero(I2);                
                for(unsigned int k = 0; k < blobs_this_comp.size(); k++)
                {
                    DSGY_PaintYABlobIntoImage(yblobs[blobs_this_comp[k]], I2);
                }

                std::cout << "Initializing the DSGYlobber." << std::endl;

                std::vector<double> x_b(0);
                std::vector<double> y_b(0);
                std::vector<double> xx_b(0);
                std::vector<double> xy_b(0);
                std::vector<double> yy_b(0);
                unsigned int px;
                for(unsigned int k = 0; k < priors_this_comp.size(); k++)
                {
                    px = priors_this_comp[k];
                    x_b.push_back(x[px]);
                    y_b.push_back(y[px]);
                    xx_b.push_back(xx[px]);
                    xy_b.push_back(xy[px]);
                    yy_b.push_back(yy[px]);                    
                }
                
                DSGYBlobber blobber(nobj);
                blobber.setTestOut(stdout);    
                blobber.setInitials(x_b, y_b, xx_b, xy_b, yy_b);

                std::cout << "Blobbing." << std::endl;
                std::vector<GYBlob> blobs;
                blobs = blobber.findBlobs(I2, nobj);

                std::cout << "Blobbing done." << std::endl;

                for(unsigned int k = 0; k < priors_this_comp.size(); k++)
                {
                    px = priors_this_comp[k];
                    x_o[px] = blobs[k].m_dXCentre;
                    y_o[px] = blobs[k].m_dYCentre;
                    o_o[px] = blobs[k].m_dOrientation;
                    xx_o[px] = blobs[k].m_dXXMoment;
                    xy_o[px] = blobs[k].m_dXYMoment;
                    yy_o[px] = blobs[k].m_dYYMoment;
                    a_o[px] = blobs[k].m_dArea;
                    M_o[px] = blobs[k].m_dMajorAxis;
                    m_o[px] = blobs[k].m_dMinorAxis;                    
                }
                
            }
        }
        
    }

    std::ofstream out_file;
    out_file.open(out_file_name);

    for(unsigned int i = 0; i < num_blobs; i++)
    {
        out_file << x_o[i] << " " <<
            y_o[i] << " " <<
            o_o[i] << " " <<
            a_o[i] << " " <<
            M_o[i] << " " <<
            m_o[i] << std::endl;
    }

    BiCC_ReleaseMat(adj);
    BiCC_ReleaseMat(label);
    BiCC_ReleaseMat(label_M);    

    return status;
}
