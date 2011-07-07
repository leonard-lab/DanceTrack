#include <iostream>
#include <fstream>

#include "BiCC.h"

#define IDX(i,j) i*cols + j

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
    unsigned int rows = 3;
    unsigned int cols = 3;
    const char* in_file_name = "Adj.dat";
    const char* out_file_name = "BiCCout.dat";

    if(argc >=2)
    {
        rows = atoi(argv[1]);
    }
    if(argc >= 3)
    {
        cols = atoi(argv[2]);
    }
 
    std::cout << "Allocating matrices" << std::endl;
    unsigned int* adj = BiCC_CreateMat(rows, cols);
    unsigned int* label_M = BiCC_CreateMat(rows, cols);
    unsigned int* label = BiCC_CreateMat(1, rows + cols);

    std::cout << "Setting adjacency" << std::endl;

    std::ifstream in_file;
    in_file.open(in_file_name);
    if(!in_file.good())
    {
        std::cerr << "Unable to open " << in_file_name << std::endl;
        return 1;
    }
    double r;
    for(unsigned int i = 0; i < rows*cols; i++)
    {
        in_file >> r;
        adj[i] = r;
    }

    std::cout << "Adjacency:" << std::endl;
    spit_mat(adj, rows, cols);

    int n = BiCC_Do(adj, rows, cols, label, label_M);

    std::cout << "Found " << n << " components.  Label matrix:" << std::endl;
    spit_mat(label_M, rows, cols);
    std::cout << "Label vector: " << std::endl;
    spit_mat(label, 1, rows + cols);
    
    std::ofstream out_file;
    out_file.open(out_file_name);
    for(unsigned int i = 0; i < rows*cols; i++)
    {
        out_file << label_M[i] << " ";
    }
    out_file << std::endl;

    BiCC_ReleaseMat(adj);
    BiCC_ReleaseMat(label);
    BiCC_ReleaseMat(label_M);

    return 0;
}
