#include <cstdlib>
#include <iostream>

static unsigned int g_Rows = 0;
static unsigned int g_Cols = 0;
static unsigned int g_CurrentLabel = 0;
static const unsigned int* g_adj = NULL;
static unsigned int* g_label = NULL;
static unsigned int* g_label_M = NULL;

#define ADJACENT(i,j) (g_adj[i*g_Cols + j])
#define ROW_LABEL(i) g_label[i]
#define COL_LABEL(j) g_label[rows + j]
#define UNLABELED_ADJACENT(i,j) (g_label_M[i*g_Cols + j] == 0) && (g_adj[i*g_Cols + j])
#define LABELM(i,j) g_label_M[i*g_Cols + j] = g_CurrentLabel
#define LABEL(i,j) g_label[i] = g_CurrentLabel; g_label[g_Rows + j] = g_CurrentLabel;

static void follow_col(unsigned int j);
static void follow_row(unsigned int i);

unsigned int* BiCC_CreateMat(unsigned int rows, unsigned int cols)
{
    unsigned int* r = (unsigned int*) calloc(rows*cols, sizeof(unsigned int));
    for(unsigned int i = 0; i < rows*cols; i++)
    {
        r[i] = 0;
    }
    return r;
}

void BiCC_ReleaseMat(unsigned int* mat)
{
    free(mat);
}

int BiCC_Do(const unsigned int* adj_sub,
            unsigned int rows,
            unsigned int cols,
            unsigned int* label,
            unsigned int* label_M)
{
    int count = 0;
    g_Rows = rows;
    g_Cols = cols;
    g_adj = adj_sub;
    g_label = label;
    g_label_M = label_M;

    int empty_row;
    g_CurrentLabel = 0;
    /* move along the rows (top), follow any components */
    for(unsigned int i = 0; i < rows; i++)
    {
        empty_row = 1;
        for(unsigned int j = 0; j < cols; j++)
        {
            if(ADJACENT(i,j))
            {
                empty_row = 0;
            }
            
            if(UNLABELED_ADJACENT(i,j))
            {
                g_CurrentLabel++;
                LABELM(i, j);
                LABEL(i,j);
                follow_row(i);
                follow_col(j);
            }
        }
        if(empty_row)
        {
            ROW_LABEL(i) = ++g_CurrentLabel;
        }
    }

    for(unsigned int j = 0; j < cols; j++)
    {
        if(g_label[rows + j] == 0)
        {
            g_label[rows+j] = ++g_CurrentLabel;
        }
    }
    
    g_adj = NULL;
    g_label = NULL;
    g_label_M = NULL;

    return g_CurrentLabel;
}

static void follow_col(unsigned int j)
{
    for(unsigned int i = 0; i < g_Rows; i++)
    {
        if(UNLABELED_ADJACENT(i,j))
        {
            LABELM(i, j);
            LABEL(i,j);
            follow_row(i);
            follow_col(j);
        }
    }
}

static void follow_row(unsigned int i)
{
    for(unsigned int j = 0; j < g_Cols; j++)
    {
        if(UNLABELED_ADJACENT(i,j))
        {
            LABELM(i,j);
            LABEL(i,j);
            follow_col(j);
            follow_row(i);
        }
    }
}
