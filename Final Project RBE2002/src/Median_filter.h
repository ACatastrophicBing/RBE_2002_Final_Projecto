#ifndef MEDIAN_FILTER
#define MEDIAN_FILTER

#include <Romi32U4.h>

class MedianFilter{
    private:
        float measurements[5] = {0};//holds the actual measurements
        float copy[5] = {0};//copy of measurements which will be sorted
        void exch(float array[], int i, int j);//exchanges elements in given array
        
    public:
        void Sort();//sorts the copy array from least to greatest
        void Init(void);
        float Filter(float);
};

#endif