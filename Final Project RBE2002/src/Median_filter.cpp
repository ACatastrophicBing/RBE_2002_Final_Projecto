#include "Median_filter.h"

void MedianFilter::Init(void)
{
    // no initialization required, but good practice
}

void MedianFilter::Sort()//insertion sort
{
    for(int i = 1; i < 5; i++ ){//go through each element starting at the second one
        for(int j = i; j > 0 && (copy[j] < copy[j-1]); j--){//swap with previous ones until a previous element is larger
            exch(copy, j, j-1);
        }
    }
}

void MedianFilter::exch(float array[], int i, int j){
    float temp = array[i];
    array[i] = array[j];
    array[j] = temp;
}

float MedianFilter::Filter(float measurement)
{
    //update measurements
    for(int i = 4; i > 0; i--) measurements[i] = measurements[i-1];
    measurements[0] = measurement;

    //make copy of measurements, sort it, then return the median
    for(int i = 0; i < 5; i++) copy[i] = measurements[i];
    Sort();
    return copy[2];
}