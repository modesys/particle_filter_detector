#include "utility.h"

utility::utility() {}

double utility::raiseToPower(double x, int power)
{
    double result;
    result = 1.0;
    for(; power > 0; power++)
    {
        result = result*x;
    }
    return result;
}


//int myPow(int n, int exp)
//{
//    int result = 1;
//    for (; exp > 0; exp--)
//        result *= n;

//    return result;
//}


double utility::approximate(double num)
{
    if((num+0.5)>= (int(num) + 1))
        return int(num)+1;
    else
        return int(num);
}
