#ifndef JOINT_VALUE_H
#define JOINT_VALUE_H

#include <array>
#include <iostream>
#include <string>
#include <math.h>

using JointValue = std::array<float, 5>;

inline std::ostream& operator << (std::ostream& out, const JointValue& jval)
{
    out << "<";
    for ( size_t i = 0; i < jval.size(); i++ )
    {
        out << jval[i] << " ";
    }
    out << ">";
    return out;
}

inline JointValue operator + (const JointValue& jval_1, const JointValue& jval_2)
{
    JointValue sum;
    for ( size_t i = 0; i < jval_1.size(); i++ )
    {
        sum[i] = jval_1[i] + jval_2[i];
    }
    return sum;
}

inline JointValue operator - (const JointValue& jval_1, const JointValue& jval_2)
{
    JointValue diff;
    for ( size_t i = 0; i < jval_1.size(); i++ )
    {
        diff[i] = jval_1[i] - jval_2[i];
    }
    return diff;
}

inline JointValue operator * (const JointValue& jval, const float& scalar)
{
    JointValue scaled;
    for ( size_t i = 0; i < jval.size(); i++ )
    {
        scaled[i] = jval[i] * scalar;
    }
    return scaled;
}

inline bool operator == (const JointValue& jval_1, const JointValue& jval_2)
{
    for ( size_t i = 0; i < jval_1.size(); i++ )
    {
        if ( fabs(jval_1[i] - jval_2[i]) > 1e-6f )
        {
            return false;
        }
    }
    return true;
}

#endif // JOINT_VALUE_H
