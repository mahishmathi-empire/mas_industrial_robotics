#ifndef JOINT_VALUE_H
#define JOINT_VALUE_H

#include <array>

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

#endif // JOINT_VALUE_H
