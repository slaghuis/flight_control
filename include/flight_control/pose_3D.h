#ifndef POSE3D_H
#define POSE3D_H

// Custom type
struct Pose3D
{
    double x, y, z, theta;
};

inline void SleepMS(int ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

// It is recommended (or, in some cases, mandatory) to define a template
// specialization of convertFromString that converts a string to Position2D.
namespace BT
{
template <> inline Pose3D convertFromString(StringView str)
{
    // real numbers separated by semicolons
    auto parts = splitString(str, ';');
    if (parts.size() != 4)
    {
        throw RuntimeError("invalid input)");
    }
    else{
        Pose3D output;
        output.x     = convertFromString<double>(parts[0]);
        output.y     = convertFromString<double>(parts[1]);
        output.z     = convertFromString<double>(parts[2]);
        output.theta = convertFromString<double>(parts[3]);
        return output;
    }
}
} // end namespace BT

#endif  // POSE3D_H