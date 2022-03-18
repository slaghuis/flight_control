#ifndef POINT2D_H
#define POINT2D_H

// Custom type
struct Point2D
{
    double x, y;
};

// It is recommended (or, in some cases, mandatory) to define a template
// specialization of convertFromString that converts a string to Point2D.
namespace BT
{
template <> inline Point2D convertFromString(StringView str)
{
    // real numbers separated by semicolons
    auto parts = splitString(str, ';');
    if (parts.size() != 2)
    {
        throw RuntimeError("invalid input)");
    }
    else{
        Point2D output;
        output.x     = convertFromString<double>(parts[0]);
        output.y     = convertFromString<double>(parts[1]);
        return output;
    }
}

template <> inline std::vector<Point2D> convertFromString(StringView str)
{
    // x;y|x;y|x;y
    std::vector<Point2D> mult;
    auto points = splitString(str, '|');
    mult.reserve(points.size());
    
    for (std::size_t i=0; i<points.size(); i++)
    {
      auto parts = splitString(points[i], ';');
      if (parts.size() != 2)
      {
        throw RuntimeError("invalid input)");
      }
      else{
          Point2D output;
          output.x     = convertFromString<double>(parts[0]);
          output.y     = convertFromString<double>(parts[1]);
          
          mult.push_back(output);
      }
    }
    
    return mult;
}

} // end namespace BT

#endif  // POINT2D_H
