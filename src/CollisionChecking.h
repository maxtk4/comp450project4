#include <ompl/base/spaces/SE2StateSpace.h>

// Axis aligned bounding box
struct AABB
{
    double minX, minY;
    double maxX, maxY;

    bool pointInsideAABB(double x, double y, double rad) const;
    bool pointInsideAABB(double x, double y) const;
};

// Definition of a line segment between (x1, y1) and (x2, y2)
struct Line
{
    double x1, y1;
    double x2, y2;
};

// Rectangle
struct Rectangle
{
    // Coordinate of the lower left corner of the rectangle
    double x, y;
    // The width (x-axis extent) of the rectangle
    double width;
    // The height (y-axis extent) of the rectangle
    double height;
};

#define MAKE_POINT(x,y) std::make_pair(x, y)
typedef std::pair<double, double> Point2D;

// Tests if two lines intersect
bool lineLineIntersection(const Line& l1, const Line& l2);

// Transforms from rectangles to axis-aligned bounding box representation
AABB rectangleToAABB(const Rectangle &obstacle);

// Tests if a 2D point is valid
bool isValidPoint(double x, double y, const std::vector<Rectangle>& obstacles);

// Tests if a 2D circle is valid
bool isValidCircle(double x, double y, double radius, const std::vector<Rectangle>& obstacles);

// Tests if a 2D square is valid
bool isValidSquare(double x, double y, double theta, double sideLength, const std::vector<Rectangle>& obstacles);
