#ifndef MATH_CLASSES
#define MATH_CLASSES

class Vector
{
    public:
        double x, y, z;

        Vector(double x = 0.0, double y = 0.0, double z = 0.0);

        Vector& operator =(Vector rhs);
        bool	operator==(Vector rhs);
        bool    operator!=(Vector rhs);
        Vector	operator *(Vector rhs);
};

class Point
{
    public:
        double x, y, z;

        Point(double x = 0.0, double y = 0.0, double z = 0.0);

        Point&	operator =(Point rhs);
        bool	operator==(Point rhs);
        bool    operator!=(Point rhs);
        Point	operator +(Vector rhs);
        Point	operator -(Vector rhs);
        Vector	operator -(Point rhs);


};

class Line
{
    public:
        Point P0, P1;
};

class Segment
{
    public:
        Point P0, P1;
};

class Plane
{
    public:
        Point	V0;
        Vector	n;
};

class Ray
{
    public:
        Point	P0, P1;
};

class Triangle
{
    public:
        Point	V0, V1, V2;
};

#endif

#define SMALL_NUM   0.00000001 // anything that avoids division overflow
// dot product (3D) which allows vector operations in arguments
#define dot(u,v)   ((u).x * (v).x + (u).y * (v).y + (u).z * (v).z)
#define perp(u,v)  ((u).x * (v).y - (u).y * (v).x)  // perp product  (2D)

int intersect2D_2Segments( Segment S1, Segment S2, Point* I0, Point* I1 );
int inSegment( Point P, Segment S);
int intersect3D_SegmentPlane( Segment S, Plane Pn, Point* I );
int intersect3D_2Planes( Plane Pn1, Plane Pn2, Line* L );
int intersect3D_RayTriangle( Ray R, Triangle T, Point* I );
