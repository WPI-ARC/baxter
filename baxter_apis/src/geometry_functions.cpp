#include "geometry_functions.h"
#include <cmath>
#include <iostream>

Point::Point(double x, double y, double z)
{
    this->x = x;
    this->y = y;
    this->z = z;
}

Point	Point::operator +(Vector rhs)
{
    Point x;

    x.x = this->x + rhs.x;
    x.y = this->y + rhs.y;
    x.z = this->z + rhs.z;

    return x;
}

Point	Point::operator -(Vector rhs)
{
    Point x;

    x.x = this->x - rhs.x;
    x.y = this->y - rhs.y;
    x.z = this->z - rhs.z;

    return x;
}

Vector	Point::operator -(Point rhs)
{
    Vector x;

    x.x = this->x - rhs.x;
    x.y = this->y - rhs.y;
    x.z = this->z - rhs.z;

    return x;
}

Point&	Point::operator =(Point rhs)
{
    this->x = rhs.x;
    this->y = rhs.y;
    this->z = rhs.z;

    return *this;
}

bool	Point::operator==(Point rhs)
{
    if ((this->x == rhs.x) && (this->x == rhs.x) && (this->x == rhs.x))
        return true;
    else
        return false;
}

bool	Point::operator!=(Point rhs)
{
    if ((this->x == rhs.x) && (this->x == rhs.x) && (this->x == rhs.x))
        return false;
    else
        return true;
}

Vector	Vector::operator*(Vector rhs)
{
    Vector x;
    Vector a = (*this);
    Vector b = rhs;

    x.x = (a.y*b.z) - (a.z*b.y);
    x.y = (a.z*b.x) - (a.x*b.z);
    x.z = (a.x*b.y) - (a.y*b.x);

    return x;
}

Vector::Vector(double x, double y, double z)
{
    this->x = x;
    this->y = y;
    this->z = z;
}

Vector& Vector::operator =(Vector rhs)
{
    this->x = rhs.x;
    this->y = rhs.y;
    this->z = rhs.z;

    return *this;
}

bool Vector::operator ==(Vector rhs)
{
    if ((this->x == rhs.x) && (this->x == rhs.x) && (this->x == rhs.x))
        return true;
    else
        return false;
}

bool Vector::operator!=(Vector rhs)
{
    if ((this->x == rhs.x) && (this->x == rhs.x) && (this->x == rhs.x))
        return false;
    else
        return true;
}

Vector operator*(Vector rhs, double scalar)
{
	Vector x;

	x.x = rhs.x * scalar;
	x.y = rhs.y * scalar;
	x.z = rhs.z * scalar;

    return x;
}

Vector operator*(double scalar, Vector rhs)
{
	Vector x;

	x.x = rhs.x * scalar;
	x.y = rhs.y * scalar;
	x.z = rhs.z * scalar;

    return x;
}

//===================================================================
// intersect2D_2Segments(): find the 2D intersection of 2 finite segments
//    Input:  two finite segments S1 and S2
//    Output: *I0 = intersect point (when it exists)
//            *I1 =  endpoint of intersect segment [I0,I1] (when it exists)
//    Return: 0=disjoint (no intersect)
//            1=intersect  in unique point I0
//            2=overlap  in segment from I0 to I1
int
intersect2D_2Segments( Segment S1, Segment S2, Point* I0, Point* I1 )
{
    Vector    u = S1.P1 - S1.P0;
    Vector    v = S2.P1 - S2.P0;
    Vector    w = S1.P0 - S2.P0;
    double     D = perp(u,v);

    // test if  they are parallel (includes either being a point)
    if (fabs(D) < SMALL_NUM) {           // S1 and S2 are parallel
        if (perp(u,w) != 0 || perp(v,w) != 0)  {
            return 0;                    // they are NOT collinear
        }
        // they are collinear or degenerate
        // check if they are degenerate  points
        double du = dot(u,u);
        double dv = dot(v,v);
        if (du==0 && dv==0) {            // both segments are points
            if (S1.P0 !=  S2.P0)         // they are distinct  points
                 return 0;
            *I0 = S1.P0;                 // they are the same point
            return 1;
        }
        if (du==0) {                     // S1 is a single point
            if  (inSegment(S1.P0, S2) == 0)  // but is not in S2
                 return 0;
            *I0 = S1.P0;
            return 1;
        }
        if (dv==0) {                     // S2 a single point
            if  (inSegment(S2.P0, S1) == 0)  // but is not in S1
                 return 0;
            *I0 = S2.P0;
            return 1;
        }
        // they are collinear segments - get  overlap (or not)
        double t0, t1;                    // endpoints of S1 in eqn for S2
        Vector w2 = S1.P1 - S2.P0;
        if (v.x != 0) {
                 t0 = w.x / v.x;
                 t1 = w2.x / v.x;
        }
        else {
                 t0 = w.y / v.y;
                 t1 = w2.y / v.y;
        }
        if (t0 > t1) {                   // must have t0 smaller than t1
                 double t=t0; t0=t1; t1=t;    // swap if not
        }
        if (t0 > 1 || t1 < 0) {
            return 0;      // NO overlap
        }
        t0 = t0<0? 0 : t0;               // clip to min 0
        t1 = t1>1? 1 : t1;               // clip to max 1
        if (t0 == t1) {                  // intersect is a point
            *I0 = S2.P0 +  t0 * v;
            return 1;
        }

        // they overlap in a valid subsegment
        *I0 = S2.P0 + t0 * v;
        *I1 = S2.P0 + t1 * v;
        return 2;
    }

    // the segments are skew and may intersect in a point
    // get the intersect parameter for S1
    double     sI = perp(v,w) / D;
    if (sI < 0 || sI > 1)                // no intersect with S1
        return 0;

    // get the intersect parameter for S2
    double     tI = perp(u,w) / D;
    if (tI < 0 || tI > 1)                // no intersect with S2
        return 0;

    *I0 = S1.P0 + sI * u;                // compute S1 intersect point
    return 1;
}
//===================================================================
 


// inSegment(): determine if a point is inside a segment
//    Input:  a point P, and a collinear segment S
//    Return: 1 = P is inside S
//            0 = P is  not inside S
int
inSegment( Point P, Segment S)
{
    if (S.P0.x != S.P1.x) {    // S is not  vertical
        if (S.P0.x <= P.x && P.x <= S.P1.x)
            return 1;
        if (S.P0.x >= P.x && P.x >= S.P1.x)
            return 1;
    }
    else {    // S is vertical, so test y  coordinate
        if (S.P0.y <= P.y && P.y <= S.P1.y)
            return 1;
        if (S.P0.y >= P.y && P.y >= S.P1.y)
            return 1;
    }
    return 0;
}
//===================================================================
 


// intersect3D_SegmentPlane(): find the 3D intersection of a segment and a plane
//    Input:  S = a segment, and Pn = a plane = {Point V0;  Vector n;}
//    Output: *I0 = the intersect point (when it exists)
//    Return: 0 = disjoint (no intersection)
//            1 =  intersection in the unique point *I0
//            2 = the  segment lies in the plane
int
intersect3D_SegmentPlane( Segment S, Plane Pn, Point* I )
{
    Vector    u = S.P1 - S.P0;
    Vector    w = S.P0 - Pn.V0;

    double     D = dot(Pn.n, u);
    double     N = -dot(Pn.n, w);

    if (fabs(D) < SMALL_NUM) {           // segment is parallel to plane
        if (N == 0)                      // segment lies in plane
            return 2;
        else
            return 0;                    // no intersection
    }
    // they are not parallel
    // compute intersect param
    double sI = N / D;
    if (sI < 0 || sI > 1)
        return 0;                        // no intersection

    *I = S.P0 + sI * u;                  // compute segment intersect point
    return 1;
}
//===================================================================
 


// intersect3D_2Planes(): find the 3D intersection of two planes
//    Input:  two planes Pn1 and Pn2
//    Output: *L = the intersection line (when it exists)
//    Return: 0 = disjoint (no intersection)
//            1 = the two  planes coincide
//            2 =  intersection in the unique line *L
int
intersect3D_2Planes( Plane Pn1, Plane Pn2, Line* L )
{
    Vector   u = Pn1.n * Pn2.n;          // cross product
    double    ax = (u.x >= 0 ? u.x : -u.x);
    double    ay = (u.y >= 0 ? u.y : -u.y);
    double    az = (u.z >= 0 ? u.z : -u.z);

    // test if the two planes are parallel
    if ((ax+ay+az) < SMALL_NUM) {        // Pn1 and Pn2 are near parallel
        // test if disjoint or coincide
        Vector   v = Pn2.V0 -  Pn1.V0;
        if (dot(Pn1.n, v) == 0)          // Pn2.V0 lies in Pn1
            return 1;                    // Pn1 and Pn2 coincide
        else 
            return 0;                    // Pn1 and Pn2 are disjoint
    }

    // Pn1 and Pn2 intersect in a line
    // first determine max abs coordinate of cross product
    int      maxc;                       // max coordinate
    if (ax > ay) {
        if (ax > az)
             maxc =  1;
        else maxc = 3;
    }
    else {
        if (ay > az)
             maxc =  2;
        else maxc = 3;
    }

    // next, to get a point on the intersect line
    // zero the max coord, and solve for the other two
    Point    iP;                // intersect point
    double    d1, d2;            // the constants in the 2 plane equations
    d1 = -dot(Pn1.n, Pn1.V0);  // note: could be pre-stored  with plane
    d2 = -dot(Pn2.n, Pn2.V0);  // ditto

    switch (maxc) {             // select max coordinate
    case 1:                     // intersect with x=0
        iP.x = 0;
        iP.y = (d2*Pn1.n.z - d1*Pn2.n.z) /  u.x;
        iP.z = (d1*Pn2.n.y - d2*Pn1.n.y) /  u.x;
        break;
    case 2:                     // intersect with y=0
        iP.x = (d1*Pn2.n.z - d2*Pn1.n.z) /  u.y;
        iP.y = 0;
        iP.z = (d2*Pn1.n.x - d1*Pn2.n.x) /  u.y;
        break;
    case 3:                     // intersect with z=0
        iP.x = (d2*Pn1.n.y - d1*Pn2.n.y) /  u.z;
        iP.y = (d1*Pn2.n.x - d2*Pn1.n.x) /  u.z;
        iP.z = 0;
    }
    L->P0 = iP;
    L->P1 = iP + u;
    return 2;
}
//===================================================================

// intersect3D_RayTriangle(): find the 3D intersection of a ray with a triangle
//    Input:  a ray R, and a triangle T
//    Output: *I = intersection point (when it exists)
//    Return: -1 = triangle is degenerate (a segment or point)
//             0 =  disjoint (no intersect)
//             1 =  intersect in unique point I1
//             2 =  are in the same plane
int
intersect3D_RayTriangle( Ray R, Triangle T, Point* I )
{
    Vector    u, v, n;              // triangle vectors
    Vector    dir, w0, w;           // ray vectors
    double     r, a, b;              // params to calc ray-plane intersect

    // get triangle edge vectors and plane normal
    u = T.V1 - T.V0;
    v = T.V2 - T.V0;
    n = u * v;              // cross product
    //if (n == (Vector)0)             // triangle is degenerate
    if ((n.x == 0.0) && (n.y == 0.0) && (n.z == 0.0))
    {
        std::cout << "Triangle is degenerate" << n.x << " " << n.y << " " << n.z;
        char c;
        std::cin >> c;
        return -1;                  // do not deal with this case
    }

    dir = R.P1 - R.P0;              // ray direction vector
    w0 = R.P0 - T.V0;
    a = -dot(n,w0);
    b = dot(n,dir);
    if (fabs(b) < SMALL_NUM) {     // ray is  parallel to triangle plane
        if (a == 0)                 // ray lies in triangle plane
            return 2;
        else return 0;              // ray disjoint from plane
    }

    // get intersect point of ray with triangle plane
    r = a / b;
    if (r < 0.0)                    // ray goes away from triangle
        return 0;                   // => no intersect
    // for a segment, also test if (r > 1.0) => no intersect

    *I = R.P0 + r * dir;            // intersect point of ray and plane

    // is I inside T?
    double    uu, uv, vv, wu, wv, D;
    uu = dot(u,u);
    uv = dot(u,v);
    vv = dot(v,v);
    w = *I - T.V0;
    wu = dot(w,u);
    wv = dot(w,v);
    D = uv * uv - uu * vv;

    // get and test parametric coords
    double s, t;
    s = (uv * wv - vv * wu) / D;
    if (s < 0.0 || s > 1.0)         // I is outside T
        return 0;
    t = (uv * wu - uu * wv) / D;
    if (t < 0.0 || (s + t) > 1.0)  // I is outside T
        return 0;

    return 1;                       // I is in T
}
//===================================================================
