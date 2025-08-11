
/*
//determines the collision of two convex shapes in 3d

shape shape1[] = {p1.x, p1.y, p1.z, p2.x, p2.y, p2.z, p3.x, p3.y, p3.z}; // side of a tetrahedron defined by three (end) points: P1(2.x, t2.y, t2.z), P2(t1.x, t1.y, t1.z), P3(t3.x, t3.y, t3.z)
shape shape2[] = {size, size, size,  size, size, -size,  size, -size, -size,  size, -size, size }; // side of a cube defined by four (end) points: P1(size, size, size), P2(size, size, -size), P3(size, -size, -size), P4(size, -size, size)

int dim_ts = sizeof(shape1)/sizeof(*shape1); // (amount of points P_i of the shape 1) -> here: 3
int dim_shape2 = sizeof(shape2)/sizeof(*shape2); // (amount of points P_i of the shape 2) -> here: 4

bool check_collision = gjk(shape1, shape2, dim_ts, dim_shape2); // 0 ... no collision, 1 ... collision
*/

#ifndef __GJK_H_INCLUDED__
#define __GJK_H_INCLUDED__

#include <math.h>
#include <iostream>
#include <cstdlib>	// rand()

const float PI = acos(-1.0f);

// GJK functions

// set of the shapes to be investigated to intersect
// shape = a point (x/y/z) or a/an set/array of points
class shape
{
	public:
		float x;
		float y;
		float z;

		shape negate();         // x -> -x ; y -> -y ; z -> -z
		float dot(shape d);     // dot product
		shape cross(shape cpt); // cross product of two vectors(shapes)
		void set(shape set);    // set a vector(shape) to a given x/y/z value(s)

		shape rot_x(float rot_x); // rotates  shape around the x-axis
		shape rot_y(float rot_y); // rotates  shape around the y-axis
		shape rot_z(float rot_z); // rotates  shape around the z-axis
};
// set of the shapes to be investigated to intersect


// class & functions for the simplex(storing the points)
class simplex
{

	public:
		void add(shape simplex_add); // add a new point to the simplex (list)
		void set_zero(void);         // (re)sets all points in the simplex-list to zero
//		void print(void);            // prints all the points in the simplex
		void del(int id);	           // delete vector from the simplex with given id(id = 0, 1, 2)
		shape getLast(void);         // returns the last added point
		shape get(int id);           // returns the x/y/z - values of a given index of the simples with id (= 0 ... 3)
		int size(void);              // returns the amount of points in the simplex list (0 ... 4)

//		void draw(int id, float size); // draw a small rectangle at the given index of the simplex
//		void draw_lines(int id1, int id2); // draw a connection line between the two simplex entries

	private:
		float x[4];
		float y[4];
		float z[4];
};
// class & functions for the simplex(storing the points)


bool gjk(shape *A, shape *B, int dim_a, int dim_b);

#endif
