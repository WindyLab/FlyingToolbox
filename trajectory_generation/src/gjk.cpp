#include "gjk.h" // GJK - header files

// rot a point around the x-axis
shape shape::rot_x(float rot_x) // float rot_x ... angle in units of degrees
{
	shape return_shape;

	return_shape.x = x;
	return_shape.y = y * cos(rot_x * PI / 180.0) - z * sin(rot_x * PI / 180.0);
	return_shape.z = y * sin(rot_x * PI / 180.0) + z * cos(rot_x * PI / 180.0);

	return return_shape;
}
// rot a point around the x-axis

// rot a point around the y-axis
shape shape::rot_y(float rot_y) // float rot_x ... angle in units of degrees
{
	shape return_shape;

	return_shape.x = x * cos(rot_y * PI / 180.0) + z * sin(rot_y * PI / 180.0);
	return_shape.y = y;
	return_shape.z = z * cos(rot_y * PI / 180.0) - x * sin(rot_y * PI / 180.0);

	return return_shape;
}
// rot a point around the y-axis

// rot a point around the z-axis
shape shape::rot_z(float rot_z) // float rot_x ... angle in units of degrees
{
	shape return_shape;

	return_shape.x = x * cos(rot_z * PI / 180.0) - y * sin(rot_z * PI / 180.0);
	return_shape.y = x * sin(rot_z * PI / 180.0) + y * cos(rot_z * PI / 180.0);
	return_shape.z = z;

	return return_shape;
}
// rot a point around the z-axis


// negates the shape(vector)
shape shape::negate()
{
	x = -x;
	y = -y;
	z = -z;
}
// negates the shape(vector)


// dot product of 2 shapes(vectors)
float shape::dot(shape d)
{
	float dotproduct = x*d.x + y*d.y + z*d.z;
	return dotproduct;
}
// dot product of 2 shapes(vectors)

// cross product of 2 shapes(vectors)
shape shape::cross(shape d)
{
	shape crossproduct;

	crossproduct.x = y*d.z - z*d.y;
	crossproduct.y = z*d.x-x*d.z;
	crossproduct.z = x*d.y - y*d.x;

	return crossproduct;
}
// cross product of 2 shapes(vectors)


// set shape to given shape (vectors)
void shape::set(shape d)
{
	x = d.x;
	y = d.y;
	z = d.z;
}
// set shape to given shape (vectors)


// dot product function
float dot_prod(float x1, float y1, float z1, float x2, float y2, float z2)
{
	return (x1*x2)+(y1*y2)+(z1*z2);
}
// dot product function


// triple cross product a x (b x c) = b (a in c) - c (a in b)
shape triple_cross_prod(shape a, shape b, shape c)
{
	shape triple_return;

	triple_return.x = b.x*a.dot(c) - c.x*a.dot(b);
	triple_return.y = b.y*a.dot(c) - c.y*a.dot(b);
	triple_return.z = b.z*a.dot(c) - c.z*a.dot(b);

	return triple_return;
}


// mass middle point
shape mass_middle_point(shape *A, int dim_a)
{

	shape mm;

	mm.x = 0.0;
	mm.y = 0.0;
	mm.z = 0.0;

	for (int i = 0 ; i < dim_a ; i++)
	{
		mm.x += A[i].x;
		mm.y += A[i].y;
		mm.z += A[i].z;
	}


	mm.x = mm.x/dim_a;
	mm.y = mm.y/dim_a;
	mm.z = mm.z/dim_a;

	return mm;

}
// mass middle point


// support function for the GJK-simplex
	shape support_function(shape *A, shape *B, int dim_a, int dim_b, shape sd) // (shape A, shape B, entries of x/y/z elements in shape a and b vec searchdirection)
	{
		// shape A - direction sd
		float max_val1 = -INFINITY;
		int max_index1 = -1;

		float dotp = 0.0;

		for (int i = 0 ; i < dim_a ; i++)
		{


			dotp = dot_prod(A[i].x, A[i].y, A[i].z, sd.x, sd.y, sd.z);

			if (dotp > max_val1)
			{
				max_val1 = dotp;
				max_index1 = i;
			}
		}
		// shape A - direction sd


		// shape B - direction -sd
		float max_val2 = -INFINITY;
		int max_index2 = -1;

		for (int j = 0 ; j < dim_b ; j++)
		{

			dotp = dot_prod(B[j].x, B[j].y, B[j].z, -sd.x, -sd.y, -sd.z);

			if (dotp > max_val2)
			{
				max_val2 = dotp;
				max_index2 = j;
			}
		}

		// shape B - direction -sd

		shape return_vec;

		return_vec.x = A[max_index1].x - B[max_index2].x;
		return_vec.y = A[max_index1].y - B[max_index2].y;
		return_vec.z = A[max_index1].z - B[max_index2].z;

		return return_vec;

	}
// support function for the GJK-simplex


void simplex::add(shape simplex_add)
{
	int i = 3;

	while (i >= 0)
	{
		if (x[i] != -INFINITY)
		{
			break;
		}

		i--;
	}

	i++;

	x[i] = simplex_add.x;
	y[i] = simplex_add.y;
	z[i] = simplex_add.z;

}


void simplex::set_zero(void)
{
	// set all values to -INFINITY or we'll get something like  x[0] / x[1] / x[2] ... =  0 / 1.4013e-45 / 0 / -1.87351e+07

	for(int i = 0 ; i < 4 ; i++)
	{
		x[i] = -INFINITY;
		y[i] = -INFINITY;
		z[i] = -INFINITY;
	}
}


shape simplex::getLast(void)
{
	shape return_last;

	int i = 3;

	while (i >= 0)
	{
		if (x[i] != -INFINITY)
		{
			break;
		}

		i--;
	}

	return_last.x = x[i];
	return_last.y = y[i];
	return_last.z = z[i];

	return return_last;
}

// return element of the simplex with given id ( = 0 ... 3)
shape simplex::get(int id)
{
	shape return_shape;

	if (id < 0 or id > 3)
	{
		std::cout << "can't fetch simplex point with given index " << id << std::endl;
	}
	else
	{
		return_shape.x = x[id];
		return_shape.y = y[id];
		return_shape.z = z[id];

	}

	return return_shape;
}
// return element of the simplex with given id ( = 0 ... 3)

// how many elements (x/y/z) are in the simplex?
int simplex::size(void)
{
	int found = 0;

	for(int i = 0; i < 4 ; i++)
		if (x[i] != -INFINITY)
			found ++;

	return found;
}
// how many elements (x/y/z) are in the simplex?

// delete a given vector(=element) from the simples list
void simplex::del(int id) // id = 1 ... 4
{

	// failsafe -> elements in the simplex
	int simplex_elements = 0;

	for(int i = 0; i < 4 ; i++)
		if (x[i] != -INFINITY)
			simplex_elements ++;
	// failsafe -> elements in the simplex

	if (simplex_elements < 4 or id == 4) // last element is the last added and will never be removed. If there are less than 4 elements in the simplex list we only add new until the list contains 4 elements,
	{
		std::cout << "error -> simplex elements: " << simplex_elements << " | id to remove: " << id << std::endl;
	}
	else
	{

		// cache shapes
		float cx[4], cy[4], cz[4];// This place may has a bug. The origin number is 2, which is absolutely wrong. 
		// Therefore, I change the number as 3.
		// cache shapes

		int c = 0;
		id--; // id = 1 ... 4 but index of simplex list goes from 0 ... 3

		for (int i = 0 ; i < 4 ; i++)
		{
//			std::cout << "  i: " << i << " / c: " << c << std::endl;

			cx[c] = x[i];
			cy[c] = y[i];
			cz[c] = z[i];

			x[i] = -INFINITY;
			y[i] = -INFINITY;
			z[i] = -INFINITY;

			if (i != id)
				c++;
		}

		for (int i = 0 ; i < 3 ; i++)
		{
			x[i] = cx[i];
			y[i] = cy[i];
			z[i] = cz[i];
		}
	}
}
// delete a given vector(=element) from the simples list


// check inside simplex evolution (2, 3, 4 points inside the simplex list)
bool containsOrigin(simplex& simplex, shape& d, int i)
{
	shape a = simplex.getLast(); // get the latest point added to the simplex

	shape a0 = a;
	a0.negate();


	if (simplex.size() == 4) // 4 elements in the simplex -> tetrahedral case
	{
		shape p_d = simplex.get(0); // get the first element -> point d
		shape p_c = simplex.get(1); // get the second element -> point c
		shape p_b = simplex.get(2); // get the third element -> point b

		// AC
		shape ac;
		ac.x = p_c.x - a.x;
		ac.y = p_c.y - a.y;
		ac.z = p_c.z - a.z;
		// AC

		// AB
		shape ab;
		ab.x = p_b.x - a.x;
		ab.y = p_b.y - a.y;
		ab.z = p_b.z - a.z;
		// AB

		// AD
		shape ad;
		ad.x = p_d.x - a.x;
		ad.y = p_d.y - a.y;
		ad.z = p_d.z - a.z;
		// AD


		// ABC
		shape mmp_abc; // mass middle point of the triangle ABC

		mmp_abc.x = (a.x + p_b.x + p_c.x)/3;
		mmp_abc.y = (a.y + p_b.y + p_c.y)/3;
		mmp_abc.z = (a.z + p_b.z + p_c.z)/3;

		shape ac_c_ab = ac.cross(ab); // ac_c_ab = AC x AB  -> normal of the (triangle) surface

		float v_abc = ac_c_ab.dot(a0);

		//delete d
		// ABC



		// ABD
		shape mmp_abd; // mass middle point of the triangle ABC

		mmp_abd.x = (a.x + p_b.x + p_d.x)/3;
		mmp_abd.y = (a.y + p_b.y + p_d.y)/3;
		mmp_abd.z = (a.z + p_b.z + p_d.z)/3;

		shape ab_c_ad = ab.cross(ad); // ab_c_ad = AB x AD  -> normal of the (triangle) surface

		float v_abd = ab_c_ad.dot(a0);

		//delete c
		// ABD

		shape mmp_acd; // mass middle point of the triangle ABC

		mmp_acd.x = (a.x + p_c.x + p_d.x)/3;
		mmp_acd.y = (a.y + p_c.y + p_d.y)/3;
		mmp_acd.z = (a.z + p_c.z + p_d.z)/3;

		shape ad_c_ac = ad.cross(ac); // ad_c_ac = AD x AC  -> normal of the (triangle) surface

		float v_acd = ad_c_ac.dot(a0);

		int amount_neg = 0;
		int amount_pos = 0;

		if (v_acd > 0)
			amount_pos++;
		else
			amount_neg++;

		if (v_abd > 0)
			amount_pos++;
		else
			amount_neg++;

		if (v_abc > 0)
			amount_pos++;
		else
			amount_neg++;

		if (amount_pos == 3) // origin enclosed in the tetrahedron -> we got a collision
		{
			return true;
		}
		else if (amount_neg == 3) // origin enclosed in the tetrahedron -> we got a collision
		{
			return true;
		}
		else // ditch one point, determine new search direction
		{
			if (amount_neg == 2 and amount_pos == 1)
			{

				if (v_acd > 0) // v_acd < 0 -> new search direction: ad_c_ac, ditch point b
				{
					simplex.del(3); // remove point b of the simplex list
					d.set(ad_c_ac); // set new search direction
				}
				else if (v_abd > 0) // v_abd < 0 ->  new search direction: ab_c_ad, ditch point c
				{
					simplex.del(2); // remove point b of the simplex list
					d.set(ab_c_ad); // set new search direction
				}
				else	// v_abc < 0 -> new search direction: ac_c_ab, ditch point d
				{
					simplex.del(1); // remove point b of the simplex list
					d.set(ac_c_ab); // set new search direction
				}
			}
			else if (amount_neg == 1 and amount_pos == 2)
			{
				if (v_acd < 0) // v_acd < 0 -> new search direction: -ad_c_ac, ditch point b
				{

					ad_c_ac.negate();

					simplex.del(3); // remove point b of the simplex list
					d.set(ad_c_ac); // set new search direction
				}
				else if (v_abd < 0) // v_abd < 0 ->  new search direction: -ab_c_ad, ditch point c
				{
					ab_c_ad.negate();

					simplex.del(2); // remove point b of the simplex list
					d.set(ab_c_ad); // set new search direction
				}
				else	// v_abc < 0 -> new search direction: -ac_c_ab, ditch point d
				{

					ac_c_ab.negate();

					simplex.del(1); // remove point b of the simplex list
					d.set(ac_c_ab); // set new search direction
				}
			}
			else
			{
				std::cout << "error(number pos/neg)" << std::endl;
			}
		}
	}
	else if (simplex.size() == 3) // 3 elements in the simplex
	{
		shape return_sd; // new search direction
		return_sd.x = 0;
		return_sd.y = 0;
		return_sd.z = 0;

		shape b = simplex.get(1); // get the second element
		shape c = simplex.get(0); // get the first element

		shape ab; // b - a
		ab.x = b.x - a.x;
		ab.y = b.y - a.y;
		ab.z = b.z - a.z;

		shape ac; // c - a
		ac.x = c.x - a.x;
		ac.y = c.y - a.y;
		ac.z = c.z - a.z;

		shape abc = ab.cross(ac); // ABC = AB x AC  -> normal of the (triangle) surface

		shape x;  x.x = 1;  x.y = 0;  x.z = 0;
		shape y;  y.x = 0;  y.y = 1;  y.z = 0;
		shape z = x.cross(y);

		shape abc_c_ac = abc.cross(ac); // ABC x AC
		shape ab_c_abc = ab.cross(abc); // AB x ABC

		if (abc_c_ac.dot(a0) > 0)
		{
			if (ac.dot(a0) > 0)
			{
				return_sd = triple_cross_prod(ac, a0, ac);
			}
			else
			{
				if (ab.dot(a0) > 0)
				{
					return_sd = triple_cross_prod(ab, a0, ab);
				}
				else
				{
					return_sd = a0;
				}
			}
		}
		else
		{
			if (ab_c_abc.dot(a0) > 0)
			{
				if (ab.dot(a0) > 0)
				{
					return_sd = triple_cross_prod(ab, a0, ab);
				}
				else
				{
					return_sd = a0;
				}
			}
			else
			{
				if (abc.dot(a0) > 0)
				{
					return_sd = abc;
				}
				else
				{
					return_sd.x = -abc.x;
					return_sd.y = -abc.y;
					return_sd.z = -abc.z;
				}
			}
		}

		d.set(return_sd);
		return false;

	}
	else if (simplex.size() == 2) // 2 elements in the simplex
	{
		shape b = simplex.get(0); // get the first element
 
		shape ab; // b - a
		ab.x = b.x - a.x;
		ab.y = b.y - a.y;
		ab.z = b.z - a.z;

		shape triple_cp;

		if (ab.dot(a0) > 0)
		{
			triple_cp = triple_cross_prod(ab, a0, ab);
		}
		else
		{
			triple_cp.x = a0.x;
			triple_cp.y = a0.y;
			triple_cp.z = a0.z;
		}

		d.set(triple_cp);
	}

		return false;
}
// check inside simplex evolution (2, 3, 4 points inside the simplex list)

// GJK functions

//main GJK function
bool gjk(shape *A, shape *B, int dim_a, int dim_b) // main gjk function -> shape A, shape B, dimension of A(=amount of elements in the shape A)
{
// std::cout << "MUU" << std::endl;
	bool check_failsafe = true; // failsafe if one point A gets removed, 1 added, point B gets removed, 1 added, point A gets removed and so on .... ( stuck in an infinite loop)
	bool check = true;

	int i = 0;
	simplex simplex; // list of points of the simplex stored here

	while (check_failsafe == true)
	{
		shape mma = mass_middle_point(A, dim_a);

		shape mmb = mass_middle_point(B, dim_b);

		shape d; // pointing from mass middle point of shape A to mass middle point of shape B

		d.x = mma.x-mmb.x;
		d.y = mma.y-mmb.y;
		d.z = mma.z-mmb.z;

		if (i > 10)
		{
			i = 0;

			check_failsafe = true;
			check = true;

			int r1 = 0;
			int r2 = 0;

			int r3 = 0;
			int r4 = 0;

			int r5 = 0;
			int r6 = 0;

			while ( r1 == 0 or r2 == 0 or r3 == 0 or r4 == 0 or r5 == 0 or r6 == 0)
			{
				r1 = rand() % 19 + (-9);
				r2 = rand() % 19 + (-9);

				r3 = rand() % 19 + (-9);
				r4 = rand() % 19 + (-9);

				r5 = rand() % 19 + (-9);
				r6 = rand() % 19 + (-9);
			}

			float s1 = (float)r1/(float)r2;
			float s2 = (float)r3/(float)r4;
			float s3 = (float)r5/(float)r6;

			// random seeded new search direction
			d.x = s1;
			d.y = s2;
			d.z = s3;
			// random seeded new search direction

		}

		simplex.set_zero(); // set everything to zero


		simplex.add(support_function(A, B, dim_a, dim_b, d));

		d.negate();

		while (check == true)
		{

			simplex.add(support_function(A, B, dim_a, dim_b, d));

			if(simplex.size() == 4)
			{
				i++;
			}

			if (simplex.getLast().dot(d) < 0) // no collision -> break
			{
				check_failsafe = false;
				check = false; // break the while loop
			}
			else // there may be a collision -> continue
			{
				if (containsOrigin(simplex, d, i))
				{
					check = true;
					check_failsafe = false; // true collision
					break;

				}
				else if (i > 10)
				{
					check = false;
				}
			}
		}
	}

	return check;
}
//main GJK function
