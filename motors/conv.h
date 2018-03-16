/*
 * conv.h
 *
 *  Created on: 20 ñ³÷. 2017
 *      Author: user
 */

#include "math.h"

#ifndef CONV_H_
#define CONV_H_

#define SQRT3 1.73205080
#define ONE_OVER_SQRT3 0.57735026
#define ONE_OVER_3 0.33333333

typedef struct Vect3_{
	float data[3];
}Vect3;

typedef struct Vect2_{
	float data[2];
}Vect2;

Vect2 clarke(Vect3 inV)
{
	Vect2 outV;
//	outV.data[0] = inV.data[0];
//	outV.data[1] = (inV.data[1]-inV.data[2])*ONE_OVER_SQRT3;
	outV.data[0] = (inV.data[0]*2. - (inV.data[1]-inV.data[2]))*ONE_OVER_3;
	outV.data[1] = (inV.data[1]-inV.data[2])*ONE_OVER_SQRT3;
	return outV;
}

Vect2 park(Vect2 inV, float angle)
{
	Vect2 outV;
	float siA = sin(angle);
	float coA = cos(angle);
	outV.data[0] = inV.data[1]*siA + inV.data[0]*coA;
	outV.data[1] = inV.data[1]*coA - inV.data[0]*siA;
	return outV;
}

Vect2 parkInv(Vect2 inV, float angle)
{
	Vect2 outV;
	float siA = sin(angle);
	float coA = cos(angle);
	outV.data[0] = inV.data[0]*coA - inV.data[1]*siA;
	outV.data[1] = inV.data[0]*siA + inV.data[1]*coA;
	return outV;
}

Vect3 clarkeInv(Vect2 inV)
{
	Vect3 outV;
	float Vmax,Vmin,Vcom;
	float Va,Vb,Vc;

	Va = inV.data[0];
	Vb = SQRT3*0.5*inV.data[1]-0.5*inV.data[0];
	Vc = -SQRT3*0.5*inV.data[1]-0.5*inV.data[0];

	Vmax=0;
	Vmin=0;

	// find order Vmin,Vmid,Vmax
	if (Va > Vb)
	{
		Vmax = Va;
		Vmin = Vb;
	}
	else
	{
		Vmax = Vb;
		Vmin = Va;
	}

	if (Vc > Vmax)
	{
		Vmax = Vc;
	}
	else if (Vc < Vmin)
	{
		Vmin = Vc;
	}

	Vcom = (Vmax+Vmin)*0.5;  //0.5*(Vmax+Vmin)

	// Subtract common-mode term to achieve SV modulation
	outV.data[0] = ((Vcom - Va));
	outV.data[1] = ((Vcom - Vb));
	outV.data[2] = ((Vcom - Vc));

//	if (outV.data[0] > outV.data[1])
//		Vmax = outV.data[0];
//	else
//		Vmax = outV.data[1];
//	if (outV.data[2] > Vmax)
//		Vmax = outV.data[2];
	outV.data[0] = (outV.data[0]+1.1)*0.5;
	outV.data[1] = (outV.data[1]+1.1)*0.5;
	outV.data[2] = (outV.data[2]+1.1)*0.5;



	return outV;
}




#endif /* CONV_H_ */
