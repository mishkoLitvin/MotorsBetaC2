/*
 * linearFit.h
 *
 *  Created on: 25 זמגע. 2016
 *      Author: Mishko Lytvyn
 */

#ifndef LINEARFIT_H_
#define LINEARFIT_H_
struct line{
	float x[100];
	float y[100];
	int i;
	float s1, s2, s3, s4;
	float fit()
	{
		for(i=0;i<100;i++)
		{
			s1+=x[i]*y[i];
			s2+=x[i];
			s3+=y[i];
			s4+=x[i]*x[i];
		}
		return (100*s1-s2*s3)/(100*s4-s2*s2);

	}

}fiter;


#endif /* LINEARFIT_H_ */
