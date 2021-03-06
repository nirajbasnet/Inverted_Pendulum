#include"PIDgeneral.h"


void PID::PIDinitialize()
{
	Sampletime_milli = 1000;
	Samplefrequency=1;
	Proportional=0;
	Integral=0;
	Derivative=0;
	error=0;
	lasterror=0;
	totalerror=0;
	input=0;
	lastinput=0;
	controlDirection=TRUE;
	offset=150;
}

void PID::SetOutputLimits(int Min, int Max)
{
	if(Min >= Max) return;
	outMin = Min;
	outMax = Max;
}

void PID::SetIntegralLimits(char check,int Min, int Max)
{
	limitintegral=check;
	if(Min >= Max) return;
	Integralmin = Min;
	Integralmax = Max;
}
void PID::SetSamplefrequency(unsigned int  NewSamplefrequency)
{
	if (NewSamplefrequency > 0)
	{

		float ratio = (float)Samplefrequency / (float)NewSamplefrequency;
		ki *= ratio;
		kd /= ratio;
		Samplefrequency = NewSamplefrequency;
	}
}
void PID::SetOffset(unsigned int value)
{
	offset=value;
}

void PID::SetTargetPoint(float target,char direction)
{
	Setpoint = target;
	if(direction !=controlDirection)
	{
		kp = (0 - kp);
		ki = (0 - ki);
		kd = (0 - kd);
	}
	controlDirection = direction;
}

void PID::SetTuningConstants(float p, float i, float d)
{
	if (p<0 || i<0 || d<0) return;

	//float SampleTimeInSec = ((float)Sampletime_milli/1000);
	float SampleTimeInSec = 1/(float)Samplefrequency;
	kp = p;
	ki = i * SampleTimeInSec;
	kd = d / SampleTimeInSec;

	if(controlDirection == FALSE)         //false for reverse direction and true for direct or positive case
	{
		kp = (0 - kp);
		ki = (0 - ki);
		kd = (0 - kd);
		
	}
}
void PID::SetInput(float x)
{
	input=x;
}
void PID::CalculatePID()
{
	error = Setpoint - input;
	Proportional= kp*pow(error,Prop_power);
	Integral += (ki *error);
	if(limitintegral==TRUE)
	{
		constrain(&Integral,Integralmin,Integralmax);
	}
	//Derivative= kd*(error-lasterror);        //Test both of these
	Derivative= -kd*(input-lastinput);
	totalerror= Proportional+Integral+Derivative;
	constrain(&totalerror,outMin,outMax);

	//lasterror= error;
	lastinput=input;


}


float PID::getPIDoutput()
{
	if(totalerror>0)
	return (totalerror+offset);
	else if(totalerror<0)
	return (totalerror-offset);
}


void PID::constrain(float *value,int minvalue,int maxvalue)
{
	if(*value>maxvalue)
	{
		*value=maxvalue;
	}
	if(*value<minvalue)
	{
		*value=minvalue;
	}
}
