class pid
{
    public
	float kp;
	float ki;
	float kd;
	float error;
	float setpoint;
	float input;
	float output;
	float Iterm;
	float outMin;
	float outMax;

	void set_limits(float a,float b);
	float compute(float in);
};
/******************************************************************************/

/******************************************************************************/
void pid::set_limits(float a,float b)
{
	if(a<b)
	{
		outMin = a;
		outMax = b;
	}
	else
	{
		outMin = b;
		outMax = a;
	}
}
/******************************************************************************/

/******************************************************************************/
float pid::compute(float in)
{
	input = in;
	error = setpoint - input;
	Iterm += (ki*error);
	if(Iterm> outMax) Iterm= outMax;
      else if(Iterm< outMin) Iterm= outMin;
      output = kp * error + Iterm;
    if(Output > outMax) Output = outMax;
    else if(Output < outMin) Output = outMin;
}
