/*
 * Q - liters/ha
 * flux - Flux in each tip;
 * V - Working Speed;
 * L - width of equipament (meters);
 *  flux = (Q x V x L)/600; get by jacto documents
 */

float Calculate(float Q, float V, float L)
{
    return (Q * V * L) / 600;
}

float MaxSpeedVal(float Q, float L)
{
	return 12000/(Q*L);
}

float mapFloat(float x, float a, float b, float c, float d)
{
    float f = x / (b - a) * (d - c) + c;
    return f;
}

int convert2SetPoint(float LPM)
{
    int sP = mapFloat(LPM, 0, 20, 0, 1023);
    return sP;
}



