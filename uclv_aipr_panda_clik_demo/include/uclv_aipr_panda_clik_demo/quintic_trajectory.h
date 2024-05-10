#pragma once

#include <math.h> /* pow */

/*
q(t) - Polinomio quintico con velocità e accelerazioni (iniziali e finali) nulle

Il polinomio è definito tra 0 e tf. E va da qi a qf.
Per t<=0 ---> q(t)=qi
Per t>=tf ---> q(t)=qf
*/
double qintic(double t, double qi, double qf, double tf)
{
  if (t <= 0.0)
  {
    return qi;
  }
  if (t >= tf)
  {
    return qf;
  }

  double tau = t / tf;
  double qhat = 6.0 * pow(tau, 5) - 15.0 * pow(tau, 4) + 10.0 * pow(tau, 3);

  return qi + (qf - qi) * qhat;
}
