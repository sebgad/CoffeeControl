/*
 * PID controle module, using the following control equation:
 * u(t) = Kp * [ e(t) + 1/Tn * $\int{0}^{t}e(\tau) d\tau$ + Tv / dT * e(t)]
 *  u(t):   manipulated variable (Stellgröße)
 *  e(t):   Deviation between actual value and target value (Regelabweichung)
 *  Kp:     P-Gain (P-Vestärkung)
 *  Tn:     reset time (Nachstellzeit)
 *  Tv:     lead time (Vorhaltzeit)
 */

#ifndef PidCtrl_h
#define PidCtrl_h

#include "Arduino.h"

class PidCtrl
{
  public:
    PidCtrl();
    void begin();
    void begin(float *);
    void begin(float *, float *);
    void changePidCoeffs(float, float, float);
    void changePidCoeffs(const float[][4], size_t);
    void changeTargetValue(float);
    void compute();
    void compute(const float &, float &);
    void addOutputLimits(const float, const float);

  private:
    float ** ptrConstants;
    float * ptrActualValue;
    float * ptrManipValue;
    float _fTargetValue;
    float _fLoLim;
    float _fUpLim;
    int _iLinkMode;
    float _fSumIntegrator;
    unsigned long _iLastComputeMillis;
    size_t _iSizeCoeffTbl;
    void _calcControlEquation();
    void _initCoeffTable(size_t);
};

#endif