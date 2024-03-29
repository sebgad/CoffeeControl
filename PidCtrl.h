/*
 * PID controle module, using the following control equation:
 * u(t) = Kp * [ e(t) + 1/Tn * $\int{0}^{t}e(\tau) d\tau$ + Tv / dT * e(t)]
 *  u(t):   manipulated variable (Stellgröße)
 *  e(t):   Deviation between actual value and target value (Regelabweichung)
 *  Kp:     P-Gain (P-Vestärkung)
 *  Tn:     reset time (Nachstellzeit) Tn = Kp/Ki
 *  Tv:     lead time (Vorhaltzeit) Tv= Kd/Kp
 *
 * THem constants are stored in the memeber ptrConstants as Kp; 1/Tn; Tv
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
    void changePidCoeffs(float, float, float, bool);
    void changePidCoeffs(const float[][4], size_t);
    void activate(bool, bool, bool);
    void changeTargetValue(float);
    void setOnOffThres(float f_tresh_on, float f_tresh_off);
    void setOnOffThres(float f_tresh);
    void setOnThres(float f_tresh);
    void setOffThres(float f_tresh);
    void compute();
    void compute(const float &, float &);
    void addOutputLimits(float, float);
    float getErrorIntegrator();
    float getErrorDiff();
    float getTargetValue();

  private:
    float ** ptrConstants;
    bool _bKpActivate;
    bool _bKiActivate;
    bool _bKdActivate;
    float * _ptrActualValue;
    float * _ptrManipValue;
    float _fTargetValue;
    float _fSumIntegrator;
    float _fErrDiff;
    float _fLoLim;
    float _fUpLim;
    float _fThresOn;
    float _fThresOff;
    bool _bThresOn = false;
    bool _bThresOff = false;
    int _iLinkMode;
    unsigned long _iLastComputeMillis;
    float _fLastControlDev;
    size_t _iSizeCoeffTbl;
    void _calcControlEquation();
    void _initCoeffTable(size_t);
};

#endif