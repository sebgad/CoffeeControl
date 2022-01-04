#include "Arduino.h"
#include "PidCtrl.h"

PidCtrl::PidCtrl() {
    /** Initialisation of PIDCtrl class
     * 
     */
    _iSizeCoeffTbl = 0;
    _fSumIntegrator = 0;

}

void PidCtrl::begin(){
    /** 
     * Start PID controler.
     * Actual value variable and manipulation value variable is initialized and not linked. 
     * Compute() method takes actual value and manipulation value as parameter.
     */
    
    _ptrActualValue = new float;
    _ptrManipValue = new float;
    _iLastComputeMillis = millis();
    _iLinkMode = 1;
}

void PidCtrl::begin(float * ptr_actual_value){
    /** 
     * Start PID controler.
     * Actual value variable is linked, manipulation value variable is initialized and not linked. 
     * Compute() method takes the manipulation variale as parameter.
     * @param ptr_actual_value     Actual value or measurement value as pointer
    */
    _ptrActualValue = ptr_actual_value;
    _ptrManipValue = new float;
    _iLastComputeMillis = millis();
    _iLinkMode = 2;
}

void PidCtrl::begin(float * ptr_actual_value, float * ptr_manip_value){
    /**
     * Start PID controler. 
     * Actual value variable and manipulation value variable is linked. Compute() method will directly change the output.
     * @param ptr_actual_value       Actual value or measurement as pointer
     * @param ptr_manip_value        Manipulation value as pointer
    */
    _ptrActualValue =  ptr_actual_value;
    _ptrManipValue = ptr_manip_value;
    _iLastComputeMillis = millis();
    _iLinkMode = 3;
}


void PidCtrl::activate(bool b_kp_activate, bool b_ki_activate, bool b_kd_activate){
    /**
     * @brief Define which Parameters should be activated
     * 
     */

    _bKpActivate = b_kp_activate;
    _bKiActivate = b_ki_activate;
    _bKdActivate = b_kd_activate;
}

void PidCtrl::changePidCoeffs(float f_coeff_prop, float f_coeff_int, float f_coeff_dif, bool b_time_coeff){
    /**
     * Change PID coefficients for PID controler.
     * @param f_coeff_prop      Proportional part of the control equitation
     * @param f_coeff_int       Integration coefficient can be Ki or Tn
     * @param f_coeff_dif       Differential coefficient can be Kd or Tv
     * @param b_time_coeff      Int and diff coefficient are based on time (reset time and lead time)
     */

    if (_iSizeCoeffTbl == 0) {
        // Only one dimensional PID controller is initialized
        _initCoeffTable(1);
    }

    ptrConstants[0][0] = f_coeff_prop;

    if (b_time_coeff){
        // coefficients are based on time and related to proportional part
        
        // factor for integration part
        if ((f_coeff_int>0.0) || (f_coeff_int<0.0)){
            ptrConstants[0][1] = f_coeff_prop / f_coeff_int; // Kp/Tn = 1/Ti = Ki
        } else {
            ptrConstants[0][1] = 0.0;
        }

        // factor for differential part
        ptrConstants[0][2] = f_coeff_prop * f_coeff_dif; // Kp*Tv = Td = Kd
    } else {
        // coefficients not related to time and not depending on proportional part
        // factor for integration part
        ptrConstants[0][1] = f_coeff_int; 
        // factor for differential part
        ptrConstants[0][2] = f_coeff_dif;
    }
}


void PidCtrl::changePidCoeffs(const float arr_coeff_table[][4], size_t i_size_conv){
   /**
    * Change PID coefficients using a characteristics
    * @param arr_coeff_table    1st column: Kp, 2nd column: Ki, 3rd column: Kd, 4th column: lower limit measuring value
    * @param i_size_conv        Size of arr_coeff_table
    */

    if (_iSizeCoeffTbl == 0) {
        // initialize array, if not already initialized
        _initCoeffTable(i_size_conv);
    }

    // calculate gradient and offset and write it to array
    for(int i_row=0; i_row<i_size_conv; i_row++){
        // specian handling for Kp=0 divide by 0
        ptrConstants[i_row][0] = arr_coeff_table[i_row][0]; // Kp
        if (arr_coeff_table[i_row][0] != 0) {
            ptrConstants[i_row][1] = arr_coeff_table[i_row][1]/arr_coeff_table[i_row][0];
            ptrConstants[i_row][2] = arr_coeff_table[i_row][2]/arr_coeff_table[i_row][0];
        }
        else{
        // really stupid choice of controller parameters... but now everything is really disabled
            ptrConstants[i_row][1] = 0;
            ptrConstants[i_row][2] = 0;
        }
        ptrConstants[i_row][3] = arr_coeff_table[i_row][3]; // lower limit measuring value
    }
}

void PidCtrl::addOutputLimits(float f_lower_lim, float f_upper_lim){
    /**
     * lower and upper limit and data type (0=float, 1=integer)
     * TODO param
     */

    _fLoLim = f_lower_lim;
    _fUpLim = f_upper_lim;
}

void PidCtrl::changeTargetValue(float f_target_value){
    /** TODO
     * 
     */

    _fTargetValue = f_target_value;
}

void PidCtrl::setOnOffThres(float f_thres_on, float f_thres_off) {
    /**
    * Set a threshold were the PID output is hard set to on when the actual value is lower f_tresh_on 
    * or hard set off when the actual value is higher f_tresh_off
    * @param f_thres_on     lower threshold to be used
    * @param f_thres_off    upper threshold to be used
    */
  _fThresOn = f_thres_on;
  _fThresOff = f_thres_off;
  _bThresOn = true;
  _bThresOff = true;
}//void PidCtrl::setOnOffThres


void PidCtrl::setOnOffThres(float f_thres) {
    /**
    * Set a threshold were the PID output is hard set to on when the actual value is lower -f_tresh 
    * or hard set off when the actual value is higher f_tresh
    * @param f_thres    threshold to be used
    */
  setOnOffThres(f_thres, -f_thres);
}//void PidCtrl::setOnOffThresh

void PidCtrl::setOnThres(float f_thres) {
    /**
    * Set a threshold were the PID output is hard set to on when the actual value is lower f_tresh 
    * @param f_thres    threshold to be used
    */
  _fThresOn = f_thres;
  _bThresOn = true;
}//void PidCtrl::setOnThresh

void PidCtrl::setOffThres(float f_tresh) {
    /**
    * Set a threshold were the PID output is hard set to off when the actual value is higher f_tresh 
    * @param f_tresh    threshold to be used
    */
  _fThresOff = f_tresh;
  _bThresOff = true; 
}//void PidCtrl::setOffThresh


void PidCtrl::compute() {
    /** 
     * Compute and calculate controler equitation. Actual value variable and manipulation value variable have to 
     * be linked in advance.
     */
    
    _calcControlEquation();
}

void PidCtrl::compute(const float & f_actual, float & f_manip){
    /**
     * Compute and calculate controler equitation. Actual value variable and manipulation value variable are NOT
     * linked in advance.
     * @param f_actual  Actual value variable
     * @param f_manip   Manipulation value variable
     */    

    *_ptrActualValue = f_actual;
    _calcControlEquation();
    f_manip = *_ptrManipValue;
}


void PidCtrl::_calcControlEquation(){
    /**
     * Calculate control equitation.
     */

    float f_delta_sec;
    float f_control_deviation;
    float f_d_control_deviation;
    float f_k_p_coeff, f_k_i_coeff, f_k_d_coeff;
    
    f_delta_sec = (float)(millis() - _iLastComputeMillis)/1000.0;
    f_control_deviation = _fTargetValue - *_ptrActualValue; // error
    
    if (_iSizeCoeffTbl > 1) {
        // Regulator with different coefficients, depending on actual variable value
        for (int i_row=_iSizeCoeffTbl; i_row>=0; i_row--){
            // TODO dont use actual value as parameter selector -> use different pointer (can be actual value or anything else)
            if (ptrConstants[i_row][3] < *_ptrActualValue){
                f_k_p_coeff = ptrConstants[i_row][0];
                f_k_i_coeff = ptrConstants[i_row][1];
                f_k_d_coeff = ptrConstants[i_row][2];
                break;
            }
        }
    } else {
        // One coefficient set for all areas
        f_k_p_coeff = ptrConstants[0][0];
        f_k_i_coeff = ptrConstants[0][1];
        f_k_d_coeff = ptrConstants[0][2];
    };

    *_ptrManipValue = 0.0;
    // setValue = Kp* (error + 1/Ti * integal(error) * dt + Td * diff(error)/dt)
    if (_bKpActivate){
        // Proportional component of the controler
        *_ptrManipValue += f_k_p_coeff * f_control_deviation;


        if (_bKdActivate){
            // Differential component of the controler
            _fErrDiff = (f_control_deviation - _fLastControlDev); // deviation of error
            *_ptrManipValue += f_k_d_coeff * _fErrDiff / f_delta_sec;
        }
    
        if (_bKiActivate){
            // Integral part of the controler
            f_d_control_deviation = f_delta_sec * f_control_deviation;
            *_ptrManipValue += f_k_i_coeff * (_fSumIntegrator + f_d_control_deviation);
            
            if ((*_ptrManipValue<_fUpLim) && (_fSumIntegrator+f_d_control_deviation>=0.0)){
                // Antiwindup measure: only sum error if manipulation value is lower then upper limit and error sum is greater than 0 (no cooling possible)
                _fSumIntegrator += f_d_control_deviation; // integrate deviation over time
            }
        }
    }

    // Check lower and upper limit of manipulation variable
    if (*_ptrManipValue<_fLoLim) { *_ptrManipValue = _fLoLim; };
    if (*_ptrManipValue>_fUpLim) { *_ptrManipValue = _fUpLim; };

    // check and react for ONOFFThreshold
    if (_bThresOn &&  (*_ptrActualValue < _fThresOn)) { *_ptrManipValue = _fUpLim; }; // below lower thres -> set to upper limit
    if (_bThresOff &&  (*_ptrActualValue > _fThresOff)) { *_ptrManipValue = _fLoLim; }; // below lower thres -> set to upper limit
    
    _fLastControlDev = f_control_deviation;
    _iLastComputeMillis = millis();

}

void PidCtrl::_initCoeffTable(size_t i_size_conv) {
  /**
   * Initialize pointer for conversion table
   * @param i_size_conv: row of the conversion table
  */
  
  // Make (row) size of conversion table in class available
  _iSizeCoeffTbl=i_size_conv;
  // assign memory to the pointer, pointer in pointer element
  ptrConstants = new float*[_iSizeCoeffTbl];
  
  // assign second pointer in pointer to get a 2dim field
  for(int i_row=0;i_row<_iSizeCoeffTbl;i_row++) {
    ptrConstants[i_row]=new float[4];
  }
}


float PidCtrl::getErrorIntegrator(){
  /**
   * @brief Get the sum error of the integrator part of the controller
  */
  return _fSumIntegrator;
}


float PidCtrl::getTargetValue(){
  /**
   * @brief Get the target value of the controller
   */ 
  return _fTargetValue;
}

float PidCtrl::getErrorDiff(){
  /**
   * @brief Get the error difference between recent deviation and last deviation
   */ 
  return _fErrDiff;
}
