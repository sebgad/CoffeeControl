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
    
    ptrActualValue = new float;
    ptrManipValue = new float;
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
    ptrActualValue = ptr_actual_value;
    ptrManipValue = new float;
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
    ptrActualValue =  ptr_actual_value;
    ptrManipValue = ptr_manip_value;
    _iLastComputeMillis = millis();
    _iLinkMode = 3;
}


void PidCtrl::activate(bool b_kp_activate, bool b_ti_activate, bool b_td_activate){
    /**
     * @brief Define which Parameters should be activated
     * 
     */

    bKpActivate = b_kp_activate;
    bTiActivate = b_ti_activate;
    bTdActivate = b_td_activate;
}

void PidCtrl::changePidCoeffs(float f_coeff_k_p, float f_coeff_t_i, float f_coeff_t_d){
    /**
     * Change PID coefficients for PID controler.
     * @param f_coeff_k_p       Proportional part of the control equitation
     * @param f_coeff_t_i       Reset time (Nachstellzeit)
     * @param f_coeff_t_d       Lead time (Vorstellzeit)
     */

    if (_iSizeCoeffTbl == 0) {
        // Only one dimensional PID controller is initialized
        _initCoeffTable(1);
    }

    ptrConstants[0][0] = f_coeff_k_p; // Kp
    if (f_coeff_k_p > 0.0) {  
        ptrConstants[0][1] = f_coeff_t_i; // Ti
        ptrConstants[0][2] = f_coeff_t_d; // Td
    }
    else{
        // really stupid choice of controller parameters... but now everything is really disabled
        ptrConstants[0][1] = 0.0;
        ptrConstants[0][2] = 0.0;
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

void PidCtrl::addOutputLimits(const float f_lower_lim, const float f_upper_lim){
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

    *ptrActualValue = f_actual;
    _calcControlEquation();
    f_manip = *ptrManipValue;
}


void PidCtrl::_calcControlEquation(){
    /**
     * Calculate control equitation.
     */

    float f_delta_sec;
    float f_control_deviation;
    float f_d_control_deviation; // deviation of actual value TODO needed?
    float f_k_p_coeff, f_t_i_coeff, f_t_d_coeff;
    
    f_delta_sec = (float)(millis() - _iLastComputeMillis)/1000.0;
    f_control_deviation = _fTargetValue - *ptrActualValue; // error
    
    if (_iSizeCoeffTbl > 1) {
        // Regulator with different coefficients, depending on actual variable value
        for (int i_row=_iSizeCoeffTbl; i_row>=0; i_row--){
            // TODO dont use actual value as parameter selector -> use different pointer (can be actual value or anything else)
            if (ptrConstants[i_row][3] < *ptrActualValue){
                f_k_p_coeff = ptrConstants[i_row][0];
                f_t_i_coeff = ptrConstants[i_row][1];
                f_t_d_coeff = ptrConstants[i_row][2];
                break;
            }
        }
    } else {
        // One coefficient set for all areas
        f_k_p_coeff = ptrConstants[0][0];
        f_t_i_coeff = ptrConstants[0][1];
        f_t_d_coeff = ptrConstants[0][2];
    };

    *ptrManipValue = 0.0;
    // setValue = Kp* (error + 1/Ti * integal(error) * dt + Td * diff(error)/dt)
    if (bKpActivate){
        // Proportional component of the controler
        *ptrManipValue += f_k_p_coeff * f_control_deviation; 

        if (bTiActivate){
            // Integral part of the controler
            _fSumIntegrator += f_delta_sec * f_control_deviation; // integrate deviation over time
            *ptrManipValue += f_k_p_coeff / f_t_i_coeff * _fSumIntegrator;
        }

        if (bTdActivate){
            // Differential component of the controler
            f_d_control_deviation = (f_control_deviation - _fLastControlDev); // deviation of error
            *ptrManipValue += f_k_p_coeff * f_t_d_coeff * f_d_control_deviation / f_delta_sec;
        }
    }

    // Check lower and upper limit of manipulation variable
    if (*ptrManipValue<_fLoLim) { *ptrManipValue = _fLoLim; };
    if (*ptrManipValue>_fUpLim) { *ptrManipValue = _fUpLim; };

    // TODO check and react for ONOFFThreshold
    
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