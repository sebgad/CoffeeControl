#include "Arduino.h"
#include "PidCtrl.h"

PidCtrl::PidCtrl() {
    /**
     * 
     */
    _iSizeCoeffTbl = 0;
    _fSumIntegrator = 0;

};

void PidCtrl::begin(){
    /** Start PID controler. In- and Output of the regulator will be done external.
     * 
     */
    
    ptrActualValue = new float;
    ptrManipValue = new float;
    _iLastComputeMillis = millis();
    _iLinkMode = 1;
}

void PidCtrl::begin(float * ptr_actual_value){
    /*
    * Start PID controler. In- and Output of the regulator will be done external.
    */
    ptrActualValue = ptr_actual_value;
    ptrManipValue = new float;
    _iLastComputeMillis = millis();
    _iLinkMode = 2;
}

void PidCtrl::begin(float * ptr_actual_value, float * ptr_manip_value){
    /*
    * Start PID controler. In- and Output of the regulator will be done external.
    */
    ptrActualValue =  ptr_actual_value;
    ptrManipValue = ptr_manip_value;
    _iLastComputeMillis = millis();
    _iLinkMode = 3;
}


void PidCtrl::changePidCoeffs(float f_coeff_p, float f_coeff_i, float f_coeff_d){
    /*
    *
    */
    if (_iSizeCoeffTbl == 0) {
        _initCoeffTable(1);
    }

    ptrConstants[0][0] = f_coeff_p;
    ptrConstants[0][1] = f_coeff_i;
    ptrConstants[0][2] = f_coeff_d;
}


void PidCtrl::changePidCoeffs(const float arr_coeff_table[][4], size_t i_size_conv){
   /**
    *
    */

    if (_iSizeCoeffTbl == 0) {
        _initCoeffTable(i_size_conv);
    }

    // calculate gradient and offset and write it to array
    for(int i_row=0; i_row<i_size_conv; i_row++){
        for(int i_col; i_col<=4; i_col++){
            ptrConstants[i_row][i_col] = arr_coeff_table[i_row][i_col];
        }
    }
}

void PidCtrl::addOutputLimits(const float f_lower_lim, const float f_upper_lim){
    /**
     * lower and upper limit and data type (0=float, 1=integer)
     */

    _fLoLim = f_lower_lim;
    _fUpLim = f_upper_lim;
}

void PidCtrl::changeTargetValue(float f_target_value){
    /**
     * 
     */

    _fTargetValue = f_target_value;
}


void PidCtrl::compute() {
    /** Everything is linked
     * 
     */
    
    _calcControlEquation();
}


void PidCtrl::compute(const float & f_actual, float & f_manip){
    /**
     * Manip variable only linked 
     */    
    *ptrActualValue = f_actual;
    _calcControlEquation();
    f_manip = *ptrManipValue;
}


void PidCtrl::_calcControlEquation(){
    /**
     * 
     */

    float f_delta_sec;
    float f_control_deviation;
    float f_k_p_coeff, f_t_n_coeff, f_t_v_coeff;
    float f_manip_value;
    
    f_delta_sec = (float)(millis() - _iLastComputeMillis)/1000.0;
    
    f_control_deviation = _fTargetValue - *ptrActualValue;
    
    if (_iSizeCoeffTbl > 1) {
        for (int i_row=_iSizeCoeffTbl; i_row>=0; i_row--){
            if (ptrConstants[i_row][3] < *ptrActualValue){
                f_k_p_coeff = ptrConstants[i_row][0];
                f_t_n_coeff = ptrConstants[i_row][1];
                f_t_v_coeff = ptrConstants[i_row][2];
                break;
            }
        }
    } else {
        f_k_p_coeff = ptrConstants[0][0];
        f_t_n_coeff = ptrConstants[0][1];
        f_t_v_coeff = ptrConstants[0][2];
    };

    _fSumIntegrator += f_delta_sec * f_control_deviation;

    *ptrManipValue = f_k_p_coeff * (f_control_deviation + 1.0/f_t_n_coeff * _fSumIntegrator + f_t_v_coeff / f_delta_sec * f_control_deviation);

    // Check lower and upper limit of manipulation variable
    if (*ptrManipValue<_fLoLim) { *ptrManipValue = _fLoLim; };
    if (*ptrManipValue>_fUpLim) { *ptrManipValue = _fUpLim; };
    
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