!KGEN-generated Fortran source file 
  
!Generated at : 2026-03-24 00:48:09 
!KGEN version : 0.8.1 
  
! Copyright 2019 NREL
! Licensed under the Apache License, Version 2.0 (the "License"); you may not use
! this file except in compliance with the License. You may obtain a copy of the
! License at http://www.apache.org/licenses/LICENSE-2.0
! Unless required by applicable law or agreed to in writing, software distributed
! under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
! CONDITIONS OF ANY KIND, either express or implied. See the License for the
! specific language governing permissions and limitations under the License.
! -------------------------------------------------------------------------------------------
! This module contains all the filters and related subroutines
! Filters:
!       LPFilter: Low-pass filter
!       SecLPFilter: Second order low-pass filter
!       HPFilter: High-pass filter
!       NotchFilter: Notch filter
!       NotchFilterSlopes: Notch Filter with descending slopes
!       PreFilterMeasuredSignals: Pre-filter signals during each run iteration


MODULE Filters
!...............................................................................................................................
    USE functions 
    USE kgen_utils_mod, ONLY: kgen_dp, kgen_array_sumcheck 
    USE tprof_mod, ONLY: tstart, tstop, tnull, tprnt 
    IMPLICIT NONE 

CONTAINS
!-------------------------------------------------------------------------------------------------------------------------------
    REAL(DbKi) FUNCTION LPFilter(InputSignal, DT, CornerFreq, FP, iStatus, reset, inst, InitialValue)
    ! Discrete time Low-Pass Filter of the form:
    !                               Continuous Time Form:   H(s) = CornerFreq/(1 + CornerFreq)
    !                               Discrete Time Form:     H(z) = (b1z + b0) / (a1*z + a0)
    !
        USE rosco_types, ONLY: filterparameters 
        USE rosco_types, ONLY: kr_rosco_types_filterparameters 
        USE rosco_types, ONLY: kv_rosco_types_filterparameters 
        TYPE(FilterParameters),       INTENT(INOUT)       :: FP 

        REAL(DbKi), INTENT(IN)         :: InputSignal
        REAL(DbKi), INTENT(IN)         :: DT                       ! time step [s]
        REAL(DbKi), INTENT(IN)         :: CornerFreq               ! corner frequency [rad/s]
        INTEGER(IntKi), INTENT(IN)      :: iStatus                  ! A status flag set by the simulation as follows: 0 if this is the first call, 1 for all subsequent time steps, -1 if this is the final call at the end of the simulation.
        INTEGER(IntKi), INTENT(INOUT)   :: inst                     ! Instance number. Every instance of this function needs to have an unique instance number to ensure instances don't influence each other.
        LOGICAL(4), INTENT(IN)      :: reset                    ! Reset the filter to the input signal
        REAL(DbKi), OPTIONAL,  INTENT(IN)          :: InitialValue           ! Value to set when reset 
        
        REAL(DbKi)                          :: InitialValue_           ! Value to set when reset
        ! Defaults

        InitialValue_ = InputSignal
        IF (PRESENT(InitialValue)) InitialValue_ = InitialValue  
        ! Initialization

        IF ((iStatus == 0) .OR. reset) THEN   
            FP%lpf1_OutputSignalLast(inst) = InitialValue_
            FP%lpf1_InputSignalLast(inst) = InitialValue_
            FP%lpf1_a1(inst) = 2 + CornerFreq*DT
            FP%lpf1_a0(inst) = CornerFreq*DT - 2
            FP%lpf1_b1(inst) = CornerFreq*DT
            FP%lpf1_b0(inst) = CornerFreq*DT
        ENDIF
        ! Define coefficients
        ! Filter


        LPFilter = 1.0/FP%lpf1_a1(inst) * (-FP%lpf1_a0(inst)*FP%lpf1_OutputSignalLast(inst) + FP%lpf1_b1(inst)*InputSignal + FP%lpf1_b0(inst)*FP%lpf1_InputSignalLast(inst))
        ! Save signals for next time step

        FP%lpf1_InputSignalLast(inst)  = InputSignal
        FP%lpf1_OutputSignalLast(inst) = LPFilter
        inst = inst + 1

    END FUNCTION LPFilter
!-------------------------------------------------------------------------------------------------------------------------------
    REAL(DbKi) FUNCTION SecLPFilter(InputSignal, DT, CornerFreq, Damp, FP, iStatus, reset, inst, InitialValue)
    ! Discrete time Low-Pass Filter of the form:
    !                               Continuous Time Form:   H(s) = CornerFreq^2/(s^2 + 2*CornerFreq*Damp*s + CornerFreq^2)
    !                               Discrete Time From:     H(z) = (b2*z^2 + b1*z + b0) / (a2*z^2 + a1*z + a0)
        USE rosco_types, ONLY: filterparameters 
        USE rosco_types, ONLY: kr_rosco_types_filterparameters 
        USE rosco_types, ONLY: kv_rosco_types_filterparameters 
        TYPE(FilterParameters),       INTENT(INOUT)       :: FP 
        REAL(DbKi), INTENT(IN)         :: InputSignal
        REAL(DbKi), INTENT(IN)         :: DT                       ! time step [s]
        REAL(DbKi), INTENT(IN)         :: CornerFreq               ! corner frequency [rad/s]
        REAL(DbKi), INTENT(IN)         :: Damp                     ! Dampening constant
        INTEGER(IntKi), INTENT(IN)      :: iStatus                  ! A status flag set by the simulation as follows: 0 if this is the first call, 1 for all subsequent time steps, -1 if this is the final call at the end of the simulation.
        INTEGER(IntKi), INTENT(INOUT)   :: inst                     ! Instance number. Every instance of this function needs to have an unique instance number to ensure instances don't influence each other.
        LOGICAL(4), INTENT(IN)          :: reset                    ! Reset the filter to the input signal
        REAL(DbKi), OPTIONAL,  INTENT(IN)          :: InitialValue           ! Value to set when reset 
        
        REAL(DbKi)                          :: InitialValue_           ! Value to set when reset
        ! Defaults

        InitialValue_ = InputSignal
        IF (PRESENT(InitialValue)) InitialValue_ = InitialValue  
        ! Initialization

        IF ((iStatus == 0) .OR. reset )  THEN
            FP%lpf2_OutputSignalLast1(inst)  = InitialValue_
            FP%lpf2_OutputSignalLast2(inst)  = InitialValue_
            FP%lpf2_InputSignalLast1(inst)   = InitialValue_
            FP%lpf2_InputSignalLast2(inst)   = InitialValue_
            ! Coefficients
            
            FP%lpf2_a2(inst) = DT**2.0*CornerFreq**2.0 + 4.0 + 4.0*Damp*CornerFreq*DT
            FP%lpf2_a1(inst) = 2.0*DT**2.0*CornerFreq**2.0 - 8.0
            FP%lpf2_a0(inst) = DT**2.0*CornerFreq**2.0 + 4.0 - 4.0*Damp*CornerFreq*DT
            FP%lpf2_b2(inst) = DT**2.0*CornerFreq**2.0
            FP%lpf2_b1(inst) = 2.0*DT**2.0*CornerFreq**2.0
            FP%lpf2_b0(inst) = DT**2.0*CornerFreq**2.0
        ENDIF
        ! Filter

        SecLPFilter = 1.0/FP%lpf2_a2(inst) * (FP%lpf2_b2(inst)*InputSignal + FP%lpf2_b1(inst)*FP%lpf2_InputSignalLast1(inst) + FP%lpf2_b0(inst)*FP%lpf2_InputSignalLast2(inst) - FP%lpf2_a1(inst)*FP%lpf2_OutputSignalLast1(inst) - FP%lpf2_a0(inst)*FP%lpf2_OutputSignalLast2(inst))
        ! Save signals for next time step

        FP%lpf2_InputSignalLast2(inst)   = FP%lpf2_InputSignalLast1(inst)
        FP%lpf2_InputSignalLast1(inst)   = InputSignal
        FP%lpf2_OutputSignalLast2(inst)  = FP%lpf2_OutputSignalLast1(inst)
        FP%lpf2_OutputSignalLast1(inst)  = SecLPFilter

        inst = inst + 1

    END FUNCTION SecLPFilter
!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------


    END MODULE Filters