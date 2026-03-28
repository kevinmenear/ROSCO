!KGEN-generated Fortran source file 
  
!Generated at : 2026-03-28 07:58:05 
!KGEN version : 0.8.1 
  
! Copyright 2019 NREL
! Licensed under the Apache License, Version 2.0 (the "License"); you may not use
! this file except in compliance with the License. You may obtain a copy of the
! License at http://www.apache.org/licenses/LICENSE-2.0
! Unless required by applicable law or agreed to in writing, software distributed
! under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
! CONDITIONS OF ANY KIND, either express or implied. See the License for the
! specific language governing permissions and limitations under the License.


MODULE Constants
    USE, INTRINSIC :: iso_c_binding 
    USE kgen_utils_mod
    USE tprof_mod, ONLY: tstart, tstop, tnull, tprnt 
    
    INTEGER, PARAMETER                  :: DbKi             = C_DOUBLE            !< Default kind for double floating-point numbers
    INTEGER, PARAMETER                  :: IntKi            = C_INT               !< Default kind for integer numbers
    
    ! NWTC Constants


    ! Control Modes
    ! VS_ControlMode


    ! VS_ConstPower

    INTEGER(IntKi), PARAMETER     :: VS_Mode_ConstPwr = 1
    ! VS_FBP

    INTEGER(IntKi), PARAMETER     :: VS_FBP_Variable_Pitch  = 0
    ! VS_State

    INTEGER(IntKi), PARAMETER     :: VS_State_Error             = 0
    INTEGER(IntKi), PARAMETER     :: VS_State_Region_1_5        = 1
    INTEGER(IntKi), PARAMETER     :: VS_State_Region_2          = 2
    INTEGER(IntKi), PARAMETER     :: VS_State_Region_2_5        = 3
    INTEGER(IntKi), PARAMETER     :: VS_State_Region_3_ConstTrq = 4
    INTEGER(IntKi), PARAMETER     :: VS_State_Region_3_ConstPwr = 5
    INTEGER(IntKi), PARAMETER     :: VS_State_Region_3_FBP      = 6
    ! PC_State

    INTEGER(IntKi), PARAMETER     :: PC_State_Disabled = 0
    INTEGER(IntKi), PARAMETER     :: PC_State_Enabled  = 1
    ! PRC Mode constants
    


END MODULE Constants