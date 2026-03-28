!KGEN-generated Fortran source file 
  
!Generated at : 2026-03-28 07:59:49 
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


    INTEGER(IntKi), PARAMETER     :: VS_Mode_WSE_TSR    = 2
    INTEGER(IntKi), PARAMETER     :: VS_Mode_Power_TSR  = 3
    INTEGER(IntKi), PARAMETER     :: VS_Mode_Torque_TSR = 4
    ! VS_ConstPower

    ! VS_FBP

    INTEGER(IntKi), PARAMETER     :: VS_FBP_Variable_Pitch  = 0
    INTEGER(IntKi), PARAMETER     :: VS_FBP_WSE_Ref         = 2
    INTEGER(IntKi), PARAMETER     :: VS_FBP_Torque_Ref      = 3
    ! VS_State

    ! PC_State

    ! PRC Mode constants
    


END MODULE Constants