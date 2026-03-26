!KGEN-generated Fortran source file 
  
!Generated at : 2026-03-25 23:31:20 
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
    
    REAL(DbKi), PARAMETER               :: PI               = 3.14159265359       ! Mathematical constant pi
    INTEGER(IntKi), PARAMETER           :: NP_1             = 1                   ! First rotational harmonic
    ! NWTC Constants


    ! Control Modes
    ! VS_ControlMode


    ! VS_ConstPower

    ! VS_FBP

    ! VS_State

    ! PC_State

    ! PRC Mode constants
    


END MODULE Constants