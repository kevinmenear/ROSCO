!KGEN-generated Fortran source file 
  
!Generated at : 2026-03-25 20:41:15 
!KGEN version : 0.8.1 
  
!**********************************************************************************************************************************
! LICENSING
! Copyright (C) 2021  National Renewable Energy Laboratory
!    This file is part of ROSCO.
! Licensed under the Apache License, Version 2.0 (the "License");
! you may not use this file except in compliance with the License.
! You may obtain a copy of the License at
!     http://www.apache.org/licenses/LICENSE-2.0
! Unless required by applicable law or agreed to in writing, software
! distributed under the License is distributed on an "AS IS" BASIS,
! WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
! See the License for the specific language governing permissions and
! limitations under the License.
!**********************************************************************************************************************************


!
!
!
!
MODULE SysSubs

    USE constants 
    USE kgen_utils_mod
    USE tprof_mod, ONLY: tstart, tstop, tnull, tprnt 


    IMPLICIT NONE 


! This module contains routines with system-specific logic and references, including all references to the console unit, CU.
! It also contains standard (but not system-specific) routines it uses.
! SysGnuLinux.f90 is specifically for the GNU Fortran (gfortran) compiler on Linux and macOS.
 


!=======================================================================

     !=======================================================================

     !=======================================================================

     !=======================================================================

END MODULE SysSubs