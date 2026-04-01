!KGEN-generated Fortran source file 
  
!Generated at : 2026-04-01 17:20:55 
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
! Helper functions, primarily borrowed from NWTC_IO, for reading inputs and carrying out other helpful tasks


MODULE ROSCO_Helpers

    USE syssubs 
    USE kgen_utils_mod
    USE tprof_mod, ONLY: tstart, tstop, tnull, tprnt 


    IMPLICIT NONE 
    ! Global Variables

    


    !=======================================================================
    ! Parse integer input: read line, check that variable name is in line, handle errors


    !=======================================================================
    ! Parse integer input: read line, check that variable name is in line, handle errors


     !=======================================================================
    ! Parse integer input: read line, check that variable name is in line, handle errors


        !=======================================================================
    ! Parse integer input: read line, check that variable name is in line, handle errors


    !=======================================================================
    ! Parse double input, this is a copy of ParseInput_Int and a change in the variable definitions


    !=======================================================================
    ! Parse string input, this is a copy of ParseInput_Int and a change in the variable definitions


!=======================================================================
!> This subroutine parses the specified line of text for AryLen REAL values.
!! Generate an error message if the value is the wrong type.
!! Use ParseAry (nwtc_io::parseary) instead of directly calling a specific routine in the generic interface.   


  !=======================================================================
!> This subroutine parses the specified line of text for AryLen INTEGER values.
!! Generate an error message if the value is the wrong type.
!! Use ParseAry (nwtc_io::parseary) instead of directly calling a specific routine in the generic interface.   


!=======================================================================
!=======================================================================
!> This subroutine parses the specified line of text for AryLen INTEGER values.
!! Generate an error message if the value is the wrong type.
!! Use ParseAry (nwtc_io::parseary) instead of directly calling a specific routine in the generic interface.   


!=======================================================================
!> This subroutine parses the specified line of text for AryLen INTEGER values.
!! Generate an error message if the value is the wrong type.
!! Use ParseAry (nwtc_io::parseary) instead of directly calling a specific routine in the generic interface.   


!=======================================================================
 !> This subroutine checks the data to be parsed to make sure it finds
    !! the expected variable name and an associated value.


!=======================================================================


!=======================================================================
!> This subroutine is used to get the NumWords "words" from a line of text.
!! It uses spaces, tabs, commas, semicolons, single quotes, and double quotes ("whitespace")
!! as word separators. If there aren't NumWords in the line, the remaining array elements will remain empty.
!! Use CountWords (nwtc_io::countwords) to count the number of words in a line.


!=======================================================================
!> Let's parse the path name from the name of the given file.
!! We'll count everything before (and including) the last "\" or "/".

!=======================================================================
!> Let's parse the root file name from the name of the given file.
!! We'll count everything after the last period as the extension.
!! Borrowed from NWTC_IO...thanks!


!=======================================================================
!> This routine determines if the given file name is absolute or relative.
!! We will consider an absolute path one that satisfies one of the
!! following four criteria:
!!     1. It contains ":/"
!!     2. It contains ":\"
!!     3. It starts with "/"
!!     4. It starts with "\"
!!   
!! All others are considered relative.

!=======================================================================
! ------------------------------------------------------
    ! Read Open Loop Control Inputs
    ! Timeseries or lookup tables of the form
    ! index (time or wind speed)   channel_1 \t channel_2 \t channel_3 ...
    ! This could be used to read any group of data of unspecified length ...
    ! 


!=======================================================================
!> This routine returns the next unit number greater than 9 that is not currently in use.
!! If it cannot find any unit between 10 and 99 that is available, it either aborts or returns an appropriate error status/message.   


!=======================================================================
!> This function returns a character string encoded with the time in the form "hh:mm:ss".


!=======================================================================
! This function checks whether an array is non-decreasing


!=======================================================================
!> This routine converts all the text in a string to upper case.


!=======================================================================
     !> This function returns a left-adjusted string representing the passed numeric value. 
    !! It eliminates trailing zeroes and even the decimal point if it is not a fraction. \n
    !! Use Num2LStr (nwtc_io::num2lstr) instead of directly calling a specific routine in the generic interface.   


!=======================================================================


    !-------------------------------------------------------------------------------------------------------------------------------
    ! Copied from NWTC_IO.f90
    !> This function returns a character string encoded with today's date in the form dd-mmm-ccyy.


END MODULE ROSCO_Helpers