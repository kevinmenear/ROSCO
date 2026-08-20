! Copyright 2019 NREL

! Licensed under the Apache License, Version 2.0 (the "License"); you may not use
! this file except in compliance with the License. You may obtain a copy of the
! License at http://www.apache.org/licenses/LICENSE-2.0

! Unless required by applicable law or agreed to in writing, software distributed
! under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
! CONDITIONS OF ANY KIND, either express or implied. See the License for the
! specific language governing permissions and limitations under the License.
! -------------------------------------------------------------------------------------------
! Read and set the parameters used by the controller

MODULE ReadSetParameters

    USE, INTRINSIC :: ISO_C_Binding

    USE Constants
    USE Functions
    USE Filters
    USE SysSubs
    USE ROSCO_Helpers
    IMPLICIT NONE



    ! Auto-generated interface for C++ implementation of ReadAvrSWAP
    INTERFACE
        SUBROUTINE readavrswap_c(avrSWAP, LocalVar, CntrPar, ErrVar) BIND(C, NAME='readavrswap_c')
            USE ISO_C_BINDING
            REAL(C_FLOAT), INTENT(INOUT) :: avrSWAP(*)
            TYPE(C_PTR), VALUE :: LocalVar
            TYPE(C_PTR), VALUE :: CntrPar
            TYPE(C_PTR), VALUE :: ErrVar
        END SUBROUTINE readavrswap_c
    END INTERFACE


    ! Auto-generated interface for C++ implementation of CheckInputs
    INTERFACE
        SUBROUTINE checkinputs_c(LocalVar, CntrPar, avrSWAP, ErrVar, size_avcMSG) BIND(C, NAME='checkinputs_c')
            USE ISO_C_BINDING
            TYPE(C_PTR), VALUE :: LocalVar
            TYPE(C_PTR), VALUE :: CntrPar
            REAL(C_FLOAT), INTENT(IN) :: avrSWAP(*)
            TYPE(C_PTR), VALUE :: ErrVar
            INTEGER(C_INT), VALUE :: size_avcMSG
        END SUBROUTINE checkinputs_c
    END INTERFACE


    ! Auto-generated interface for C++ implementation of ReadControlParameterFileSub
    INTERFACE
        SUBROUTINE readcontrolparameterfilesub_c(CntrPar, LocalVar, accINFILE, accINFILE_size, RootName, ErrVar) BIND(C, NAME='readcontrolparameterfilesub_c')
            USE ISO_C_BINDING
            TYPE(C_PTR), VALUE :: CntrPar
            TYPE(C_PTR), VALUE :: LocalVar
            CHARACTER(KIND=C_CHAR), INTENT(IN) :: accINFILE(*)
            INTEGER(C_INT), VALUE :: accINFILE_size
            CHARACTER(KIND=C_CHAR), INTENT(IN) :: RootName(*)
            TYPE(C_PTR), VALUE :: ErrVar
        END SUBROUTINE readcontrolparameterfilesub_c
    END INTERFACE

CONTAINS
 ! -----------------------------------------------------------------------------------
    ! Read avrSWAP array passed from ServoDyn    
    SUBROUTINE ReadAvrSWAP(avrSWAP, LocalVar, CntrPar, ErrVar)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : LocalVariables
        USE vit_localvariables_view, ONLY: localvariables_view_t, vit_populate_localvariables, vit_copy_scalars_to_localvariables
        USE vit_controlparameters_view, ONLY: controlparameters_view_t, vit_populate_controlparameters, vit_copy_scalars_to_controlparameters
        USE vit_errorvariables_view, ONLY: errorvariables_view_t, vit_populate_errorvariables, vit_copy_scalars_to_errorvariables
        IMPLICIT NONE
        REAL(4), INTENT(INOUT) :: avrSWAP(*)
        TYPE(LOCALVARIABLES), INTENT(INOUT), TARGET :: LocalVar
        TYPE(CONTROLPARAMETERS), INTENT(IN), TARGET :: CntrPar
        TYPE(ERRORVARIABLES), INTENT(INOUT), TARGET :: ErrVar
        TYPE(localvariables_view_t), TARGET :: LocalVar_view
        TYPE(controlparameters_view_t), TARGET :: CntrPar_view
        TYPE(errorvariables_view_t), TARGET :: ErrVar_view
        ! Populate view structs from Fortran types
        CALL vit_populate_localvariables(LocalVar, LocalVar_view)
        CALL vit_populate_controlparameters(CntrPar, CntrPar_view)
        CALL vit_populate_errorvariables(ErrVar, ErrVar_view)
        CALL readavrswap_c(avrSWAP, C_LOC(LocalVar_view), C_LOC(CntrPar_view), C_LOC(ErrVar_view))
        ! Copy modified scalars back from view to Fortran type
        CALL vit_copy_scalars_to_localvariables(LocalVar_view, LocalVar)
        CALL vit_copy_scalars_to_errorvariables(ErrVar_view, ErrVar)
    END SUBROUTINE ReadAvrSWAP
! -----------------------------------------------------------------------------------
    ! Define parameters for control actions
    SUBROUTINE SetParameters(avrSWAP, accINFILE, size_avcMSG, CntrPar, LocalVar, objInst, PerfData, RootName, ErrVar)
                
        USE ROSCO_Types, ONLY : ControlParameters, LocalVariables, ObjectInstances, PerformanceData, ErrorVariables
        
        REAL(ReKi),                 INTENT(INOUT)   :: avrSWAP(*)          ! The swap array, used to pass data to, and receive data from, the DLL controller.
        CHARACTER(C_CHAR),          INTENT(IN   )   :: accINFILE(NINT(avrSWAP(50)))     ! The name of the parameter input file

        INTEGER(IntKi),             INTENT(IN   )   :: size_avcMSG
        TYPE(ControlParameters),    INTENT(INOUT)   :: CntrPar
        TYPE(LocalVariables),       INTENT(INOUT)   :: LocalVar
        TYPE(ObjectInstances),      INTENT(INOUT)   :: objInst
        TYPE(PerformanceData),      INTENT(INOUT)   :: PerfData
        TYPE(ErrorVariables),       INTENT(INOUT)   :: ErrVar
        CHARACTER(NINT(avrSWAP(50))-1), INTENT(IN)  :: RootName 

        
        INTEGER(IntKi)                              :: K, I, I_OL    ! Index used for looping through blades.
        CHARACTER(1024)                             :: OL_String                    ! Open description loop string
        INTEGER(IntKi)                              :: OL_Count                     ! Number of open loop channels
        INTEGER(IntKi)                              :: UnOpenLoop       ! Open Loop file unit
        INTEGER(IntKi)                              :: N_OL_Cables
        INTEGER(IntKi)                              :: N_OL_StCs

        CHARACTER(*),               PARAMETER       :: RoutineName = 'SetParameters'

        
        
        ! Error Catching Variables
        ! Set ErrVar%aviFAIL to 0 in each iteration:
        ErrVar%aviFAIL = 0
        ! ALLOCATE(ErrVar%ErrMsg(size_avcMSG-1))
        ErrVar%size_avcMSG  = size_avcMSG
        
        ! Initialize all filter instance counters at 1
        objInst%instLPF         = 1
        objInst%instSecLPF      = 1
        objInst%instSecLPFV     = 1
        objInst%instHPF         = 1
        objInst%instNotchSlopes = 1
        objInst%instNotch       = 1
        objInst%instPI          = 1
        objInst%instRes         = 1
        objInst%instRL          = 1
        
        ! Set unused outputs to zero (See Appendix A of Bladed User's Guide):
        avrSWAP(35) = 1.0 ! Generator contactor status: 1=main (high speed) variable-speed generator
        avrSWAP(36) = 0.0 ! Shaft brake status: 0=off
        avrSWAP(41) = 0.0 ! Demanded yaw actuator torque
        avrSWAP(46) = 0.0 ! Demanded pitch rate (Collective pitch)
        avrSWAP(55) = 0.0 ! Pitch override: 0=yes
        avrSWAP(56) = 0.0 ! Torque override: 0=yes
        avrSWAP(65) = 0.0 ! Number of variables returned for logging
        avrSWAP(72) = 0.0 ! Generator start-up resistance
        avrSWAP(79) = 4.0 ! Request for loads: 0=none
        avrSWAP(80) = 0.0 ! Variable slip current status
        avrSWAP(81) = 0.0 ! Variable slip current demand
        
        ! Read any External Controller Parameters specified in the User Interface
        !   and initialize variables:
        IF (LocalVar%iStatus == 0) THEN ! .TRUE. if we're on the first call to the DLL
            
            ! Inform users that we are using this user-defined routine:
            ! ErrVar%aviFAIL = 1
            write (*,*) '                                                                              '//NEW_LINE('A')// &
                        '------------------------------------------------------------------------------'//NEW_LINE('A')// &
                        'Running ROSCO-'//TRIM(rosco_version)//NEW_LINE('A')// &
                        'A wind turbine controller framework for public use in the scientific field    '//NEW_LINE('A')// &
                        'Developed in collaboration: National Renewable Energy Laboratory              '//NEW_LINE('A')// &
                        '                            Delft University of Technology, The Netherlands   '//NEW_LINE('A')// &
                        '------------------------------------------------------------------------------'
            ! Specifically save accINFILE info (DISCON.IN)
            LocalVar%ACC_INFILE_SIZE = NINT(avrSWAP(50))
            Allocate(LocalVar%ACC_INFILE(LocalVar%ACC_INFILE_SIZE))
            LocalVar%ACC_INFILE = accINFILE

            ! Read Control Parameter File
            CALL ReadControlParameterFileSub(CntrPar, LocalVar, accINFILE, NINT(avrSWAP(50)), RootName, ErrVar)
            ! If there's been an file reading error, don't continue
            ! Add RoutineName to error message
            IF (ErrVar%aviFAIL < 0) THEN
                ErrVar%ErrMsg = RoutineName//':'//TRIM(ErrVar%ErrMsg)
                RETURN
            ENDIF

            IF (CntrPar%WE_Mode > 0) THEN
                CALL READCpFile(CntrPar,PerfData,ErrVar)
            ENDIF
        
            ! Initialize the Local variables:
            LocalVar%PitCom     = LocalVar%BlPitch ! This will ensure that the variable speed controller picks the correct control region and the pitch controller picks the correct gain on the first call

            ! Wind speed estimator initialization
            LocalVar%WE_Vw      = LocalVar%HorWindV
            LocalVar%WE_VwI     = LocalVar%WE_Vw - CntrPar%WE_Gamma*LocalVar%RotSpeed
            LocalVar%WE_Op      = 1
            LocalVar%WE_Op_Last = 1
            
            ! Setpoint Smoother initialization to zero
            LocalVar%SS_DelOmegaF = 0

            IF (CntrPar%VS_FBP == VS_FBP_Variable_Pitch) THEN
                ! Generator Torque at K omega^2 or rated
                IF (LocalVar%GenSpeed > 0.98 * CntrPar%PC_RefSpd) THEN
                    LocalVar%GenTq = CntrPar%VS_RtTq
                ELSE
                    LocalVar%GenTq = min(CntrPar%VS_RtTq, CntrPar%VS_Rgn2K*LocalVar%GenSpeed*LocalVar%GenSpeed)
                ENDIF
            ELSE
                ! Set torque initial condition based on operating schedule at current wind speed
                LocalVar%GenTq = interp1d(CntrPar%VS_FBP_U, CntrPar%VS_FBP_Tau, LocalVar%HorWindV, ErrVar)
            ENDIF
            LocalVar%VS_LastGenTrq = LocalVar%GenTq
            LocalVar%VS_MaxTq      = CntrPar%VS_MaxTq
            LocalVar%VS_GenPwr     = LocalVar%GenTq * LocalVar%GenSpeed * CntrPar%VS_GenEff/100.0
            
            ! Initialize variables
            LocalVar%CC_DesiredL = 0
            LocalVar%CC_ActuatedL = 0
            LocalVar%CC_ActuatedDL = 0
            LocalVar%StC_Input = 0

            LocalVar%ZMQ_YawOffset = 0
            LocalVar%ZMQ_PitOffset = 0
            LocalVar%ZMQ_ID = CntrPar%ZMQ_ID

            ! Check validity of input parameters:
            CALL CheckInputs(LocalVar, CntrPar, avrSWAP, ErrVar, size_avcMSG)

            ! Add RoutineName to error message
            IF (ErrVar%aviFAIL < 0) THEN
                ErrVar%ErrMsg = RoutineName//':'//TRIM(ErrVar%ErrMsg)
            ENDIF


        ENDIF

        ! Initialize other values on each timestep
        
        ! Open Loop index
        IF (CntrPar%OL_BP_Mode == 0) THEN
            LocalVar%OL_Index = LocalVar%Time

        ELSE
            ! Wind speed, OL_BP_Mode = 1
            LocalVar%OL_Index = LocalVar%WE_Vw
            IF (CntrPar%OL_BP_FiltFreq > 0) THEN
                LocalVar%OL_Index = LPFilter(LocalVar%WE_Vw, LocalVar%DT,CntrPar%OL_BP_FiltFreq, LocalVar%FP, LocalVar%iStatus, LocalVar%restart, objInst%instLPF)
            ENDIF

        ENDIF

    END SUBROUTINE SetParameters
    
    ! -----------------------------------------------------------------------------------
    ! Read all constant control parameters from DISCON.IN parameter file
    ! Also, all computed CntrPar%* parameters should be computed in this subroutine
    SUBROUTINE ReadControlParameterFileSub(CntrPar, LocalVar, accINFILE, accINFILE_size, RootName, ErrVar)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : ControlParameters, ErrorVariables, LocalVariables
        USE vit_controlparameters_view, ONLY: controlparameters_view_t, vit_populate_controlparameters, vit_copy_scalars_to_controlparameters, vit_view_in_controlparameters
        USE vit_localvariables_view, ONLY: localvariables_view_t, vit_populate_localvariables, vit_copy_scalars_to_localvariables
        USE vit_errorvariables_view, ONLY: errorvariables_view_t, vit_populate_errorvariables, vit_copy_scalars_to_errorvariables
        IMPLICIT NONE
        TYPE(CONTROLPARAMETERS), INTENT(INOUT), TARGET :: CntrPar
        TYPE(LOCALVARIABLES), INTENT(INOUT), TARGET :: LocalVar
        INTEGER(4) :: accINFILE_size
        TYPE(ERRORVARIABLES), INTENT(INOUT), TARGET :: ErrVar
        CHARACTER(ACCINFILE_SIZE), INTENT(IN) :: accINFILE(ACCINFILE_SIZE)
        CHARACTER(ACCINFILE_SIZE), INTENT(IN) :: RootName
        CHARACTER(KIND=C_CHAR) :: accINFILE_c((accINFILE_size) * ((ACCINFILE_SIZE)))
        INTEGER :: vit_i_accINFILE, vit_j_accINFILE
        CHARACTER(KIND=C_CHAR) :: RootName_c(accINFILE_size)
        INTEGER :: vit_i_RootName
        TYPE(controlparameters_view_t), TARGET :: CntrPar_view
        TYPE(localvariables_view_t), TARGET :: LocalVar_view
        TYPE(errorvariables_view_t), TARGET :: ErrVar_view
        ! Populate view structs from Fortran types
        CALL vit_populate_controlparameters(CntrPar, CntrPar_view)
        CALL vit_populate_localvariables(LocalVar, LocalVar_view)
        CALL vit_populate_errorvariables(ErrVar, ErrVar_view)
        ! Convert CHARACTER args to C_CHAR arrays
        DO vit_j_accINFILE = 1, (ACCINFILE_SIZE)
            DO vit_i_accINFILE = 1, accINFILE_size
                accINFILE_c((vit_j_accINFILE - 1) * (accINFILE_size) + vit_i_accINFILE) = &
                    accINFILE(vit_j_accINFILE)(vit_i_accINFILE:vit_i_accINFILE)
            END DO
        END DO
        DO vit_i_RootName = 1, accINFILE_size
            RootName_c(vit_i_RootName) = RootName(vit_i_RootName:vit_i_RootName)
        END DO
        CALL readcontrolparameterfilesub_c(C_LOC(CntrPar_view), C_LOC(LocalVar_view), accINFILE_c, accINFILE_size, RootName_c, C_LOC(ErrVar_view))
        ! Copy modified scalars back from view to Fortran type
        CALL vit_copy_scalars_to_controlparameters(CntrPar_view, CntrPar)
        CALL vit_copy_scalars_to_localvariables(LocalVar_view, LocalVar)
        CALL vit_copy_scalars_to_errorvariables(ErrVar_view, ErrVar)
        ! Copy ALLOCATABLE fields back from view to Fortran type
        CALL vit_view_in_controlparameters(CntrPar_view, CntrPar)
    END SUBROUTINE ReadControlParameterFileSub
    ! -----------------------------------------------------------------------------------
    ! Read all constant control parameters from DISCON.IN parameter file
    SUBROUTINE ReadCpFile(CntrPar,PerfData, ErrVar)
        USE ROSCO_Types, ONLY : PerformanceData, ControlParameters, ErrorVariables
        
        TYPE(ControlParameters),    INTENT(INOUT)   :: CntrPar
        TYPE(PerformanceData),      INTENT(INOUT)   :: PerfData
        TYPE(ErrorVariables),       INTENT(INOUT)   :: ErrVar
        
        ! Local variables
        INTEGER(IntKi)                                  :: UnPerfParameters
        INTEGER(IntKi)                                  :: i ! iteration index

        INTEGER(IntKi)                                  :: CurLine 
        INTEGER                                         :: IOS   
        CHARACTER(256)                                  :: IOM
        CHARACTER(*), PARAMETER                         :: RoutineName = 'ReadCpFile'
        REAL(DbKi), DIMENSION(:), ALLOCATABLE           :: TmpPerf

        CurLine = 1
        CALL GetNewUnit(UnPerfParameters, ErrVar)
        OPEN(unit=UnPerfParameters, file=TRIM(CntrPar%PerfFileName), status='old', action='read')
                
        ! ----------------------- Axis Definitions ------------------------
        CALL ReadEmptyLine(UnPerfParameters,CurLine)
        CALL ReadEmptyLine(UnPerfParameters,CurLine)
        CALL ReadEmptyLine(UnPerfParameters,CurLine)
        CALL ReadEmptyLine(UnPerfParameters,CurLine)
        CALL ParseAry(UnPerfParameters, CurLine, 'Pitch angle vector', PerfData%Beta_vec, CntrPar%PerfTableSize(1), TRIM(CntrPar%PerfFileName), ErrVar, .FALSE.)
        CALL ReadEmptyLine(UnPerfParameters,CurLine) 
        CALL ParseAry(UnPerfParameters, CurLine, 'TSR vector', PerfData%TSR_vec, CntrPar%PerfTableSize(2), TRIM(CntrPar%PerfFileName), ErrVar, .FALSE.)

        ! ----------------------- Read Cp, Ct, Cq, Tables ------------------------
        CALL ReadEmptyLine(UnPerfParameters,CurLine) 
        CALL ReadEmptyLine(UnPerfParameters,CurLine) ! Input file should contains wind speed information here - unneeded for now
        CALL ReadEmptyLine(UnPerfParameters,CurLine) 
        CALL ReadEmptyLine(UnPerfParameters,CurLine) 
        CALL ReadEmptyLine(UnPerfParameters,CurLine) 
        ALLOCATE(PerfData%Cp_mat(CntrPar%PerfTableSize(2),CntrPar%PerfTableSize(1)))
        DO i = 1,CntrPar%PerfTableSize(2)
            READ(UnPerfParameters, *,IOSTAT=IOS,IOMSG=IOM) PerfData%Cp_mat(i,:) ! Read Cp table
            IF (IOS > 0) THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg = "Error reading "//TRIM(CntrPar%PerfFileName)//" Cp table, IOMSG="//IOM//"Please check formatting and size of matrices in that file."
                CLOSE(UnPerfParameters)
                RETURN
            ENDIF
        END DO
        CALL ReadEmptyLine(UnPerfParameters,CurLine) 
        CALL ReadEmptyLine(UnPerfParameters,CurLine) 
        CALL ReadEmptyLine(UnPerfParameters,CurLine) 
        CALL ReadEmptyLine(UnPerfParameters,CurLine) 
        ALLOCATE(PerfData%Ct_mat(CntrPar%PerfTableSize(2),CntrPar%PerfTableSize(1)))
        DO i = 1,CntrPar%PerfTableSize(2)
            READ(UnPerfParameters, *,IOSTAT=IOS,IOMSG=IOM) PerfData%Ct_mat(i,:) ! Read Ct table
            IF (IOS > 0) THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg = "Error reading "//TRIM(CntrPar%PerfFileName)//" Cp table, IOMSG="//IOM//"Please check formatting and size of matrices in that file."
                CLOSE(UnPerfParameters)
                RETURN
            ENDIF
        END DO
        CALL ReadEmptyLine(UnPerfParameters,CurLine) 
        CALL ReadEmptyLine(UnPerfParameters,CurLine) 
        CALL ReadEmptyLine(UnPerfParameters,CurLine) 
        CALL ReadEmptyLine(UnPerfParameters,CurLine) 
        ALLOCATE(PerfData%Cq_mat(CntrPar%PerfTableSize(2),CntrPar%PerfTableSize(1)))
        DO i = 1,CntrPar%PerfTableSize(2)
            READ(UnPerfParameters, *,IOSTAT=IOS,IOMSG=IOM) PerfData%Cq_mat(i,:) ! Read Cq table
            IF (IOS > 0) THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg = "Error reading "//TRIM(CntrPar%PerfFileName)//" Cp table IOMSG="//IOM//"Please check formatting and size of matrices in that file."
                CLOSE(UnPerfParameters)
                RETURN
            ENDIF
        END DO

        ! Close file
        CLOSE(UnPerfParameters)

        ! Add RoutineName to error message
        IF (ErrVar%aviFAIL < 0) THEN
            ErrVar%ErrMsg = RoutineName//':'//TRIM(ErrVar%ErrMsg)
        ENDIF
    
    END SUBROUTINE ReadCpFile
    ! -----------------------------------------------------------------------------------
    ! Check for errors before any execution
    SUBROUTINE CheckInputs(LocalVar, CntrPar, avrSWAP, ErrVar, size_avcMSG)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : LocalVariables, ControlParameters, ErrorVariables
        USE vit_localvariables_view, ONLY: localvariables_view_t, vit_populate_localvariables, vit_copy_scalars_to_localvariables
        USE vit_controlparameters_view, ONLY: controlparameters_view_t, vit_populate_controlparameters, vit_copy_scalars_to_controlparameters
        USE vit_errorvariables_view, ONLY: errorvariables_view_t, vit_populate_errorvariables, vit_copy_scalars_to_errorvariables
        IMPLICIT NONE
        TYPE(LOCALVARIABLES), INTENT(INOUT), TARGET :: LocalVar
        TYPE(CONTROLPARAMETERS), INTENT(INOUT), TARGET :: CntrPar
        REAL(4), INTENT(IN) :: avrSWAP(*)
        TYPE(ERRORVARIABLES), INTENT(INOUT), TARGET :: ErrVar
        INTEGER(4), INTENT(IN) :: size_avcMSG
        TYPE(localvariables_view_t), TARGET :: LocalVar_view
        TYPE(controlparameters_view_t), TARGET :: CntrPar_view
        TYPE(errorvariables_view_t), TARGET :: ErrVar_view
        ! Populate view structs from Fortran types
        CALL vit_populate_localvariables(LocalVar, LocalVar_view)
        CALL vit_populate_controlparameters(CntrPar, CntrPar_view)
        CALL vit_populate_errorvariables(ErrVar, ErrVar_view)
        CALL checkinputs_c(C_LOC(LocalVar_view), C_LOC(CntrPar_view), avrSWAP, C_LOC(ErrVar_view), size_avcMSG)
        ! Copy modified scalars back from view to Fortran type
        CALL vit_copy_scalars_to_localvariables(LocalVar_view, LocalVar)
        CALL vit_copy_scalars_to_controlparameters(CntrPar_view, CntrPar)
        CALL vit_copy_scalars_to_errorvariables(ErrVar_view, ErrVar)
    END SUBROUTINE CheckInputs
    
END MODULE ReadSetParameters
