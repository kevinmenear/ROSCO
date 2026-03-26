FC_0 ?= gfortran
FC_FLAGS_SET_0 ?= -O2 -ffree-form -ffree-line-length-none -fdefault-real-8 -fdefault-double-8 -ffp-contract=off
# Makefile for KGEN-generated kernel


ALL_OBJS := vit_bridge_wrap_360.o vit_bridge_sigma.o vit_bridge_picontroller.o vit_bridge_lpfilter.o vit_bridge_colemantransforminverse.o vit_bridge_colemantransform.o vit_controlparameters_view.o ipc_cpp.o Controllers.o ROSCO_Types.o Constants.o ControllerBlocks.o SysIFL.o Filters.o Functions.o kernel_driver.o kgen_utils.o tprof_mod.o

# VIT: C++ compilation support
CXX ?= g++
CXXFLAGS ?= -O2 -fPIC -ffp-contract=off

ipc_cpp.o: ipc_cpp.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# VIT: view populator module
vit_controlparameters_view.o: vit_controlparameters_view.f90 ROSCO_Types.o
	$(FC_0) $(FC_FLAGS_SET_0) -c $< -o $@

# VIT: callee bridge for kernel verification
vit_bridge_colemantransform.o: vit_bridge_colemantransform.f90 vit_controlparameters_view.o Controllers.o ROSCO_Types.o Constants.o ControllerBlocks.o SysIFL.o Filters.o Functions.o kernel_driver.o kgen_utils.o tprof_mod.o
	$(FC_0) $(FC_FLAGS_SET_0) -c $< -o $@

# VIT: callee bridge for kernel verification
vit_bridge_colemantransforminverse.o: vit_bridge_colemantransforminverse.f90 vit_bridge_colemantransform.o vit_controlparameters_view.o Controllers.o ROSCO_Types.o Constants.o ControllerBlocks.o SysIFL.o Filters.o Functions.o kernel_driver.o kgen_utils.o tprof_mod.o
	$(FC_0) $(FC_FLAGS_SET_0) -c $< -o $@

# VIT: callee bridge for kernel verification
vit_bridge_lpfilter.o: vit_bridge_lpfilter.f90 vit_bridge_colemantransforminverse.o vit_bridge_colemantransform.o vit_controlparameters_view.o Controllers.o ROSCO_Types.o Constants.o ControllerBlocks.o SysIFL.o Filters.o Functions.o kernel_driver.o kgen_utils.o tprof_mod.o
	$(FC_0) $(FC_FLAGS_SET_0) -c $< -o $@

# VIT: callee bridge for kernel verification
vit_bridge_picontroller.o: vit_bridge_picontroller.f90 vit_bridge_lpfilter.o vit_bridge_colemantransforminverse.o vit_bridge_colemantransform.o vit_controlparameters_view.o Controllers.o ROSCO_Types.o Constants.o ControllerBlocks.o SysIFL.o Filters.o Functions.o kernel_driver.o kgen_utils.o tprof_mod.o
	$(FC_0) $(FC_FLAGS_SET_0) -c $< -o $@

# VIT: callee bridge for kernel verification
vit_bridge_sigma.o: vit_bridge_sigma.f90 vit_bridge_picontroller.o vit_bridge_lpfilter.o vit_bridge_colemantransforminverse.o vit_bridge_colemantransform.o vit_controlparameters_view.o Controllers.o ROSCO_Types.o Constants.o ControllerBlocks.o SysIFL.o Filters.o Functions.o kernel_driver.o kgen_utils.o tprof_mod.o
	$(FC_0) $(FC_FLAGS_SET_0) -c $< -o $@

# VIT: callee bridge for kernel verification
vit_bridge_wrap_360.o: vit_bridge_wrap_360.f90 vit_bridge_sigma.o vit_bridge_picontroller.o vit_bridge_lpfilter.o vit_bridge_colemantransforminverse.o vit_bridge_colemantransform.o vit_controlparameters_view.o Controllers.o ROSCO_Types.o Constants.o ControllerBlocks.o SysIFL.o Filters.o Functions.o kernel_driver.o kgen_utils.o tprof_mod.o
	$(FC_0) $(FC_FLAGS_SET_0) -c $< -o $@

build: ${ALL_OBJS}
	${FC_0} ${FC_FLAGS_SET_0} -o kernel.exe -lstdc++ $^  

run: build
	./kernel.exe

Controllers.o: Controllers.f90 ROSCO_Types.o Constants.o ControllerBlocks.o SysIFL.o Filters.o Functions.o kgen_utils.o tprof_mod.o vit_controlparameters_view.o
	${FC_0} ${FC_FLAGS_SET_0} -c -o $@ $<

ROSCO_Types.o: ROSCO_Types.f90 kgen_utils.o tprof_mod.o Constants.o
	${FC_0} ${FC_FLAGS_SET_0} -c -o $@ $<

Constants.o: Constants.f90 kgen_utils.o tprof_mod.o
	${FC_0} ${FC_FLAGS_SET_0} -c -o $@ $<

ControllerBlocks.o: ControllerBlocks.f90 kgen_utils.o tprof_mod.o SysIFL.o Filters.o
	${FC_0} ${FC_FLAGS_SET_0} -c -o $@ $<

SysIFL.o: SysIFL.f90 kgen_utils.o tprof_mod.o Constants.o
	${FC_0} ${FC_FLAGS_SET_0} -c -o $@ $<

Filters.o: Filters.f90 kgen_utils.o tprof_mod.o Functions.o ROSCO_Types.o
	${FC_0} ${FC_FLAGS_SET_0} -c -o $@ $<

Functions.o: Functions.f90 kgen_utils.o tprof_mod.o Constants.o ROSCO_Types.o
	${FC_0} ${FC_FLAGS_SET_0} -c -o $@ $<

kernel_driver.o: kernel_driver.f90 Controllers.o ROSCO_Types.o Constants.o ControllerBlocks.o SysIFL.o Filters.o Functions.o kgen_utils.o tprof_mod.o
	${FC_0} ${FC_FLAGS_SET_0} -c -o $@ $<

kgen_utils.o: kgen_utils.f90
	${FC_0} ${FC_FLAGS_SET_0} -c -o $@ $<

tprof_mod.o: tprof_mod.f90
	${FC_0} ${FC_FLAGS_SET_0} -c -o $@ $<

clean:
	rm -f kernel.exe *.mod ${ALL_OBJS}
