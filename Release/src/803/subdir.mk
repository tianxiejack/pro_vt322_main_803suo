################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/803/803uart.cpp 

OBJS += \
./src/803/803uart.o 

CPP_DEPS += \
./src/803/803uart.d 


# Each subdirectory must supply rules for building sources it contributes
src/803/%.o: ../src/803/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: NVCC Compiler'
	/usr/local/cuda-8.0/bin/nvcc -I/usr/local/include -I../src/OSA_CAP/inc -I../inc/ -I../src/ptz/inc -I../src/DxTimer/include -I../src/port -I../src/comm -I../src/platform -I../src/IPC -I/usr/local/include/libusb-1.0 -I../src/803 -O3 -ccbin aarch64-linux-gnu-g++ -gencode arch=compute_53,code=sm_53 -m64 -odir "src/803" -M -o "$(@:%.o=%.d)" "$<"
	/usr/local/cuda-8.0/bin/nvcc -I/usr/local/include -I../src/OSA_CAP/inc -I../inc/ -I../src/ptz/inc -I../src/DxTimer/include -I../src/port -I../src/comm -I../src/platform -I../src/IPC -I/usr/local/include/libusb-1.0 -I../src/803 -O3 --compile -m64 -ccbin aarch64-linux-gnu-g++  -x c++ -o  "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

