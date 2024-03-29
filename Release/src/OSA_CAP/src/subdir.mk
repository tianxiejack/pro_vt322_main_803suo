################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/OSA_CAP/src/osa.cpp \
../src/OSA_CAP/src/osa_buf.cpp \
../src/OSA_CAP/src/osa_eth_client.cpp \
../src/OSA_CAP/src/osa_eth_server.cpp \
../src/OSA_CAP/src/osa_event.cpp \
../src/OSA_CAP/src/osa_file.cpp \
../src/OSA_CAP/src/osa_i2c.cpp \
../src/OSA_CAP/src/osa_mbx.cpp \
../src/OSA_CAP/src/osa_msgq.cpp \
../src/OSA_CAP/src/osa_mutex.cpp \
../src/OSA_CAP/src/osa_pipe.cpp \
../src/OSA_CAP/src/osa_prf.cpp \
../src/OSA_CAP/src/osa_que.cpp \
../src/OSA_CAP/src/osa_rng.cpp \
../src/OSA_CAP/src/osa_sem.cpp \
../src/OSA_CAP/src/osa_thr.cpp \
../src/OSA_CAP/src/osa_tsk.cpp 

OBJS += \
./src/OSA_CAP/src/osa.o \
./src/OSA_CAP/src/osa_buf.o \
./src/OSA_CAP/src/osa_eth_client.o \
./src/OSA_CAP/src/osa_eth_server.o \
./src/OSA_CAP/src/osa_event.o \
./src/OSA_CAP/src/osa_file.o \
./src/OSA_CAP/src/osa_i2c.o \
./src/OSA_CAP/src/osa_mbx.o \
./src/OSA_CAP/src/osa_msgq.o \
./src/OSA_CAP/src/osa_mutex.o \
./src/OSA_CAP/src/osa_pipe.o \
./src/OSA_CAP/src/osa_prf.o \
./src/OSA_CAP/src/osa_que.o \
./src/OSA_CAP/src/osa_rng.o \
./src/OSA_CAP/src/osa_sem.o \
./src/OSA_CAP/src/osa_thr.o \
./src/OSA_CAP/src/osa_tsk.o 

CPP_DEPS += \
./src/OSA_CAP/src/osa.d \
./src/OSA_CAP/src/osa_buf.d \
./src/OSA_CAP/src/osa_eth_client.d \
./src/OSA_CAP/src/osa_eth_server.d \
./src/OSA_CAP/src/osa_event.d \
./src/OSA_CAP/src/osa_file.d \
./src/OSA_CAP/src/osa_i2c.d \
./src/OSA_CAP/src/osa_mbx.d \
./src/OSA_CAP/src/osa_msgq.d \
./src/OSA_CAP/src/osa_mutex.d \
./src/OSA_CAP/src/osa_pipe.d \
./src/OSA_CAP/src/osa_prf.d \
./src/OSA_CAP/src/osa_que.d \
./src/OSA_CAP/src/osa_rng.d \
./src/OSA_CAP/src/osa_sem.d \
./src/OSA_CAP/src/osa_thr.d \
./src/OSA_CAP/src/osa_tsk.d 


# Each subdirectory must supply rules for building sources it contributes
src/OSA_CAP/src/%.o: ../src/OSA_CAP/src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: NVCC Compiler'
	/usr/local/cuda-8.0/bin/nvcc -I/usr/local/include -I../src/OSA_CAP/inc -I../inc/ -I../src/ptz/inc -I../src/DxTimer/include -I../src/port -I../src/comm -I../src/platform -I../src/IPC -I/usr/local/include/libusb-1.0 -I../src/803 -O3 -ccbin aarch64-linux-gnu-g++ -gencode arch=compute_53,code=sm_53 -m64 -odir "src/OSA_CAP/src" -M -o "$(@:%.o=%.d)" "$<"
	/usr/local/cuda-8.0/bin/nvcc -I/usr/local/include -I../src/OSA_CAP/inc -I../inc/ -I../src/ptz/inc -I../src/DxTimer/include -I../src/port -I../src/comm -I../src/platform -I../src/IPC -I/usr/local/include/libusb-1.0 -I../src/803 -O3 --compile -m64 -ccbin aarch64-linux-gnu-g++  -x c++ -o  "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


