.PHONY: all additive_synth play

CC:=gcc
C_FLAGS:=-ggdb
C_INCLUDES:=-I ../../Drivers/CMSIS/DSP/Include -I ../../Drivers/CMSIS/DSP/Include/dsp \
	-I ../../Drivers/CMSIS/Core/Include -I ../Inc/
C_DEFINES:=-DARM_MATH_GENERIC
CMSIS_DSP_LIBRARY_SOURCES:=../../Drivers/CMSIS/DSP/Source/TransformFunctions/TransformFunctions.c \
	../../Drivers/CMSIS/DSP/Source/CommonTables/CommonTables.c \
	../../Drivers/CMSIS/DSP/Source/FastMathFunctions/FastMathFunctions.c \
	../../Drivers/CMSIS/DSP/Source/BasicMathFunctions/BasicMathFunctions.c \
	../../Drivers/CMSIS/DSP/Source/ComplexMathFunctions/ComplexMathFunctions.c \
	../../Drivers/CMSIS/DSP/Source/StatisticsFunctions/StatisticsFunctions.c \
	../../Drivers/CMSIS/DSP/Source/MatrixFunctions/MatrixFunctions.c
COMMON_SOURCES:=../Src/wavegen.c \
	../Src/organ_presets.c
C_LIBS:=-lm

additive_synth:
	${CC} ${C_FLAGS} -o additive_synth.out \
		${C_INCLUDES} \
		${C_DEFINES} \
		${CMSIS_DSP_LIBRARY_SOURCES} \
		${COMMON_SOURCES} \
		additive_synth_test.c \
		${C_LIBS}
	./additive_synth.out
	python soundify.py

play:
	play output.wav

all: additive_synth play

