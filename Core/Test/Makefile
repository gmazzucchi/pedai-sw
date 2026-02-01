C_INCLUDES:=-I ../../Drivers/CMSIS/DSP/Include -I ../../Drivers/CMSIS/DSP/Include/dsp -I ../../Drivers/CMSIS/Core/Include
C_DEFINES:=-DARM_MATH_GENERIC
COMMON_SOURCE_FILES:=../../Drivers/CMSIS/DSP/Source/TransformFunctions/TransformFunctions.c \
	../../Drivers/CMSIS/DSP/Source/CommonTables/CommonTables.c \
	../../Drivers/CMSIS/DSP/Source/FastMathFunctions/FastMathFunctions.c \
	../../Drivers/CMSIS/DSP/Source/BasicMathFunctions/BasicMathFunctions.c \
	../../Drivers/CMSIS/DSP/Source/ComplexMathFunctions/ComplexMathFunctions.c \
	../../Drivers/CMSIS/DSP/Source/StatisticsFunctions/StatisticsFunctions.c \
	../../Drivers/CMSIS/DSP/Source/MatrixFunctions/MatrixFunctions.c
C_LIBS:=-lm

additive_synth:
	gcc -O2 -o Binaries/additive_synth.out \
	${C_INCLUDES} \
	${C_DEFINES} \
	${COMMON_SOURCE_FILES} \
	additive_synth_test.c \
	${C_LIBS}
