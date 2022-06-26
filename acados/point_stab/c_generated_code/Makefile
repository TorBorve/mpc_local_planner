#
# Copyright 2019 Gianluca Frison, Dimitris Kouzoupis, Robin Verschueren,
# Andrea Zanelli, Niels van Duijkeren, Jonathan Frey, Tommaso Sartor,
# Branimir Novoselnik, Rien Quirynen, Rezart Qelibari, Dang Doan,
# Jonas Koenemann, Yutao Chen, Tobias Schöls, Jonas Schlagenhauf, Moritz Diehl
#
# This file is part of acados.
#
# The 2-Clause BSD License
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.;
#





# define sources and use make's implicit rules to generate object files (*.o)

# model
MODEL_SRC=
MODEL_SRC+= point_stab_model/point_stab_expl_ode_fun.c
MODEL_SRC+= point_stab_model/point_stab_expl_vde_forw.c
MODEL_OBJ := $(MODEL_SRC:.c=.o)

# optimal control problem - mostly CasADi exports
OCP_SRC=
OCP_SRC+= point_stab_constraints/point_stab_constr_h_fun_jac_uxt_zt.c
OCP_SRC+= point_stab_constraints/point_stab_constr_h_fun.c
OCP_SRC+= point_stab_cost/point_stab_cost_y_0_fun.c
OCP_SRC+= point_stab_cost/point_stab_cost_y_0_fun_jac_ut_xt.c
OCP_SRC+= point_stab_cost/point_stab_cost_y_0_hess.c
OCP_SRC+= point_stab_cost/point_stab_cost_y_fun.c
OCP_SRC+= point_stab_cost/point_stab_cost_y_fun_jac_ut_xt.c
OCP_SRC+= point_stab_cost/point_stab_cost_y_hess.c
OCP_SRC+= acados_solver_point_stab.c
OCP_OBJ := $(OCP_SRC:.c=.o)

# for sim solver
SIM_SRC= acados_sim_solver_point_stab.c
SIM_OBJ := $(SIM_SRC:.c=.o)

# for target example
EX_SRC= main_point_stab.c
EX_OBJ := $(EX_SRC:.c=.o)
EX_EXE := $(EX_SRC:.c=)

# for target example_sim
EX_SIM_SRC= main_sim_point_stab.c
EX_SIM_OBJ := $(EX_SIM_SRC:.c=.o)
EX_SIM_EXE := $(EX_SIM_SRC:.c=)

# combine model, sim and ocp object files
OBJ=
OBJ+= $(MODEL_OBJ)
OBJ+= $(SIM_OBJ)
OBJ+= $(OCP_OBJ)

EXTERNAL_DIR=
EXTERNAL_LIB=

INCLUDE_PATH = /home/fuelfighter/acados/acados/include
LIB_PATH = /home/fuelfighter/acados/acados/lib

# preprocessor flags for make's implicit rules
CPPFLAGS+= -I$(INCLUDE_PATH)
CPPFLAGS+= -I$(INCLUDE_PATH)/acados
CPPFLAGS+= -I$(INCLUDE_PATH)/blasfeo/include
CPPFLAGS+= -I$(INCLUDE_PATH)/hpipm/include


# define the c-compiler flags for make's implicit rules
CFLAGS = -fPIC -std=c99   #-fno-diagnostics-show-line-numbers -g
# # Debugging
# CFLAGS += -g3

# linker flags
LDFLAGS+= -L$(LIB_PATH)

# link to libraries
LDLIBS+= -lacados
LDLIBS+= -lhpipm
LDLIBS+= -lblasfeo
LDLIBS+= -lm
LDLIBS+= 

# libraries
LIBACADOS_SOLVER=libacados_solver_point_stab.so
LIBACADOS_OCP_SOLVER=libacados_ocp_solver_point_stab.so
LIBACADOS_SIM_SOLVER=lib$(SIM_SRC:.c=.so)

# virtual targets
.PHONY : all clean

#all: clean example_sim example shared_lib

all: clean example_sim example
shared_lib: bundled_shared_lib ocp_shared_lib sim_shared_lib

# some linker targets
example: $(EX_OBJ) $(OBJ)
	$(CC) $^ -o $(EX_EXE) $(LDFLAGS) $(LDLIBS)

example_sim: $(EX_SIM_OBJ) $(MODEL_OBJ) $(SIM_OBJ)
	$(CC) $^ -o $(EX_SIM_EXE) $(LDFLAGS) $(LDLIBS)

bundled_shared_lib: $(OBJ)
	$(CC) -shared $^ -o $(LIBACADOS_SOLVER) $(LDFLAGS) $(LDLIBS)

ocp_shared_lib: $(OCP_OBJ) $(MODEL_OBJ)
	$(CC) -shared $^ -o $(LIBACADOS_OCP_SOLVER) $(LDFLAGS) $(LDLIBS) \
	-L$(EXTERNAL_DIR) -l$(EXTERNAL_LIB)

sim_shared_lib: $(SIM_OBJ) $(MODEL_OBJ)
	$(CC) -shared $^ -o $(LIBACADOS_SIM_SOLVER) $(LDFLAGS) $(LDLIBS)


# Cython targets
ocp_cython_c: ocp_shared_lib
	cython \
	-o acados_ocp_solver_pyx.c \
	-I $(INCLUDE_PATH)/../interfaces/acados_template/acados_template \
	$(INCLUDE_PATH)/../interfaces/acados_template/acados_template/acados_ocp_solver_pyx.pyx \
	-I /home/fuelfighter/catkin_ws/src/mpc_local_planner/acados/point_stab/c_generated_code \

ocp_cython_o: ocp_cython_c
	$(CC) $(ACADOS_FLAGS) -c -O2 \
	-fPIC \
	-o acados_ocp_solver_pyx.o \
	-I /usr/include/python3.8 \
	-I $(INCLUDE_PATH)/blasfeo/include/ \
	-I $(INCLUDE_PATH)/hpipm/include/ \
	-I $(INCLUDE_PATH) \
	-I /home/fuelfighter/acados/acados/build/env/lib/python3.8/site-packages/numpy/core/include \
	acados_ocp_solver_pyx.c \

ocp_cython: ocp_cython_o
	$(CC) $(ACADOS_FLAGS) -shared \
	-o acados_ocp_solver_pyx.so \
	-Wl,-rpath=$(LIB_PATH) \
	acados_ocp_solver_pyx.o \
	$(abspath .)/libacados_ocp_solver_point_stab.so \
	$(LDFLAGS) $(LDLIBS)

clean:
	$(RM) $(OBJ) $(EX_OBJ) $(EX_SIM_OBJ)
	$(RM) $(LIBACADOS_SOLVER) $(LIBACADOS_OCP_SOLVER) $(LIBACADOS_SIM_SOLVER)
	$(RM) $(EX_EXE) $(EX_SIM_EXE)

clean_ocp_shared_lib:
	$(RM) $(LIBACADOS_OCP_SOLVER)
	$(RM) $(OCP_OBJ)

clean_ocp_cython:
	$(RM) libacados_ocp_solver_point_stab.so
	$(RM) acados_solver_point_stab.o
	$(RM) acados_ocp_solver_pyx.so
	$(RM) acados_ocp_solver_pyx.o
