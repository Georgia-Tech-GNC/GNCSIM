#
# Copyright (c) The acados authors.
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



cmake_minimum_required(VERSION 3.13)

project(nomtraj_aug)

# build options.

option(BUILD_ACADOS_SOLVER_LIB "Should the solver library acados_solver_nomtraj_aug be build?" OFF)
option(BUILD_ACADOS_OCP_SOLVER_LIB "Should the OCP solver library acados_ocp_solver_nomtraj_aug be build?" OFF)
option(BUILD_ACADOS_OCP_SOLVER_STATIC_LIB "Should the static OCP solver library acados_ocp_solver_nomtraj_aug_static be built?" OFF)
option(BUILD_EXAMPLE "Should the example main_nomtraj_aug be build?" OFF)
option(BUILD_EXAMPLE_STATIC "Should the example main_nomtraj_aug be build linking to the static OCP solver library?" OFF)


if(CMAKE_CXX_COMPILER_ID MATCHES "GNU" AND CMAKE_SYSTEM_NAME MATCHES "Windows")
    # MinGW: remove prefix and change suffix to match MSVC
    # (such that MATLAB mex recognizes the libraries)
    set(CMAKE_SHARED_LIBRARY_PREFIX "")
    set(CMAKE_IMPORT_LIBRARY_SUFFIX ".lib")
    set(CMAKE_IMPORT_LIBRARY_PREFIX "")
    set(CMAKE_STATIC_LIBRARY_SUFFIX ".lib")
    set(CMAKE_STATIC_LIBRARY_PREFIX "")
endif()


if(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE C:/Users/danil/OneDrive/Documents/GitHub/GNCSIM/src/c_generated_code)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE C:/Users/danil/OneDrive/Documents/GitHub/GNCSIM/src/c_generated_code)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE C:/Users/danil/OneDrive/Documents/GitHub/GNCSIM/src/c_generated_code)
endif()


# object target names
set(MODEL_OBJ model_nomtraj_aug)
set(OCP_OBJ ocp_nomtraj_aug)
# model
set(MODEL_SRC
    nomtraj_aug_model/nomtraj_aug_dyn_disc_phi_fun.c
    nomtraj_aug_model/nomtraj_aug_dyn_disc_phi_fun_jac.c
)
add_library(${MODEL_OBJ} OBJECT ${MODEL_SRC} )


# optimal control problem - mostly CasADi exports
if(${BUILD_ACADOS_SOLVER_LIB} OR ${BUILD_ACADOS_OCP_SOLVER_LIB} OR ${BUILD_ACADOS_OCP_SOLVER_STATIC_LIB} OR ${BUILD_EXAMPLE})
    set(OCP_SRC

        nomtraj_aug_cost/nomtraj_aug_cost_y_0_fun.c
        nomtraj_aug_cost/nomtraj_aug_cost_y_0_fun_jac_ut_xt.c
        nomtraj_aug_cost/nomtraj_aug_cost_y_fun.c
        nomtraj_aug_cost/nomtraj_aug_cost_y_fun_jac_ut_xt.c
        nomtraj_aug_cost/nomtraj_aug_cost_y_e_fun.c
        nomtraj_aug_cost/nomtraj_aug_cost_y_e_fun_jac_ut_xt.c
        acados_solver_nomtraj_aug.c
    )
    add_library(${OCP_OBJ} OBJECT ${OCP_SRC})
endif()
# for target example
set(EX_SRC main_nomtraj_aug.c)
set(EX_EXE main_nomtraj_aug)
set(EXTERNAL_DIR)
set(EXTERNAL_LIB)

# set some search paths for preprocessor and linker
set(ACADOS_INCLUDE_PATH C:/dev/acados/include CACHE PATH "Define the path which contains the include directory for acados.")
set(ACADOS_LIB_PATH C:/dev/acados/lib CACHE PATH "Define the path which contains the lib directory for acados.")

# c-compiler flags for debugging
set(CMAKE_C_FLAGS_DEBUG "-O0 -ggdb")

set(CMAKE_C_FLAGS "-fPIC -std=c99  -O2  ")
#-fno-diagnostics-show-line-numbers -g

set(ACADOS_INCLUDE_PATHS
   ${ACADOS_INCLUDE_PATH}
   ${ACADOS_INCLUDE_PATH}/acados
   ${ACADOS_INCLUDE_PATH}/blasfeo/include
   ${ACADOS_INCLUDE_PATH}/hpipm/include
)

include_directories(${ACADOS_INCLUDE_PATHS})

# linker flags
link_directories(${ACADOS_LIB_PATH})

# link to libraries
if(UNIX)
    set(ACADOS_LINK_LIBS acados hpipm blasfeo m      CACHE STRING "Define what libs to link with acados solver")
else()
    set(ACADOS_LINK_LIBS acados hpipm blasfeo      CACHE STRING "Define what libs to link with acados solver")
endif()

link_libraries(${ACADOS_LINK_LIBS})

# the targets
# bundled_shared_lib
if(${BUILD_ACADOS_SOLVER_LIB})
    set(LIB_ACADOS_SOLVER acados_solver_nomtraj_aug)
    add_library(${LIB_ACADOS_SOLVER} SHARED
    $<TARGET_OBJECTS:${MODEL_OBJ}>
    $<TARGET_OBJECTS:${OCP_OBJ}>)
    install(TARGETS ${LIB_ACADOS_SOLVER} DESTINATION ${CMAKE_INSTALL_PREFIX})
endif(${BUILD_ACADOS_SOLVER_LIB})

# ocp_shared_lib
if(${BUILD_ACADOS_OCP_SOLVER_LIB})
    set(LIB_ACADOS_OCP_SOLVER acados_ocp_solver_nomtraj_aug)
    add_library(${LIB_ACADOS_OCP_SOLVER} SHARED
    $<TARGET_OBJECTS:${MODEL_OBJ}>
    $<TARGET_OBJECTS:${OCP_OBJ}>)
    # Specify libraries or flags to use when linking a given target and/or its dependents.
    target_link_libraries(${LIB_ACADOS_OCP_SOLVER} PRIVATE ${EXTERNAL_LIB})
    target_link_directories(${LIB_ACADOS_OCP_SOLVER} PRIVATE ${EXTERNAL_DIR})
    install(TARGETS ${LIB_ACADOS_OCP_SOLVER} DESTINATION ${CMAKE_INSTALL_PREFIX})
endif(${BUILD_ACADOS_OCP_SOLVER_LIB})

# ocp_static_lib
if(${BUILD_ACADOS_OCP_SOLVER_STATIC_LIB})
    set(LIB_ACADOS_OCP_SOLVER_STATIC acados_ocp_solver_nomtraj_aug_static)
    add_library(${LIB_ACADOS_OCP_SOLVER_STATIC} STATIC
        $<TARGET_OBJECTS:${MODEL_OBJ}>
        $<TARGET_OBJECTS:${OCP_OBJ}>
    )
    target_link_libraries(${LIB_ACADOS_OCP_SOLVER_STATIC} PRIVATE ${EXTERNAL_LIB})
    target_link_directories(${LIB_ACADOS_OCP_SOLVER_STATIC} PRIVATE ${EXTERNAL_DIR})
    install(TARGETS ${LIB_ACADOS_OCP_SOLVER_STATIC} DESTINATION ${CMAKE_INSTALL_PREFIX})
endif(${BUILD_ACADOS_OCP_SOLVER_STATIC_LIB})

# example
if(${BUILD_EXAMPLE})
    add_executable(${EX_EXE} ${EX_SRC}
    $<TARGET_OBJECTS:${MODEL_OBJ}>
    $<TARGET_OBJECTS:${OCP_OBJ}>)
    install(TARGETS ${EX_EXE} DESTINATION ${CMAKE_INSTALL_PREFIX})
endif(${BUILD_EXAMPLE})

# example linking to static lib
if(${BUILD_EXAMPLE_STATIC})
    add_executable(${EX_EXE} ${EX_SRC}
    )
    install(TARGETS ${EX_EXE} DESTINATION ${CMAKE_INSTALL_PREFIX})
    target_link_libraries(${EX_EXE} PRIVATE ${LIB_ACADOS_OCP_SOLVER_STATIC} ${EXTERNAL_LIB})
endif(${BUILD_EXAMPLE_STATIC})



# unset options for clean cmake cache on subsequent cmake runs
unset(BUILD_ACADOS_SOLVER_LIB CACHE)
unset(BUILD_ACADOS_OCP_SOLVER_LIB CACHE)
unset(BUILD_ACADOS_OCP_SOLVER_STATIC_LIB CACHE)
unset(BUILD_EXAMPLE CACHE)
unset(BUILD_EXAMPLE_STATIC CACHE)
