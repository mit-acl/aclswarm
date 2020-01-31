###########################################################################
## Makefile generated for MATLAB file/project 'ADMMGainDesign3D'. 
## 
## Makefile     : ADMMGainDesign3D_rtw.mk
## Generated on : Tue Jan 28 15:13:46 2020
## MATLAB Coder version: 4.1 (R2018b)
## 
## Build Info:
## 
## Final product: ./ADMMGainDesign3D.a
## Product type : static-library
## 
###########################################################################

###########################################################################
## MACROS
###########################################################################

# Macro Descriptions:
# PRODUCT_NAME            Name of the system to build
# MAKEFILE                Name of this makefile
# COMPUTER                Computer type. See the MATLAB "computer" command.

PRODUCT_NAME              = ADMMGainDesign3D
MAKEFILE                  = ADMMGainDesign3D_rtw.mk
COMPUTER                  = GLNXA64
MATLAB_ROOT               = /usr/local/MATLAB/R2018b
MATLAB_BIN                = /usr/local/MATLAB/R2018b/bin
MATLAB_ARCH_BIN           = $(MATLAB_BIN)/glnxa64
MASTER_ANCHOR_DIR         = 
START_DIR                 = /home/plusk01/dev/acl_ws/src/aclswarm/aclswarm/matlab/Helpers/codegen/lib/ADMMGainDesign3D
ARCH                      = glnxa64
RELATIVE_PATH_TO_ANCHOR   = .
C_STANDARD_OPTS           = -ansi -pedantic -Wno-long-long -fwrapv
CPP_STANDARD_OPTS         = -std=c++98 -pedantic -Wno-long-long -fwrapv

###########################################################################
## TOOLCHAIN SPECIFICATIONS
###########################################################################

# Toolchain Name:          GNU gcc/g++ | gmake (64-bit Linux)
# Supported Version(s):    
# ToolchainInfo Version:   R2018b
# Specification Revision:  1.0
# 
#-------------------------------------------
# Macros assumed to be defined elsewhere
#-------------------------------------------

# C_STANDARD_OPTS
# CPP_STANDARD_OPTS

#-----------
# MACROS
#-----------

WARN_FLAGS         = -Wall -W -Wwrite-strings -Winline -Wstrict-prototypes -Wnested-externs -Wpointer-arith -Wcast-align
WARN_FLAGS_MAX     = $(WARN_FLAGS) -Wcast-qual -Wshadow
CPP_WARN_FLAGS     = -Wall -W -Wwrite-strings -Winline -Wpointer-arith -Wcast-align
CPP_WARN_FLAGS_MAX = $(CPP_WARN_FLAGS) -Wcast-qual -Wshadow

TOOLCHAIN_SRCS = 
TOOLCHAIN_INCS = 
TOOLCHAIN_LIBS = 

#------------------------
# BUILD TOOL COMMANDS
#------------------------

# C Compiler: GNU C Compiler
CC = gcc

# Linker: GNU Linker
LD = g++

# C++ Compiler: GNU C++ Compiler
CPP = g++

# C++ Linker: GNU C++ Linker
CPP_LD = g++

# Archiver: GNU Archiver
AR = ar

# MEX Tool: MEX Tool
MEX_PATH = $(MATLAB_ARCH_BIN)
MEX = "$(MEX_PATH)/mex"

# Download: Download
DOWNLOAD =

# Execute: Execute
EXECUTE = $(PRODUCT)

# Builder: GMAKE Utility
MAKE_PATH = %MATLAB%/bin/glnxa64
MAKE = "$(MAKE_PATH)/gmake"


#-------------------------
# Directives/Utilities
#-------------------------

CDEBUG              = -g
C_OUTPUT_FLAG       = -o
LDDEBUG             = -g
OUTPUT_FLAG         = -o
CPPDEBUG            = -g
CPP_OUTPUT_FLAG     = -o
CPPLDDEBUG          = -g
OUTPUT_FLAG         = -o
ARDEBUG             =
STATICLIB_OUTPUT_FLAG =
MEX_DEBUG           = -g
RM                  = @rm -f
ECHO                = @echo
MV                  = @mv
RUN                 =

#----------------------------------------
# "Faster Builds" Build Configuration
#----------------------------------------

ARFLAGS              = ruvs
CFLAGS               = -c $(C_STANDARD_OPTS) -fPIC \
                       -O0
CPPFLAGS             = -c $(CPP_STANDARD_OPTS) -fPIC \
                       -O0
CPP_LDFLAGS          = -Wl,-rpath,"$(MATLAB_ARCH_BIN)",-L"$(MATLAB_ARCH_BIN)"
CPP_SHAREDLIB_LDFLAGS  = -shared -Wl,-rpath,"$(MATLAB_ARCH_BIN)",-L"$(MATLAB_ARCH_BIN)" -Wl,--no-undefined
DOWNLOAD_FLAGS       =
EXECUTE_FLAGS        =
LDFLAGS              = -Wl,-rpath,"$(MATLAB_ARCH_BIN)",-L"$(MATLAB_ARCH_BIN)"
MEX_CPPFLAGS         = -R2018a -MATLAB_ARCH=$(ARCH) $(INCLUDES) \
                         \
                       CXXOPTIMFLAGS="$(C_STANDARD_OPTS)  \
                       -O0 \
                        $(DEFINES)" \
                         \
                       -silent
MEX_CPPLDFLAGS       = LDFLAGS=='$$LDFLAGS'
MEX_CFLAGS           = -R2018a -MATLAB_ARCH=$(ARCH) $(INCLUDES) \
                         \
                       COPTIMFLAGS="$(C_STANDARD_OPTS)  \
                       -O0 \
                        $(DEFINES)" \
                         \
                       -silent
MEX_LDFLAGS          = LDFLAGS=='$$LDFLAGS'
MAKE_FLAGS           = -f $(MAKEFILE)
SHAREDLIB_LDFLAGS    = -shared -Wl,-rpath,"$(MATLAB_ARCH_BIN)",-L"$(MATLAB_ARCH_BIN)" -Wl,--no-undefined

#--------------------
# File extensions
#--------------------

H_EXT               = .h
OBJ_EXT             = .o
C_EXT               = .c
EXE_EXT             =
SHAREDLIB_EXT       = .so
HPP_EXT             = .hpp
OBJ_EXT             = .o
CPP_EXT             = .cpp
EXE_EXT             =
SHAREDLIB_EXT       = .so
STATICLIB_EXT       = .a
MEX_EXT             = .mexa64
MAKE_EXT            = .mk


###########################################################################
## OUTPUT INFO
###########################################################################

PRODUCT = ./ADMMGainDesign3D.a
PRODUCT_TYPE = "static-library"
BUILD_TYPE = "Static Library"

###########################################################################
## INCLUDE PATHS
###########################################################################

INCLUDES_BUILDINFO = -I$(START_DIR) -I/home/plusk01/dev/acl_ws/src/aclswarm/aclswarm/matlab/Helpers -I$(MATLAB_ROOT)/extern/include -I$(MATLAB_ROOT)/simulink/include -I$(MATLAB_ROOT)/rtw/c/src -I$(MATLAB_ROOT)/rtw/c/src/ext_mode/common -I$(MATLAB_ROOT)/rtw/c/ert -I$(START_DIR)/CXSparse/Include -I$(START_DIR)/CXSparse/SuiteSparse_config -I$(START_DIR)/CXSparse/CXSparseSupport

INCLUDES = $(INCLUDES_BUILDINFO)

###########################################################################
## DEFINES
###########################################################################

DEFINES_OPTS = -DCS_COMPLEX
DEFINES_STANDARD = -DMODEL=ADMMGainDesign3D -DHAVESTDIO -DUSE_RTMODEL -DUNIX

DEFINES = $(DEFINES_OPTS) $(DEFINES_STANDARD)

###########################################################################
## SOURCE FILES
###########################################################################

SRCS = $(START_DIR)/ADMMGainDesign3D_rtwutil.cpp $(START_DIR)/ADMMGainDesign3D_initialize.cpp $(START_DIR)/ADMMGainDesign3D_terminate.cpp $(START_DIR)/ADMMGainDesign3D.cpp $(START_DIR)/ADMMGainDesign2D.cpp $(START_DIR)/svd.cpp $(START_DIR)/anyNonFinite.cpp $(START_DIR)/svd1.cpp $(START_DIR)/xnrm2.cpp $(START_DIR)/sqrt.cpp $(START_DIR)/xscal.cpp $(START_DIR)/xdotc.cpp $(START_DIR)/xaxpy.cpp $(START_DIR)/xrotg.cpp $(START_DIR)/xrot.cpp $(START_DIR)/xswap.cpp $(START_DIR)/speye.cpp $(START_DIR)/sparse.cpp $(START_DIR)/reshape.cpp $(START_DIR)/diag.cpp $(START_DIR)/parenAssign2D.cpp $(START_DIR)/vertcat.cpp $(START_DIR)/horzcat.cpp $(START_DIR)/mtimes.cpp $(START_DIR)/parenReference2D.cpp $(START_DIR)/fillIn.cpp $(START_DIR)/binOp.cpp $(START_DIR)/introsort.cpp $(START_DIR)/insertionsort.cpp $(START_DIR)/sparse1.cpp $(START_DIR)/diag1.cpp $(START_DIR)/triu.cpp $(START_DIR)/heapsort.cpp $(START_DIR)/bigProduct.cpp $(START_DIR)/vec.cpp $(START_DIR)/CXSparseAPI.cpp $(START_DIR)/eig.cpp $(START_DIR)/schur.cpp $(START_DIR)/xzlarfg.cpp $(START_DIR)/xzlarf.cpp $(START_DIR)/xdhseqr.cpp $(START_DIR)/xdlanv2.cpp $(START_DIR)/xzggev.cpp $(START_DIR)/xzlascl.cpp $(START_DIR)/xzlartg.cpp $(START_DIR)/xzhgeqz.cpp $(START_DIR)/xztgevc.cpp $(START_DIR)/sum.cpp $(START_DIR)/locBsearch1.cpp $(START_DIR)/ADMMGainDesign3D_emxutil.cpp $(START_DIR)/ADMMGainDesign3D_emxAPI.cpp $(START_DIR)/rt_nonfinite.cpp $(START_DIR)/rtGetNaN.cpp $(START_DIR)/rtGetInf.cpp $(START_DIR)/CXSparse/Source/cs_add_ri.c $(START_DIR)/CXSparse/Source/cs_add_ci.c $(START_DIR)/CXSparse/Source/cs_amd_ri.c $(START_DIR)/CXSparse/Source/cs_amd_ci.c $(START_DIR)/CXSparse/Source/cs_chol_ri.c $(START_DIR)/CXSparse/Source/cs_chol_ci.c $(START_DIR)/CXSparse/Source/cs_cholsol_ri.c $(START_DIR)/CXSparse/Source/cs_cholsol_ci.c $(START_DIR)/CXSparse/Source/cs_counts_ri.c $(START_DIR)/CXSparse/Source/cs_counts_ci.c $(START_DIR)/CXSparse/Source/cs_cumsum_ri.c $(START_DIR)/CXSparse/Source/cs_cumsum_ci.c $(START_DIR)/CXSparse/Source/cs_dfs_ri.c $(START_DIR)/CXSparse/Source/cs_dfs_ci.c $(START_DIR)/CXSparse/Source/cs_dmperm_ri.c $(START_DIR)/CXSparse/Source/cs_dmperm_ci.c $(START_DIR)/CXSparse/Source/cs_droptol_ri.c $(START_DIR)/CXSparse/Source/cs_droptol_ci.c $(START_DIR)/CXSparse/Source/cs_dropzeros_ri.c $(START_DIR)/CXSparse/Source/cs_dropzeros_ci.c $(START_DIR)/CXSparse/Source/cs_dupl_ri.c $(START_DIR)/CXSparse/Source/cs_dupl_ci.c $(START_DIR)/CXSparse/Source/cs_entry_ri.c $(START_DIR)/CXSparse/Source/cs_entry_ci.c $(START_DIR)/CXSparse/Source/cs_etree_ri.c $(START_DIR)/CXSparse/Source/cs_etree_ci.c $(START_DIR)/CXSparse/Source/cs_fkeep_ri.c $(START_DIR)/CXSparse/Source/cs_fkeep_ci.c $(START_DIR)/CXSparse/Source/cs_gaxpy_ri.c $(START_DIR)/CXSparse/Source/cs_gaxpy_ci.c $(START_DIR)/CXSparse/Source/cs_happly_ri.c $(START_DIR)/CXSparse/Source/cs_happly_ci.c $(START_DIR)/CXSparse/Source/cs_house_ri.c $(START_DIR)/CXSparse/Source/cs_house_ci.c $(START_DIR)/CXSparse/Source/cs_ipvec_ri.c $(START_DIR)/CXSparse/Source/cs_ipvec_ci.c $(START_DIR)/CXSparse/Source/cs_load_ri.c $(START_DIR)/CXSparse/Source/cs_load_ci.c $(START_DIR)/CXSparse/Source/cs_lsolve_ri.c $(START_DIR)/CXSparse/Source/cs_lsolve_ci.c $(START_DIR)/CXSparse/Source/cs_ltsolve_ri.c $(START_DIR)/CXSparse/Source/cs_ltsolve_ci.c $(START_DIR)/CXSparse/Source/cs_lu_ri.c $(START_DIR)/CXSparse/Source/cs_lu_ci.c $(START_DIR)/CXSparse/Source/cs_lusol_ri.c $(START_DIR)/CXSparse/Source/cs_lusol_ci.c $(START_DIR)/CXSparse/Source/cs_malloc_ri.c $(START_DIR)/CXSparse/Source/cs_malloc_ci.c $(START_DIR)/CXSparse/Source/cs_maxtrans_ri.c $(START_DIR)/CXSparse/Source/cs_maxtrans_ci.c $(START_DIR)/CXSparse/Source/cs_multiply_ri.c $(START_DIR)/CXSparse/Source/cs_multiply_ci.c $(START_DIR)/CXSparse/Source/cs_norm_ri.c $(START_DIR)/CXSparse/Source/cs_norm_ci.c $(START_DIR)/CXSparse/Source/cs_permute_ri.c $(START_DIR)/CXSparse/Source/cs_permute_ci.c $(START_DIR)/CXSparse/Source/cs_pinv_ri.c $(START_DIR)/CXSparse/Source/cs_pinv_ci.c $(START_DIR)/CXSparse/Source/cs_post_ri.c $(START_DIR)/CXSparse/Source/cs_post_ci.c $(START_DIR)/CXSparse/Source/cs_print_ri.c $(START_DIR)/CXSparse/Source/cs_print_ci.c $(START_DIR)/CXSparse/Source/cs_pvec_ri.c $(START_DIR)/CXSparse/Source/cs_pvec_ci.c $(START_DIR)/CXSparse/Source/cs_qr_ri.c $(START_DIR)/CXSparse/Source/cs_qr_ci.c $(START_DIR)/CXSparse/Source/cs_qrsol_ri.c $(START_DIR)/CXSparse/Source/cs_qrsol_ci.c $(START_DIR)/CXSparse/Source/cs_scatter_ri.c $(START_DIR)/CXSparse/Source/cs_scatter_ci.c $(START_DIR)/CXSparse/Source/cs_scc_ri.c $(START_DIR)/CXSparse/Source/cs_scc_ci.c $(START_DIR)/CXSparse/Source/cs_schol_ri.c $(START_DIR)/CXSparse/Source/cs_schol_ci.c $(START_DIR)/CXSparse/Source/cs_sqr_ri.c $(START_DIR)/CXSparse/Source/cs_sqr_ci.c $(START_DIR)/CXSparse/Source/cs_symperm_ri.c $(START_DIR)/CXSparse/Source/cs_symperm_ci.c $(START_DIR)/CXSparse/Source/cs_tdfs_ri.c $(START_DIR)/CXSparse/Source/cs_tdfs_ci.c $(START_DIR)/CXSparse/Source/cs_transpose_ri.c $(START_DIR)/CXSparse/Source/cs_transpose_ci.c $(START_DIR)/CXSparse/Source/cs_compress_ri.c $(START_DIR)/CXSparse/Source/cs_compress_ci.c $(START_DIR)/CXSparse/Source/cs_updown_ri.c $(START_DIR)/CXSparse/Source/cs_updown_ci.c $(START_DIR)/CXSparse/Source/cs_usolve_ri.c $(START_DIR)/CXSparse/Source/cs_usolve_ci.c $(START_DIR)/CXSparse/Source/cs_utsolve_ri.c $(START_DIR)/CXSparse/Source/cs_utsolve_ci.c $(START_DIR)/CXSparse/Source/cs_util_ri.c $(START_DIR)/CXSparse/Source/cs_util_ci.c $(START_DIR)/CXSparse/Source/cs_reach_ri.c $(START_DIR)/CXSparse/Source/cs_reach_ci.c $(START_DIR)/CXSparse/Source/cs_spsolve_ri.c $(START_DIR)/CXSparse/Source/cs_spsolve_ci.c $(START_DIR)/CXSparse/Source/cs_ereach_ri.c $(START_DIR)/CXSparse/Source/cs_ereach_ci.c $(START_DIR)/CXSparse/Source/cs_leaf_ri.c $(START_DIR)/CXSparse/Source/cs_leaf_ci.c $(START_DIR)/CXSparse/Source/cs_randperm_ri.c $(START_DIR)/CXSparse/Source/cs_randperm_ci.c $(START_DIR)/CXSparse/Source/cs_operator_ri.c $(START_DIR)/CXSparse/Source/cs_operator_ci.c $(START_DIR)/CXSparse/CXSparseSupport/solve_from_lu.c $(START_DIR)/CXSparse/CXSparseSupport/solve_from_qr.c $(START_DIR)/CXSparse/CXSparseSupport/makeCXSparseMatrix.c

ALL_SRCS = $(SRCS)

###########################################################################
## OBJECTS
###########################################################################

OBJS = ADMMGainDesign3D_rtwutil.o ADMMGainDesign3D_initialize.o ADMMGainDesign3D_terminate.o ADMMGainDesign3D.o ADMMGainDesign2D.o svd.o anyNonFinite.o svd1.o xnrm2.o sqrt.o xscal.o xdotc.o xaxpy.o xrotg.o xrot.o xswap.o speye.o sparse.o reshape.o diag.o parenAssign2D.o vertcat.o horzcat.o mtimes.o parenReference2D.o fillIn.o binOp.o introsort.o insertionsort.o sparse1.o diag1.o triu.o heapsort.o bigProduct.o vec.o CXSparseAPI.o eig.o schur.o xzlarfg.o xzlarf.o xdhseqr.o xdlanv2.o xzggev.o xzlascl.o xzlartg.o xzhgeqz.o xztgevc.o sum.o locBsearch1.o ADMMGainDesign3D_emxutil.o ADMMGainDesign3D_emxAPI.o rt_nonfinite.o rtGetNaN.o rtGetInf.o cs_add_ri.o cs_add_ci.o cs_amd_ri.o cs_amd_ci.o cs_chol_ri.o cs_chol_ci.o cs_cholsol_ri.o cs_cholsol_ci.o cs_counts_ri.o cs_counts_ci.o cs_cumsum_ri.o cs_cumsum_ci.o cs_dfs_ri.o cs_dfs_ci.o cs_dmperm_ri.o cs_dmperm_ci.o cs_droptol_ri.o cs_droptol_ci.o cs_dropzeros_ri.o cs_dropzeros_ci.o cs_dupl_ri.o cs_dupl_ci.o cs_entry_ri.o cs_entry_ci.o cs_etree_ri.o cs_etree_ci.o cs_fkeep_ri.o cs_fkeep_ci.o cs_gaxpy_ri.o cs_gaxpy_ci.o cs_happly_ri.o cs_happly_ci.o cs_house_ri.o cs_house_ci.o cs_ipvec_ri.o cs_ipvec_ci.o cs_load_ri.o cs_load_ci.o cs_lsolve_ri.o cs_lsolve_ci.o cs_ltsolve_ri.o cs_ltsolve_ci.o cs_lu_ri.o cs_lu_ci.o cs_lusol_ri.o cs_lusol_ci.o cs_malloc_ri.o cs_malloc_ci.o cs_maxtrans_ri.o cs_maxtrans_ci.o cs_multiply_ri.o cs_multiply_ci.o cs_norm_ri.o cs_norm_ci.o cs_permute_ri.o cs_permute_ci.o cs_pinv_ri.o cs_pinv_ci.o cs_post_ri.o cs_post_ci.o cs_print_ri.o cs_print_ci.o cs_pvec_ri.o cs_pvec_ci.o cs_qr_ri.o cs_qr_ci.o cs_qrsol_ri.o cs_qrsol_ci.o cs_scatter_ri.o cs_scatter_ci.o cs_scc_ri.o cs_scc_ci.o cs_schol_ri.o cs_schol_ci.o cs_sqr_ri.o cs_sqr_ci.o cs_symperm_ri.o cs_symperm_ci.o cs_tdfs_ri.o cs_tdfs_ci.o cs_transpose_ri.o cs_transpose_ci.o cs_compress_ri.o cs_compress_ci.o cs_updown_ri.o cs_updown_ci.o cs_usolve_ri.o cs_usolve_ci.o cs_utsolve_ri.o cs_utsolve_ci.o cs_util_ri.o cs_util_ci.o cs_reach_ri.o cs_reach_ci.o cs_spsolve_ri.o cs_spsolve_ci.o cs_ereach_ri.o cs_ereach_ci.o cs_leaf_ri.o cs_leaf_ci.o cs_randperm_ri.o cs_randperm_ci.o cs_operator_ri.o cs_operator_ci.o solve_from_lu.o solve_from_qr.o makeCXSparseMatrix.o

ALL_OBJS = $(OBJS)

###########################################################################
## PREBUILT OBJECT FILES
###########################################################################

PREBUILT_OBJS = 

###########################################################################
## LIBRARIES
###########################################################################

LIBS = 

###########################################################################
## SYSTEM LIBRARIES
###########################################################################

SYSTEM_LIBS =  -lm -lstdc++

###########################################################################
## ADDITIONAL TOOLCHAIN FLAGS
###########################################################################

#---------------
# C Compiler
#---------------

CFLAGS_BASIC = $(DEFINES) $(INCLUDES)

CFLAGS += $(CFLAGS_BASIC)

#-----------------
# C++ Compiler
#-----------------

CPPFLAGS_BASIC = $(DEFINES) $(INCLUDES)

CPPFLAGS += $(CPPFLAGS_BASIC)

###########################################################################
## INLINED COMMANDS
###########################################################################

###########################################################################
## PHONY TARGETS
###########################################################################

.PHONY : all build clean info prebuild download execute


all : build
	@echo "### Successfully generated all binary outputs."


build : prebuild $(PRODUCT)


prebuild : 


download : build


execute : download


###########################################################################
## FINAL TARGET
###########################################################################

#---------------------------------
# Create a static library         
#---------------------------------

$(PRODUCT) : $(OBJS) $(PREBUILT_OBJS)
	@echo "### Creating static library "$(PRODUCT)" ..."
	$(AR) $(ARFLAGS)  $(PRODUCT) $(OBJS)
	@echo "### Created: $(PRODUCT)"


###########################################################################
## INTERMEDIATE TARGETS
###########################################################################

#---------------------
# SOURCE-TO-OBJECT
#---------------------

%.o : %.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.o : %.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(RELATIVE_PATH_TO_ANCHOR)/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.o : $(RELATIVE_PATH_TO_ANCHOR)/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/rtw/c/src/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/rtw/c/src/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/CXSparse/CXSparseSupport/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/CXSparse/CXSparseSupport/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/CXSparse/Source/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/CXSparse/Source/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : /home/plusk01/dev/acl_ws/src/aclswarm/aclswarm/matlab/Helpers/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.o : /home/plusk01/dev/acl_ws/src/aclswarm/aclswarm/matlab/Helpers/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


###########################################################################
## DEPENDENCIES
###########################################################################

$(ALL_OBJS) : rtw_proj.tmw $(MAKEFILE)


###########################################################################
## MISCELLANEOUS TARGETS
###########################################################################

info : 
	@echo "### PRODUCT = $(PRODUCT)"
	@echo "### PRODUCT_TYPE = $(PRODUCT_TYPE)"
	@echo "### BUILD_TYPE = $(BUILD_TYPE)"
	@echo "### INCLUDES = $(INCLUDES)"
	@echo "### DEFINES = $(DEFINES)"
	@echo "### ALL_SRCS = $(ALL_SRCS)"
	@echo "### ALL_OBJS = $(ALL_OBJS)"
	@echo "### LIBS = $(LIBS)"
	@echo "### MODELREF_LIBS = $(MODELREF_LIBS)"
	@echo "### SYSTEM_LIBS = $(SYSTEM_LIBS)"
	@echo "### TOOLCHAIN_LIBS = $(TOOLCHAIN_LIBS)"
	@echo "### CFLAGS = $(CFLAGS)"
	@echo "### LDFLAGS = $(LDFLAGS)"
	@echo "### SHAREDLIB_LDFLAGS = $(SHAREDLIB_LDFLAGS)"
	@echo "### CPPFLAGS = $(CPPFLAGS)"
	@echo "### CPP_LDFLAGS = $(CPP_LDFLAGS)"
	@echo "### CPP_SHAREDLIB_LDFLAGS = $(CPP_SHAREDLIB_LDFLAGS)"
	@echo "### ARFLAGS = $(ARFLAGS)"
	@echo "### MEX_CFLAGS = $(MEX_CFLAGS)"
	@echo "### MEX_CPPFLAGS = $(MEX_CPPFLAGS)"
	@echo "### MEX_LDFLAGS = $(MEX_LDFLAGS)"
	@echo "### MEX_CPPLDFLAGS = $(MEX_CPPLDFLAGS)"
	@echo "### DOWNLOAD_FLAGS = $(DOWNLOAD_FLAGS)"
	@echo "### EXECUTE_FLAGS = $(EXECUTE_FLAGS)"
	@echo "### MAKE_FLAGS = $(MAKE_FLAGS)"


clean : 
	$(ECHO) "### Deleting all derived files..."
	$(RM) $(PRODUCT)
	$(RM) $(ALL_OBJS)
	$(ECHO) "### Deleted all derived files."


