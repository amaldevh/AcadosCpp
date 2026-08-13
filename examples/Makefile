ACADOS_ROOT=$(shell echo $$ACADOS_ROOT)
LDFLAGS=
LDFLAGS+= -L $(shell pwd)/cpp_quadrotor_ocp
LDFLAGS+= -L$(ACADOS_ROOT)/lib
LDFLAGS+= -Wl,--disable-new-dtags,-rpath,$(shell pwd)/cpp_quadrotor_ocp
LDFLAGS+= -Wl,--disable-new-dtags,-rpath,$(ACADOS_ROOT)/lib
LDLIBS=
LDLIBS+= -lquadrotor_ocp -lacados_solver_quadrotor -lquadrotor_sim
LDLIBS+= -lacados
LDLIBS+= -lhpipm -lblasfeo -lm


CXXFLAGS = -fPIC -std=c++17 -O3
CXXFLAGS+= -I$(shell pwd)/cpp_quadrotor_ocp
CXXFLAGS+= -I$(ACADOS_ROOT)/include
CXXFLAGS+= -I$(ACADOS_ROOT)/include/acados
CXXFLAGS+= -I$(ACADOS_ROOT)/include/blasfeo/include
CXXFLAGS+= -I$(ACADOS_ROOT)/include/hpipm/include

# compile control loop
control_loop: cpp_control_loop.cc
	$(CXX) cpp_control_loop.cc -o control_loop $(CXXFLAGS) $(LDFLAGS) $(LDLIBS)
