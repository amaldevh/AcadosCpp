import os
from jinja2 import Environment, FileSystemLoader
import glob
import argparse
import pybind11
import sysconfig 
import subprocess

def generate_code(model_name, c_generated_code_dir, output_dir):
    if not os.path.isdir(c_generated_code_dir):
        raise ValueError(f"The specified C generated code directory does not exist: {c_generated_code_dir}")
    # Define the template directory and load the template
    template_dir = os.path.join(os.path.dirname(__file__), 'templates')
    env = Environment(loader=FileSystemLoader(template_dir))
    template = env.get_template('model_ocp.cc.j2')

    # Render the template with the provided class name
    rendered_code = template.render(model_name=model_name)

    # Ensure the output directory exists
    os.makedirs(output_dir, exist_ok=True)

    # Write the rendered code to a file
    output_file_path = os.path.join(output_dir, f'model_ocp.cc')
    with open(output_file_path, 'w') as output_file:
        output_file.write(rendered_code)

    # Write header
    template = env.get_template('model_ocp.hh.j2')

    rendered_code = template.render(model_name=model_name, out_file=f'model_ocp.cc')
    header_path = os.path.join(output_dir, f'model_ocp.hh')
    with open(header_path, 'w') as header_file:
        header_file.write(rendered_code)
    
    # Write python extension
    template = env.get_template('model_ocp_py.cc.j2')
    rendered_code = template.render(model_name=model_name, out_file=f'model_ocp.cc')
    py_extension_path =  os.path.join(output_dir, f'model_ocp_py.cc')
    with open(py_extension_path, 'w') as py_extension_file:
        py_extension_file.write(rendered_code)

    # Generate sim files
    # Write the rendered code to a file
    template = env.get_template('model_sim.cc.j2')
    rendered_code = template.render(model_name=model_name)
    output_file_path = os.path.join(output_dir, f'model_sim.cc')
    with open(output_file_path, 'w') as output_file:
        output_file.write(rendered_code)
    # Write header
    template = env.get_template('model_sim.hh.j2')
    rendered_code = template.render(model_name=model_name, out_file=f'model_sim.cc')
    header_path = os.path.join(output_dir, f'model_sim.hh')
    with open(header_path, 'w') as header_file:
        header_file.write(rendered_code)
    # Write python extension
    template = env.get_template('model_sim_py.cc.j2')
    rendered_code = template.render(model_name=model_name, out_file=f'model_sim.cc')
    py_extension_path =  os.path.join(output_dir, f'model_sim_py.cc')
    with open(py_extension_path, 'w') as py_extension_file:
        py_extension_file.write(rendered_code)

    # copy all source and headers
    src_files = glob.glob(os.path.join(c_generated_code_dir, '*.c'))
    header_files = glob.glob(os.path.join(c_generated_code_dir, '*.h'))
    model_dir = os.path.join(c_generated_code_dir, f'{model_name}_model')
    cost_dir = os.path.join(c_generated_code_dir, f'{model_name}_cost')

    # assert that model_dir and cost_dir exist
    if not os.path.isdir(model_dir):
        raise ValueError(f"The specified model directory does not exist: {model_dir}")
    if not os.path.isdir(cost_dir):
        raise ValueError(f"The specified cost directory does not exist: {cost_dir}")
    
    # assert length of src_files and header_files is greater than 0
    if len(src_files) == 0:
        raise ValueError(f"No source files found in the specified C generated code directory: {c_generated_code_dir}")
    if len(header_files) == 0:
        raise ValueError(f"No header files found in the specified C generated code directory: {c_generated_code_dir}")
    os.system(f'cp {" ".join(src_files)} {output_dir}/')
    os.system(f'cp {" ".join(header_files)} {output_dir}/')
    os.system(f'cp -r {model_dir} {output_dir}/')
    os.system(f'cp -r {cost_dir} {output_dir}/')
    print(f"Generated OCP code and Makefile for model '{model_name}' in '{output_dir}'")
    print("Now building the shared library...")
    build(model_name, output_dir)

def get_python_flags():
    all_flags = subprocess.check_output(['python3-config', '--cflags', '--ldflags']).decode('utf-8')
    extension_suffix = sysconfig.get_config_var('EXT_SUFFIX')
    return all_flags, extension_suffix

def build(model_name, dest):
    
    if not os.path.isdir(dest):
        raise ValueError(f"The specified destination directory does not exist: {dest}")
    os.chdir(dest)
    acados_root = os.environ.get('ACADOS_ROOT', None)
    if acados_root is None:
        raise EnvironmentError("ACADOS_ROOT environment variable is not set.")
    py11_inc = pybind11.get_include()
    py_flags, ext_suffix = get_python_flags()

    make_cmd1=f"""gcc -std=c99 -fPIC -O2
        {model_name}_model/{model_name}_expl_ode_fun.c 
        {model_name}_model/{model_name}_expl_vde_forw.c 
        {model_name}_model/{model_name}_expl_vde_adj.c 
        acados_sim_solver_{model_name}.c 
        {model_name}_cost/{model_name}_cost_y_0_fun.c 
        {model_name}_cost/{model_name}_cost_y_0_fun_jac_ut_xt.c 
        {model_name}_cost/{model_name}_cost_y_0_hess.c 
        {model_name}_cost/{model_name}_cost_y_fun.c 
        {model_name}_cost/{model_name}_cost_y_fun_jac_ut_xt.c 
        {model_name}_cost/{model_name}_cost_y_hess.c 
        {model_name}_cost/{model_name}_cost_y_e_fun.c 
        {model_name}_cost/{model_name}_cost_y_e_fun_jac_ut_xt.c 
        {model_name}_cost/{model_name}_cost_y_e_hess.c acados_solver_{model_name}.c 
        -shared -o lib{model_name}_ocp_fncs.so 
        -L{acados_root}/lib -lacados -lhpipm -lblasfeo -lm
        -I{acados_root}/include
        -I{acados_root}/include/acados
        -I{acados_root}/include/blasfeo/include
        -I{acados_root}/include/hpipm/include
        -Wl,--disable-new-dtags,-rpath,{acados_root}/lib
        """.replace('\n', ' ')
    print("Building shared functions library...")
    # print(make_cmd1) 
    ret = os.system(make_cmd1)
    if ret != 0:
        raise RuntimeError("Failed to build the shared functions library.")
    make_cmd2 = f"""g++ -std=c++17 -O3 -fPIC -shared 
        model_ocp.cc -o lib{model_name}_ocp.so
        -l{model_name}_ocp_fncs
        -I{dest}
        -L{dest}
        -L{acados_root}/lib -lacados -lhpipm -lblasfeo -lm
        -I{acados_root}/include
        -I{acados_root}/include/acados
        -I{acados_root}/include/blasfeo/include
        -I{acados_root}/include/hpipm/include
        -Wl,--disable-new-dtags,-rpath,{acados_root}/lib
        """.replace('\n', ' ')
    print("Building C++ library...")
    # print(make_cmd2) 
    ret = os.system(make_cmd2)
    if ret != 0:
        raise RuntimeError("Failed to build the C++ library.")

    make_cmd3 = f"""g++ -std=c++17 -O3 -fPIC -shared 
        model_ocp_py.cc -o {model_name}_ocp_py{ext_suffix} 
        -l{model_name}_ocp_fncs
        -l{model_name}_ocp
        -I{dest}
        -I{py11_inc} {py_flags}
        -L{dest}
        -L{acados_root}/lib -lacados -lhpipm -lblasfeo -lm
        -I{acados_root}/include
        -I{acados_root}/include/acados
        -I{acados_root}/include/blasfeo/include
        -I{acados_root}/include/hpipm/include
        -Wl,--disable-new-dtags,-rpath,{acados_root}/lib
        """.replace('\n', ' ')
    print("Building Python extension module...")
    # print(make_cmd3) 
    ret = os.system(make_cmd3)
    if ret != 0:
        raise RuntimeError("Failed to build the Python extension module.")

    # Sim modules
    make_cmd_sim1 = f"""g++ -std=c++17 -O3 -fPIC -shared 
        model_sim.cc -o lib{model_name}_sim.so
        -l{model_name}_ocp_fncs
        -I{dest}
        -L{dest}
        -L{acados_root}/lib -lacados -lhpipm -lblasfeo -lm
        -I{acados_root}/include
        -I{acados_root}/include/acados
        -I{acados_root}/include/blasfeo/include
        -I{acados_root}/include/hpipm/include
        -Wl,--disable-new-dtags,-rpath,{acados_root}/lib
        """.replace('\n', ' ')
    print("Building C++ Sim library...")
    # print(make_cmd_sim1) 
    ret = os.system(make_cmd_sim1)
    if ret != 0:
        raise RuntimeError("Failed to build the C++ Sim library.")
    make_cmd_sim2 = f"""g++ -std=c++17 -O3 -fPIC -shared 
        model_sim_py.cc -o {model_name}_sim_py{ext_suffix} 
        -l{model_name}_ocp_fncs
        -l{model_name}_sim
        -I{dest}
        -I{py11_inc} {py_flags}
        -L{dest}
        -L{acados_root}/lib -lacados -lhpipm -lblasfeo -lm
        -I{acados_root}/include
        -I{acados_root}/include/acados
        -I{acados_root}/include/blasfeo/include
        -I{acados_root}/include/hpipm/include
        -Wl,--disable-new-dtags,-rpath,{acados_root}/lib
        """.replace('\n', ' ')
    print("Building Python Sim extension module...")
    # print(make_cmd_sim2)
    ret = os.system(make_cmd_sim2)
    if ret != 0:
        raise RuntimeError("Failed to build the Python Sim extension module.")

def parse_args():
    parser = argparse.ArgumentParser(description='Generate OCP C++ code from acados C code.')
    parser.add_argument('--model_name', type=str, required=True, help='Name of the model (e.g., quadrotor)')
    parser.add_argument('--c_generated_code_dir', type=str, required=True, help='Directory containing acados C generated code')
    parser.add_argument('--output_dir', type=str, required=True, help='Output directory for the generated C++ code')
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    generate_code(args.model_name, args.c_generated_code_dir, args.output_dir)