import os
Import("env")

project_dir = env.subst("$PROJECT_DIR")
lib_dir = os.path.join(project_dir, "lib", "acados_arm")

env.Append(
    LIBPATH=[lib_dir],
    LIBS=["acados", "hpipm", "blasfeo", "m"]
)