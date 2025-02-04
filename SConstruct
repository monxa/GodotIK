#!/usr/bin/env python
import os
import sys

LIB_BASE_PATH = "godot_project/addons/libik/bin/libik"

try:
    Import('env')
except Exception:
    env = SConscript("godot-cpp/SConstruct").Clone()

# Enable caching if a cache directory is specified via the environment.
if 'SCONS_CACHE_DIR' in os.environ:
    env.CacheDir(os.environ['SCONS_CACHE_DIR'])

def recursive_glob(sourcedir, suffix):
    result = []
    for root, dirs, files in os.walk(sourcedir):
        for file in files:
            if file.endswith(suffix):
                result.append(os.path.join(root, file))
    return result

env.Append(CPPPATH=["src"])
env.Append(CFLAGS=['-g', '-pg'])
env["symbols_visibility"] = "visible"

sources = recursive_glob("src/", ".cpp")

doc_data = env.GodotCPPDocData("godot-cpp/src/gen/doc_data.gen.cpp", source=Glob("doc_classes/*.xml"))
sources.append(doc_data)

lib_output_path = f'{LIB_BASE_PATH}{env["SHLIBSUFFIX"]}'

if env["platform"] in ["ios", "android"]:
    lib_output_path = f'{LIB_BASE_PATH}.{env["platform"]}{env["SHLIBSUFFIX"]}'

library = env.SharedLibrary(
        lib_output_path,
        source=sources,
)

Default(library)