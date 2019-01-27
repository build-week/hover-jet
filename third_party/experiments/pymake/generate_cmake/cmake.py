import os
from functools import partial

from generate_cmake.log import Log


def _reduce_srcs(sources, base_directory, prejoin="${PROJECT_SOURCE_DIR}"):
    # return map(lambda o: os.path.join(prejoin, o.replace(base_directory + '/', "")), sources)
    # return map(lambda o: os.path.join(prejoin, o.replace(base_directory + '/', "")), sources)
    return sources


def create_lib(build_item, base_directory=""):
    bloated_srcs = map(partial(os.path.join, build_item['location']), build_item['srcs'])
    reduced_srcs = _reduce_srcs(bloated_srcs, base_directory)
    txt = "add_library({} {})".format(build_item['target'], " ".join(reduced_srcs))
    if len(build_item['deps']):
        txt += "\ntarget_link_libraries({} {})".format(build_item['target'], " ".join(build_item['deps']))
    return txt


def create_bin(build_item, base_directory=""):
    reduced_srcs = _reduce_srcs(build_item['srcs'], base_directory)

    if 'is_test' in build_item.get('flags', []):
        assert len(reduced_srcs) == 1, "A test only supports one source file"
        txt = 'add_test({target} {srcs} {libs})'.format(
            target=build_item['target'],
            srcs=reduced_srcs[0],
            libs=" ".join(build_item['deps']) if len(build_item['deps']) else '""'
        )
    else:
        txt = "add_executable({} {})".format(build_item['target'], " ".join(reduced_srcs))
        if len(build_item['deps']):
            txt += "\ntarget_link_libraries({} {})".format(build_item['target'], " ".join(build_item['deps']))
    return txt


def should_create_target(build_item):
    if 'CMakeLists.txt' in os.listdir(build_item['location']):
        Log.debug("Ignoring: {}".format(build_item['target']))
        return False
    else:
        return True


def write_cmake(text, base_directory):
    write_folder = os.path.join(base_directory, "tmp")
    if not os.path.exists(write_folder):
        os.makedirs(write_folder)

    write_path = os.path.join(write_folder, "CMakeLists.txt")
    with open(write_path, 'w') as cmake_out:
        cmake_out.write(text)


def build_cmakes(to_build, base_directory):
    actions = {
        'lib': create_lib,
        'binary': create_bin
    }

    from graph import dependency_sort
    write_order = dependency_sort(to_build)

    Log.success('\n\nGenerating cmake....')
    cmake_text = ""
    for write in write_order:
        if write in to_build:
            build_item = to_build[write]
            if not should_create_target(build_item):
                continue

            action = actions[build_item['kind']]
            new_text = action(build_item, base_directory=base_directory)
            cmake_text += new_text + '\n'
            Log.info(new_text)

    write_cmake(cmake_text, base_directory)
