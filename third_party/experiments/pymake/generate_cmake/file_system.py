import os
import re

from generate_cmake.parse import between

valid_extensions = ['cc', 'hh']


def parse_cfg(path, base):
    with open(os.path.join(path, 'cfg.pymake')) as f:
        text = f.read()

    include_paths = []
    commands = text.split('\n')
    for command in commands:
        if(command.startswith('add_path') or
           command.startswith('add_ext_path')):
            include_paths.append(
                os.path.join(
                    base,
                    between(command, '(', ')')
                )
            )

    return {'include_paths': include_paths}


def resolve_include_TODO(file_path, include_path, available_include_paths):
    dirname = os.path.dirname(file_path)
    other_files = filter(os.path.isfile, os.listdir(dirname))

    if include_path in other_files:
        return os.path.join(dirname, include_path)
    # TODO: Make this handle more cases


def resolve_include(file_path, include_path, available_include_paths=[]):
    # return os.path.join('/home/jacob/repos/experiments', include_path)
    directory, _ = os.path.split(file_path)
    extended_available_incl_paths = available_include_paths + [directory]
    for path in extended_available_incl_paths:
        potential_path = os.path.join(path, include_path)
        if os.path.exists(potential_path):
            return potential_path
    else:
        raise ValueError("Unknown include: {} in {}".format(include_path, file_path))


def parse_ignore(path):
    if path is None or '.ignore' not in os.listdir(path):
        ignores = [
            ".git",
            "bin/",
            "third_party/",
            "common/",
        ]
    else:
        with open(os.path.join(path, '.ignore')) as f:
            text = f.read()
        ignores = text.split('\n')
    return list(map(re.compile, ignores))


def deep_listdir(path):
    dir_contents = os.listdir(path)
    return map(lambda o: os.path.join(path, o), dir_contents)


def get_files(path, ignores_path=None, ignores=None):
    from functools import partial
    if ignores is None:
        ignores = parse_ignore(ignores_path)

    def want_file(file):
        re_partial = partial(re.match, string=file)
        allow_by_is_file = os.path.isfile(file)
        allow_by_extension = any(map(file.endswith, valid_extensions))
        allow_by_ignores = not any(map(re_partial, ignores))

        directory, _ = os.path.split(file)
        requires = [
            allow_by_ignores,
            allow_by_extension,
            allow_by_is_file,
        ]

        return all(requires)

    def want_dir(dirname):
        directory, filename = os.path.split(dirname)
        re_partial = partial(re.match, string=filename + '/')
        allow_by_ignores = not any(map(re_partial, ignores))
        return allow_by_ignores and os.path.isdir(dirname)

    dir_elements = deep_listdir(path)
    files_here = filter(want_file, dir_elements)
    dirs_here = filter(want_dir, dir_elements)

    for subdir in dirs_here:
        subdir_path = os.path.join(path, subdir)
        if 'CMakeLists.txt' in os.listdir(subdir_path):
            continue

        files_there = get_files(subdir_path, ignores=ignores)
        files_here.extend(files_there)
    return files_here
