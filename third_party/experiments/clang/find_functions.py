import os

from clang import cindex
from clang.cindex import CursorKind
import clang


kinds = set()


def visitor(cursor, path, go=False):
    children = list(cursor.get_children())
    # print '  has {0} children'.format(len(children))

    if (cursor.kind not in kinds):
        print cursor.kind, ':', cursor.location.file, cursor.spelling

    kinds.add(cursor.kind)

    if cursor.kind == clang.cindex.CursorKind.INCLUSION_DIRECTIVE:
        print 'ffff'
        print cursor.spelling

    if cursor.kind in [CursorKind.UNEXPOSED_EXPR, CursorKind.UNEXPOSED_DECL, CursorKind.TRANSLATION_UNIT]:
        print cursor.kind, ':', cursor.location.file, cursor.spelling

    if (cursor.location.file is not None):
        if (os.path.realpath(cursor.location.file.name) != path):
            return

    if (cursor.location.file is not None and cursor.kind == clang.cindex.CursorKind.CXX_METHOD):
        path = os.path.realpath(cursor.location.file.name)

        print os.path.split(path)[-1], cursor.spelling
        for child in children:
            visitor(child, path)

    for child in children:
        visitor(child, path)


if __name__ == '__main__':
    cindex.Config.set_library_path('/usr/lib/llvm-3.8/lib')
    path = '/home/jacob/repos/experiments/geometry/spatial/bounding_volume_hierarchy.hh'

    index = cindex.Index.create()

    include_paths = [
        "/home/jacob/repos/experiments/geometry/spatial",
        "/home/jacob/repos/experiments/",
        "/home/jacob/repos/third_party/Sophus",
        "/home/jacob/repos/third_party/eigen3",
    ]

    include_args = map(lambda o: "-I" + o, include_paths)

    tu = index.parse(path, args=include_args)

    for incl in tu.get_includes():
        name = os.path.realpath(incl.location.file.name)

        if name.startswith('/home/jacob/repos/experiments/') and incl.depth <= 3:
            # print incl.location.file.name, ': ', incl.source.name, ': ', incl.depth
            location = os.path.split(incl.location.file.name)[-1]
            source = os.path.split(incl.source.name)[-1]
            include = os.path.split(incl.include.name)[-1]

            outstr = "loc: {0: <50} src: {1: <50} incl: {2: <40} depth: {3: <10}".format(
                location,
                source,
                include,
                incl.depth,
            )
            print outstr

    print '---'

    visitor(tu.cursor, path)

    print kinds
