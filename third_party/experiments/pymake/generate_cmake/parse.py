from generate_cmake.log import Log


def splitstrip(text, delim=','):
    return list(map(lambda o: o.strip(), text.split(delim)))


def needs(text, requirements):
    for requirement in requirements:
        assert requirement in text, "{} missing {}".format(text, requirement)


def mustnot(text, disquirements):
    for requirement in disquirements:
        assert requirement not in text, "{} may not have {}".format(text, requirement)


def between(text, ldelim='(', rdelim=')'):
    needs(text, [ldelim, rdelim])
    start = text.find(ldelim)
    end = text.rfind(rdelim)
    assert end - start > 1, "Delimeters empty"
    return text[start + 1: end]


pairs = {
    '<': '>',
    '(': ')',
    '"': '"',
    "'": "'",
}


def grab_dependencies(line):
    '''Parse dependencies out of a line'''
    needs(line, ['(', ')'])
    needs(line, ['//%deps'])

    args = between(line, '(', ')')
    deps = splitstrip(args)
    Log.debug('    deps: {}'.format(deps))
    return {'deps': deps}


def grab_lib(line):
    needs(line, ['(', ')'])
    needs(line, ['//%lib'])
    args = between(line, '(', ')')
    arg_tokens = splitstrip(args)
    Log.debug('    lib: {}'.format(arg_tokens))
    return {'lib': {'target': arg_tokens[0], 'srcs': arg_tokens[1:]}}


def grab_bin(line):
    needs(line, ['(', ')'])
    needs(line, ['//%bin'])
    args = between(line, '(', ')')
    arg_tokens = splitstrip(args)
    Log.debug('    bin: {}'.format(arg_tokens))
    assert len(arg_tokens) in (0, 1), "Wrong number of arguments"
    return {'bin': {'target': arg_tokens[0]}}


def grab_include(line):
    needs(line, ['#include'])

    incl_path = ""
    for start, end in pairs.items():
        if start in line:
            incl_path = between(line, start, end)
            break

    if start == '"':
        incl_type = "local"
    elif start == '<':
        incl_type = "system"
    else:
        raise NotImplementedError("wtf?")
    Log.debug("    include: {} ({})".format(incl_path, incl_type))

    return {'include': {'given_path': incl_path, 'type': incl_type}}


def make_flagger(flag_name):
    def flagger(line):
        Log.debug('    flag: {}'.format(flag_name))
        return {'flags': [flag_name]}
    return flagger


tokens = {
    '//%bin': grab_bin,
    '//%deps': grab_dependencies,
    '//%lib': grab_lib,
    '#include': grab_include,
    'int main(': make_flagger('has_main'),
    'void main(': make_flagger('has_main'),
    '//%ignore': make_flagger('ignore'),
    '#include "testing/gtest.hh"': make_flagger('is_test'),
    # '//%hdrlib': grab_hdr_lib,
}


def incremental_update(d, new_d):
    for key, item in new_d.items():
        if key not in d:
            d[key] = []

        if isinstance(item, list):
            d[key].extend(item)
        else:
            d[key].append(item)


def strip_spaces_after_comment(dirty_line):
    if dirty_line.startswith('//'):
        return '//' + dirty_line[2:].lstrip()
    else:
        return dirty_line


def parse_text(text):
    elements = {
        'bin': [],
        'deps': [],
        'lib': [],
        'include': [],
        'flags': []
    }

    lines = text.split('\n')
    for dirty_line in lines:
        line = strip_spaces_after_comment(dirty_line)
        for token, action in tokens.items():
            if line.startswith(token):
                update = action(line)
                incremental_update(elements, update)

    return elements


def has_annotations(elements):
    no_deps = len(elements['deps']) == 0
    no_bin = len(elements['bin']) == 0
    no_lib = len(elements['lib']) == 0
    return not all([no_deps, no_bin, no_lib])


def should_ignore(elements):
    return 'ignore' in elements['flags']


def main():
    text = "// %deps(a, b, c)\n//    %deps(ogop)\n//%deps(abs)\n%deps(adq)"
    parsed = parse_text(text)
    assert set(parsed['deps']) == {'a', 'b', 'c', 'ogop', 'abs'}


if __name__ == '__main__':
    main()
