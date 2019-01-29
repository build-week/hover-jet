from copy import deepcopy


def maybe_insert_deps(adj, key, data):
    if key in adj:
        if 'visited' in adj[key]:
            return
        deps = adj[key]['deps']
        adj[key]['visited'] = True

        for dep in deps:
            maybe_insert_deps(adj, dep, data)

    else:
        adj[key] = {'visited': True}
    data.append(key)


def dependency_sort(adj):
    our_adj = deepcopy(adj)
    assert len(our_adj.keys()) > 0, "Must have a graph!"
    result = []
    for element in adj:
        maybe_insert_deps(our_adj, element, result)
    return result

if __name__ == '__main__':
    test = {
        'a': {'deps': ['b', 'd']},
        'b': {'deps': ['c']},
        'd': {'deps': ['c']},
        'e': {'deps': ['d']},
    }

    print dependency_sort(test)
