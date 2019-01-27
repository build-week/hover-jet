from collections import defaultdict

'''Must Support:

- pow
    - sqrt
- add
- multiply (product rule)
- divide (quotient rule)

TODO:
    - Can we use egraphs for expression simplification?
    - Can we make this work by only defining field operations?
    - Can we use random search to discover theorems?

'''


class DirectedAcyclicGraph(object):
    '''Implements a simple directed graph.'''
    def __init__(self):
        self.nodes = {}
        self.edges = defaultdict(lambda: set())
        self.parents = defaultdict(lambda: set())

    def node(self, name, content):
        self.nodes[name] = content
        self.parents[name] = set()

    def edge(self, a, b):
        assert a in self.nodes
        assert b in self.nodes

        self.edges[a].add(b)
        self.parents[b].add(a)

    def pretty_print(self):
        orphans = filter(lambda o: len(o[1]) == 0, self.parents.items())
        for node_id, _ in orphans:
            print node_id


class OpGraph(object):
    def __init__(self, scope_name='main'):
        self.tree = DirectedAcyclicGraph()

    def symbol(self, name):
        content = {
            'name': name,
            'kind': 'symbol'
        }
        self.tree.node(name, content)

    def op(self, name, args):
        content = {
            'name': name,
            'kind': 'op',
        }
        str_args = ', '.join(map(str, args))
        rname = '{}({})'.format(name, str_args)
        self.tree.node(rname, content)
        for arg in args:
            self.tree.edge(rname, arg)


def build_tree():
    pass


if __name__ == '__main__':
    st = OpGraph()
    st.symbol('a')
    st.symbol('b')
    st.op('add', ['a', 'b'])
    st.op('sub', ['add(a, b)', 'b'])

    print st.tree.pretty_print()
