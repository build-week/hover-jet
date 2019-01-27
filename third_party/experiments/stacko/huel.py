

def zoop(*args):
    print args

    for a in zip(*args):
        print a


zoop(['1', '2', '3'], ['a', 'b', 'c'])
