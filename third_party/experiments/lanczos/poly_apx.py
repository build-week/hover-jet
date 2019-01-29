import sympy


def main():
    '''
    This is a "derivation" of the simple rule
    '''
    a = sympy.Symbol('a')
    b = sympy.Symbol('b')
    c = sympy.Symbol('c')

    t = sympy.Symbol('(t - t_0)')

    u_list = []
    eqns = []
    for n in range(3):
        u = sympy.Symbol('u_{}'.format(n))
        tt = t - n
        eqns.append((a * (tt ** 2)) + (b * tt) + c - u)
        u_list.append(u)

    sympy.pretty_print(eqns)

    soln = sympy.solve(eqns, [a, b, c])
    f = (soln[a] * (t * t)) + (soln[b] * t) + c
    sympy.pretty_print(f)

    df_dt = sympy.diff(f, t)
    sympy.pretty_print(sympy.factor(df_dt))


if __name__ == '__main__':
    main()
