import sympy


def equality():
    g = sympy.Symbol('g(x)')
    f = sympy.Symbol('f(x)')
    lam_0 = sympy.Symbol('lambda_0')
    alpha = sympy.Symbol('alpha')

    half = sympy.Rational(1, 2)
    lam = lam_0 - ((1 / alpha) * g)
    lagrangian = f + -(lam * g) + -alpha * half * ((lam - lam_0) ** 2)

    print("L(x, lambda) = ...")
    sympy.pretty_print(sympy.simplify(lagrangian))

    print("\n\n\n\n")
    print("lambda(i + 1) = ...")
    sympy.pretty_print(lam)


if __name__ == "__main__":
    equality()
