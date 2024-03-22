
using LinearAlgebra

"""
calc_basis_vectors(v)

Berechnet die drei Basisvektoren eines Koordinatensystems, indem ein gegebener Basisvektor `v` als Eingabe verwendet wird. `v` soll z darstellen

# Argumente
- `v`: Ein Vektor, der als Basisvektor verwendet wird.

# Rückgabewerte
- Eine 3x3 Matrix mit drei Basisvektoren, die das Koordinatensystem definieren.
"""


function calc_basis_vectors(v)

    z = v / norm(v)
    x = cross(z,[0, 0, 1])
    #Nullvektor überpfrüfung
    if norm(x) < 1e-10
        x = cross(v, [0, 1, 0])
    end
    x = x / norm(x)
    y = cross(z,x)
    y = y / norm(y)

    x,y,z
end