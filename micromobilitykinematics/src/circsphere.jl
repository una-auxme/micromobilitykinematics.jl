# calculates the intersecting points of a sphere (surface) and a circle
# pos_circ: position of the circle
# r_circ: radius of the circle
# n_circ: normal vector of the circle
# pos_sphere: position of the sphere
# r_sphere: radius of the sphere


include("circcirc.jl")




"""
    circsphere(pos_circ,r_circ,n_circ,pos_sphere,r_sphere)

calculates the intersecting points of a sphere (surface) and a circle

# Arguments
-`pos_circ`: position of the circle
-`r_circ`: radius of the circle
-`n_circ`: normal vector of the circle
-`pos_sphere`: position of the sphere
-`r_sphere`: radius of the sphere

# Returns
-`p1`: intersecting point
-`p2`: intersecting point

"""
function circsphere(pos_circ,r_circ,n_circ,pos_sphere,r_sphere)
    print(pos_circ,r_circ,n_circ,pos_sphere,r_sphere)
    n_circ = n_circ/norm(n_circ)        #n_circ must be a unit vector
    d = (n_circ[1]*pos_sphere[1] + n_circ[2]*pos_sphere[2] + n_circ[3]*pos_sphere[3] - sum(n_circ.*pos_circ))
    if abs(d) > r_sphere
        error("No Intersection")
    elseif abs(d) == r_sphere
        I = pos_circ + r_sphere*n_circ
    else
        r_circ_2 = sqrt(r_sphere^2-d^2)
        pos_circ_2 = pos_sphere -d*n_circ
        p1, p2 = circcirc(pos_circ, r_circ, n_circ, pos_circ_2, r_circ_2)
    end
    return p1,p2
end

