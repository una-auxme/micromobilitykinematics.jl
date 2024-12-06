"""
    intersection(line::Line, plane::Plane)

Calculating the intersection point between a line and a plane

# Arguments
- `line::Line`: instance of a line
- `plane::Plane`: instance of a plane

# Returns
- `intersection_point::Vector`: point of intersection

"""
function intersection(line::Line, plane::Plane)

    x = dot(plane.normal, plane.point .- line.point) / dot(plane.normal, line.direction)

    intersection_point = line.point .+ x * line.direction

    return intersection_point

end

"""
    intersection(line::Line, line2::Line)

Calculating the intersection point between two lines 

# Arguments
- `line1::Line`: instance of a line
- `line2::Line`: instance of a line

# Returns
There are two possibilities:
- If there is no intersection point, an error message is displayed.
- There is one intersection then `intersection_point::Vector` is returned.

"""
function intersection(line1::Line, line2::Line)
    

    A = hcat(line1.direction,line2.direction*-1)
    b = line2.point .- line1.point
  

    if (line1.direction / norm(line1.direction)) ==  (line2.direction / norm(line2.direction))
        error("Both directional vectors have the same direction.")
        return nothing 
    end

    t = A[1:2,:] \ b[1:2]

    mA = reshape(A[3,:],1,2)
    mt = reshape(t,2,1)

    if  mA * mt != b[3]
        error("The given arguments are skew lines.")
    end

    intersection_point = line1.point .+ t[1] .* line1.direction


    return intersection_point
end



"""
    intersection(line::Line, sphere::Sphere)

calculating the intersection point for a line and a sphere

# Arguments
- `line::Line`: instance of a line
- `sphere::Sphere`: instance of a sphere

# Returns
There are three possibilities:
- If there is no intersection point, an error message is displayed.
- There is one intersection then `intersection_point::Vector` is returned.
- There are two intersection then `intersection_point1::Vector` and `intersection_point2::Vector` are returned.

"""
function intersection(line::Line,sphere::Sphere)
    a = dot(line.direction, line.direction)
    b = 2 * dot(line.direction, line.point - sphere.center)
    c = dot(line.point - sphere.center, line.point - sphere.center) - sphere.radius^2

    discriminant = b^2 - 4 * a * c

    if discriminant < 0
        # Kein Schnittpunkt
        error("No intersection")
        return nothing
    elseif discriminant == 0
        # Eine Lösung (tangentialer Schnittpunkt)
        t = -b / (2 * a)
        intersection_point = line.point + t *  line.direction
        return intersection_point
    else
        # Zwei Lösungen (Schnittpunkte)
        t1 = (-b + sqrt(discriminant)) / (2 * a)
        t2 = (-b - sqrt(discriminant)) / (2 * a)
        intersection_point1 = line.point + t1 *  line.direction
        intersection_point2 = line.point + t2 *  line.direction
        return intersection_point1, intersection_point2
    end
end

"""
    intersection(circ1::Circle, circ2::Circle)

calculating the intersection point for two cicles

# Arguments
- `circ1::Circle`: instance of a circle
- `circ2::Circle`: instance of a circle

# Returns
There are three possibilities:
- If there is no intersection point, an error message is displayed.
- There is one intersection then `intersection_point::Vector` is returned.
- There are two intersection then `intersection_point1::Vector` and `intersection_point2::Vector` are returned.

"""
function intersection(circ1::Circle, circ2::Circle)
    if circ1.normal != circ2.normal
        throw(ArgumentError("The two circles do not lie in the same plane of three-dimensional space!"))
    end

    n_circ = circ1.normal

    n_circ = n_circ/norm(n_circ)        # n_circ must be a unit vector
    d = norm(circ1.center - circ2.center)                 
    
    if d > (abs(circ1.radius) + abs(circ2.radius)) || ((abs(d) + abs(circ1.radius)) <  abs(circ2.radius)) || ((abs(d) + abs(circ2.radius)) <  abs(circ1.radius))   # Check if circles do not intersect      
        error("circles do not intersect")
    elseif d == (abs(circ1.radius) + abs(circ2.radius)) ||   (abs(d) + abs(circ1.radius)) == abs(circ2.radius) # Check if circles are tangent at one point     
        p1 = circ1.center + (circ1.center -circ2.center)/norm(circ1.center -circ2.center) * circ1.radius;
        p2 = [];
    else                                        
        lambda = acos((norm(circ1.center -circ2.center)^2 + circ1.radius^2-circ2.radius^2)/(2*circ1.radius*(norm(circ1.center -circ2.center))));
        h = sin(lambda)*circ1.radius;  
        a = sqrt(circ1.radius^2-h^2);
        ps = circ1.center - ((circ1.center-circ2.center)/norm(circ1.center-circ2.center))*a;
        v = cross(n_circ,(circ1.center-circ2.center)/norm(circ1.center-circ2.center));
        p1 = ps - h*(v/norm(v));
        p2 = ps + h*(v/norm(v));
        return p1,p2
    end
end

"""
    intersection(circ::Circle, sphere::Sphere)

calculating the intersection point for a circle and a sphere

# Arguments
- `circ::Circle`: instance of a circle
- `sphere::Sphere`: instance of a sphere

# Returns
There are three possibilities:
- If there is no intersection point, an error message is displayed.
- There is one intersection then `intersection_point::Vector` is returned.
- There are two intersection then `intersection_point1::Vector` and `intersection_point2::Vector` are returned.

"""
function intersection(circ::Circle, sphere::Sphere)
    normal = circ.normal/norm(circ.normal)        #circ.normal must be a unit vector
    d = (normal[1]*sphere.center[1] + normal[2]*sphere.center[2] + normal[3]*sphere.center[3] - sum(normal.*circ.center))
    if abs(d) > sphere.radius
        error("No intersection")
    elseif abs(d) == sphere.radius
        I = circ.center + sphere.radius*normal
    else
        r_circ_2 = sqrt(sphere.radius^2-d^2)
        center_circ_2 = sphere.center -d*normal
        circ2 = Circle(center_circ_2, r_circ_2, normal)
        p1, p2 = intersection(circ, circ2)
    end
    return p1,p2
end