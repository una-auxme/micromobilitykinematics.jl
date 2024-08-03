"""
    outer_sigularity_constraint(steering::Steering, suspension::Suspension)

calculats the angle change of two sequential steps of the left front wheel.
The result of the subtraction should always be negativ.

#Arguments
- `steering::Steering`: Instance of a specific steering
- `suspension::Suspension`: Instance of a specific suspension

#Returns:
- Difference between the current outer turning angle of the wheel and the next step
"""
function outer_sigularity_constraint(angleConfig::Tuple{T,T},steering::Steering, suspension::Suspension) where {T<:Real}
    θx, θz = angleConfig

    steering_now = copy(steering)
    steering_next = copy(steering)

    steeringkinematics!((θx, θz), steering_now, suspension)
    steeringkinematics!((θx, θz+1) , steering_next, suspension)
    
    δo = angle_δo(steering_now)
    δo_next = angle_δo(steering_next)
    return δo - δo_next
end 



"""
    inner_sigularity_constraint(steering::Steering, suspension::Suspension)

calculats the angle change of two sequential steps of the right front wheel.
The result of the subtraction should always be negativ.

#Arguments
- `steering::Steering`: Instance of a specific steering
- `suspension::Suspension`: Instance of a specific suspension

#Returns:
- Difference between the current inner turning angle of the wheel and the next step
"""
function inner_sigularity_constraint(angleConfig::Tuple{T,T},steering::Steering, suspension::Suspension) where {T<:Real}

    θx, θz = angleConfig

    steering_now = copy(steering)
    steering_next = copy(steering)

    steeringkinematics!((θx, θz) , steering_now, suspension)
    steeringkinematics!((θx, θz+1) , steering_next, suspension)
    
    δi = angle_δi(steering_now)
    δi_next = angle_δi(steering_next)
    return δi - δi_next
end


"""
    angle_dependence(steering::Steering)

calculates the Diffrence of the front wheels steering angles.
!The inner steering angle should always be bigger then the outer!

#Arguments
- `steering::Steering`: Instance of a specific steering

#Returns
- Difference between the current turning angles of the front wheels
"""
function angle_dependence(angleConfig::Tuple{T,T},steering::Steering, suspension::Suspension) where {T<:Real}

    steering_now = copy(steering)
    steeringkinematics!(angleConfig, steering_now, suspension)

    δo = angle_δo(steering_now)
    δi = angle_δi(steering_now)
    return δi - δo
end


"""
    left_circsphere_plane_dependence(steering::Steering)

claculats the diffrence between the distance of the centers and the actual radius of the actual range of motion
of the tie rod. (Left side of the kinematik steering mechanism)

#Arguments
-`steering::Steering`: Instance of a specific steering

#Returns
- Dependence for the interaction between circle and sphere of the kinematic

"""
function left_circsphere_plane_dependence(angleConfig::Tuple{T,T},steering::Steering, suspension::Suspension) where {T<:Real}


    steering_now = copy(steering)
    steeringkinematics!(angleConfig, steering_now, suspension)

    circ = Circle(steering_now.track_lever_mounting_points_ucs[1],steering_now.track_lever.length,steering_now.base_vec_wheel_ucs[1][:,3])
    sphere = Sphere(steering_now.sphere_joints[1],steering_now.tie_rod.length)

    normal = circ.normal/norm(circ.normal)        #circ.normal must be a unit vector
    d = (normal[1]*sphere.center[1] + normal[2]*sphere.center[2] + normal[3]*sphere.center[3] - sum(normal.*circ.center))

    return abs(d) > sphere.radius
end

"""
    right_circsphere_plane_dependence(steering::Steering)

claculats the diffrence between the distance of the circ/shere centers and the actual radius of the actual range of motion
of the tie rod. (Right side of the kinematik steering mechanism)


#Arguments
-`steering::Steering`: Instance of a specific steering

#Returns
- Dependence for the interaction between circle and sphere of the kinematic

"""
function right_circsphere_plane_dependence(angleConfig::Tuple{T,T},steering::Steering, suspension::Suspension) where {T<:Real}

    steering_now = copy(steering)
    steeringkinematics!(angleConfig, steering_now, suspension)

    circ = Circle(steering_now.track_lever_mounting_points_ucs[2],steering_now.track_lever.length,steering_now.base_vec_wheel_ucs[2][:,3])
    sphere = Sphere(steering_now.sphere_joints[2],steering_now.tie_rod.length)

    normal = circ.normal/norm(circ.normal)        #circ.normal must be a unit vector
    d = (normal[1]*sphere.center[1] + normal[2]*sphere.center[2] + normal[3]*sphere.center[3] - sum(normal.*circ.center))
    return abs(d) > sphere.radius
end

"""
    left_circcirc_min_intersec_dependence(steering::Steering)

calculates the Diffrence of the distance of the circ centeres d and the total length of both radii.
If the value of d is bigger then the total length of both radii there is no intersection of both circles.
(Left side of the kinematik steering mechanism)

#Arguments
-`steering::Steering`: Instance of a specific steering

#Returns
- Dependence for the minimal interaction between circle and circle of the kinematic
"""

function left_circcirc_min_intersec_dependence(angleConfig::Tuple{T,T},steering::Steering, suspension::Suspension) where {T<:Real}

    steering_now = copy(steering)
    steeringkinematics!(angleConfig, steering_now, suspension)


    #  from the GeoSpatialRelations package of the intersection() function

    circ1 = Circle(steering_now.track_lever_mounting_points_ucs[1],steering_now.track_lever.length,steering_now.base_vec_wheel_ucs[1][:,3])
    sphere = Sphere(steering_now.sphere_joints[1],steering_now.tie_rod.length)

    normal = circ1.normal/norm(circ1.normal)
    d = (normal[1]*sphere.center[1] + normal[2]*sphere.center[2] + normal[3]*sphere.center[3] - sum(normal.*circ1.center))

    r_circ_2 = sqrt(sphere.radius^2-d^2)
    center_circ_2 = sphere.center -d*normal

    circ2 = Circle(center_circ_2, r_circ_2, normal)

    if circ1.normal != circ2.normal
        throw(ArgumentError("The two circles do not lie in the same plane of three-dimensional space!"))
    end
    
    n_circ = circ1.normal
    n_circ = n_circ/norm(n_circ)        

    d = norm(circ1.center - circ2.center)                 

    return abs(d) - (abs(circ1.radius) + abs(circ2.radius))
end 


"""
    left_circcirc_min_intersec_dependence(steering::Steering)

calculates the Diffrence of the distance of the circ centeres d and the total length of both radii.
If the value of d is bigger then the total length of both radii there is no intersection of both circles.
(Right side of the kinematik steering mechanism)

#Arguments
-`steering::Steering`: Instance of a specific steering

#Returns
- Dependence for the minimal interaction between circle and circle of the kinematic
"""

function right_circcirc_min_intersec_dependence(angleConfig::Tuple{T,T}, steering::Steering, suspension::Suspension) where {T<:Real}
    #  from the GeoSpatialRelations package of the intersection() function

    steering_now = copy(steering)
    steeringkinematics!(angleConfig, steering_now, suspension)


    circ1 = Circle(steering_now.track_lever_mounting_points_ucs[2],steering_now.track_lever.length,steering_now.base_vec_wheel_ucs[2][:,3])
    sphere = Sphere(steering_now.sphere_joints[2],steering_now.tie_rod.length)

    normal = circ1.normal/norm(circ1.normal)
    d = (normal[1]*sphere.center[1] + normal[2]*sphere.center[2] + normal[3]*sphere.center[3] - sum(normal.*circ1.center))

    r_circ_2 = sqrt(sphere.radius^2-d^2)
    center_circ_2 = sphere.center -d*normal

    circ2 = Circle(center_circ_2, r_circ_2, normal)

    if circ1.normal != circ2.normal
        throw(ArgumentError("The two circles do not lie in the same plane of three-dimensional space!"))
    end
    
    n_circ = circ1.normal
    n_circ = n_circ/norm(n_circ)        

    d = norm(circ1.center - circ2.center)                 

    return abs(d) - (abs(circ1.radius) + abs(circ2.radius))
end 

"""
    left_circcirc_max_intersec_dependence(steering::Steering)

calculates the Diffrence of the radius og the circle2 (tie rod) and the total length of the distance of the circ centeres d and the radius of the circle1 (track lever).
If the value of the radius of circle2 is bigger then ther is no intersect posible. circle1 lies in circle2.
(Left side of the kinematik steering mechanism)

#Arguments
-`steering::Steering`: Instance of a specific steering

#Returns
- Dependence for the maximal interaction between circle and circle of the kinematic
"""
function left_circcirc_max_intersec_dependence(angleConfig::Tuple{T,T}, steering::Steering, suspension::Suspension) where {T<:Real}

    steering_now = copy(steering)
    steeringkinematics!(angleConfig, steering_now, suspension)


#  from the GeoSpatialRelations package of the intersection() function

    circ1 = Circle(steering_now.track_lever_mounting_points_ucs[1],steering_now.track_lever.length,steering_now.base_vec_wheel_ucs[1][:,3])
    sphere = Sphere(steering_now.sphere_joints[1],steering_now.tie_rod.length)

    normal = circ1.normal/norm(circ1.normal)
    d = (normal[1]*sphere.center[1] + normal[2]*sphere.center[2] + normal[3]*sphere.center[3] - sum(normal.*circ1.center))

    r_circ_2 = sqrt(sphere.radius^2-d^2)
    center_circ_2 = sphere.center -d*normal

    circ2 = Circle(center_circ_2, r_circ_2, normal)

    if circ1.normal != circ2.normal
        throw(ArgumentError("The two circles do not lie in the same plane of three-dimensional space!"))
    end

    n_circ = circ1.normal
    n_circ = n_circ/norm(n_circ)        

    d = norm(circ1.center - circ2.center)                 

    return abs(circ2.radius)-(abs(d) + abs(circ1.radius))
end 


"""
    left_circcirc_max_intersec_dependence(steering::Steering)

calculates the Diffrence of the radius og the circle2 (tie rod) and the total length of the distance of the circ centeres d and the radius of the circle1 (track lever).
If the value of the radius of circle2 is bigger then ther is no intersect posible. circle1 lies in circle2.
(Right side of the kinematik steering mechanism)

#Arguments
-`steering::Steering`: Instance of a specific steering

#Returns
- Dependence for the maximal interaction between circle and circle of the kinematic
"""
function right_circcirc_max_intersec_dependence(angleConfig::Tuple{T,T}, steering::Steering, suspension::Suspension) where {T<:Real}

    steering_now = copy(steering)
    steeringkinematics!(angleConfig, steering_now, suspension)

#  from the GeoSpatialRelations package of the intersection() function

    circ1 = Circle(steering_now.track_lever_mounting_points_ucs[2],steering_now.track_lever.length,steering_now.base_vec_wheel_ucs[2][:,3])
    sphere = Sphere(steering_now.sphere_joints[2],steering_now.tie_rod.length)

    normal = circ1.normal/norm(circ1.normal)
    d = (normal[1]*sphere.center[1] + normal[2]*sphere.center[2] + normal[3]*sphere.center[3] - sum(normal.*circ1.center))

    r_circ_2 = sqrt(sphere.radius^2-d^2)
    center_circ_2 = sphere.center -d*normal

    circ2 = Circle(center_circ_2, r_circ_2, normal)

    if circ1.normal != circ2.normal
        throw(ArgumentError("The two circles do not lie in the same plane of three-dimensional space!"))
    end

    n_circ = circ1.normal
    n_circ = n_circ/norm(n_circ)        

    d = norm(circ1.center - circ2.center)                 

    return abs(circ2.radius)-(abs(d) + abs(circ1.radius))
end 


"""
    track_circle_dependence(steering::Steering, measurments::Measurements)

calculates the diffrence of the max. ideal outer wheel angle and the current max. outer wheel angle.

#Arguments
-`steering::Steering`: Instance of a specific steering
-`measurments::Measurements`: Instance of a specific all relevant Measurements of the vehicle

#Returns
- Dependence for the minmal track circle
"""
function track_circle_dependence(angleConfig::Tuple{T,T}, steering::Steering, suspension::Suspension, measurments::Measurements) where {T<:Real}

    steering_now = copy(steering)
    steeringkinematics!(angleConfig, steering_now, suspension)

    δo = angle_δo(steering_now)
    r_is = measurments.wheel_base / sind(δo)    
    return measurments.turning_radius - r_is
end


