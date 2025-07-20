"""
    steeringkinematicsMOVED!(θ::Tuple{T,T,T}, steering::Steering, suspension::Suspension) where {T<:Real}

For the moving rotational component with the angles (θx, θz), the kinematics of the steering is calculated

# Arguments
- `θ::Tuple{T,T,T}`: angles (θx,θy,θz) in which the rotational component is rotated
        - `θx`: Angle of rotation of the rotation component around the x-axis
        - `θy`: Angle of rotation of the rotation component around the y-axis
        - `θz`: Angle of rotation of the rotation component around the z-axis
- `steering::Steering`: Instance of a specific steering
- `suspension::Suspension`: Instance of a specific suspension

# Returns:
- no returns because of in place programming
"""
function steeringkinematicsMOVED!(θ::Tuple, steering::Steering, suspension::Suspension)

    # --- extract steering angles ---
    θx, θy, θz = θ

    # --- Save input steering angles to steering object (in degrees) ---
    steering.θx = θx
    steering.θy = θy
    steering.θz = θz

    # --- Convert angles to radians for computation ---
    θx = deg2rad(θx)
    θy = deg2rad(θy)
    θz = deg2rad(θz)

    ############# INITIAL NEUTRAL POSITIONS ################

    # --- Coordinate system basis vectors (identity matrix) ---
    rotational_component_ucs = [[1;0;0] [0;1;0] [0;0;1]]

    # --- Define neutral position vectors for rotational centers and joints ---
    vec_x_rotational_neutral = [0.0; 0.0; -steering.rotational_component.x_rotational_radius]
    vec_z_rotational_neutral = [-steering.rotational_component.z_rotational_radius; 0.0; 0.0] + vec_x_rotational_neutral 

    left_sphere_joints_neutral = [0.0; steering.rotational_component.to_joint_pivot_point; 0] + vec_z_rotational_neutral
    right_sphere_joints_neutral = [0.0; -steering.rotational_component.to_joint_pivot_point; 0] + vec_z_rotational_neutral
    
    # --- Apply rotation around Y-axis (handlebar tilt) ---
    ~, vec_x_rotational_neutral = rotate3(vec_x_rotational_neutral, rotational_component_ucs[2,:], -θy)
    ~, vec_z_rotational_neutral = rotate3(vec_z_rotational_neutral, rotational_component_ucs[2,:], -θy)
    ~, left_sphere_joints_neutral = rotate3(left_sphere_joints_neutral, rotational_component_ucs[2,:], -θy)
    ~, right_sphere_joints_neutral = rotate3(right_sphere_joints_neutral, rotational_component_ucs[2,:], -θy)

    # --- Save updated neutral vectors ---
    steering.vec_x_rotational_neutral = vec_x_rotational_neutral
    steering.vec_z_rotational_neutral = vec_z_rotational_neutral
    steering.sphere_joints_neutral = (left_sphere_joints_neutral, right_sphere_joints_neutral)


    ############# APPLY ROTATIONS (X, THEN Z) ################
    ~, vec_x_rotational = rotate3(vec_x_rotational_neutral, rotational_component_ucs[1,:], θx)

    ~, vec_z_rotational = rotate3(vec_z_rotational_neutral, rotational_component_ucs[1,:], θx)
    ~, vec_z_rotational = rotate3(vec_z_rotational, vec_x_rotational, θz)
    
    ~, left_sphere_joints = rotate3(steering.sphere_joints_neutral[1], rotational_component_ucs[1,:], θx)
    ~, left_sphere_joints = rotate3(left_sphere_joints, vec_x_rotational, θz)

    ~, right_sphere_joints = rotate3(steering.sphere_joints_neutral[2], rotational_component_ucs[1,:], θx)
    ~, right_sphere_joints = rotate3(right_sphere_joints, vec_x_rotational, θz)

    # --- Save rotated joint vectors ---
    steering.vec_z_rotational = vec_z_rotational
    steering.vec_x_rotational = vec_x_rotational
    steering.sphere_joints = (left_sphere_joints, right_sphere_joints) 



    ############# SUSPENSION AND WHEEL GEOMETRY ################

    # --- Wheel coordinate system basis ---
    wheel_ucs = [[1.0;0.0;0.0] [0.0;1.0;0.0] [0.0;0.0;1.0]]

    # --- Update suspension state ---
    suspensionkinematics!(suspension)

    # --- Get wheel positions from suspension ---
    wheel_ucs_position = (suspension.lowerwishbone[1].sphere_joint, suspension.lowerwishbone[2].sphere_joint)
    steering.wheel_ucs_position = wheel_ucs_position
    
    # --- Compute z-axis of each wheel based on wishbone geometry ---
    left_axis_z = (suspension.upperwishbone[1].sphere_joint - suspension.lowerwishbone[1].sphere_joint) / norm(suspension.upperwishbone[1].sphere_joint - suspension.lowerwishbone[1].sphere_joint)
    right_axis_z = (suspension.upperwishbone[2].sphere_joint - suspension.lowerwishbone[2].sphere_joint) / norm(suspension.upperwishbone[2].sphere_joint - suspension.lowerwishbone[2].sphere_joint)
    wheel_ucs_axis_z = (left_axis_z,  right_axis_z)

    # --- Compute local basis vectors for wheel coordinate systems ---
    base_vec_y, base_vec_x, base_vec_z = calc_basis_vectors(wheel_ucs_axis_z[1])
    left_base_vec = [base_vec_x base_vec_y base_vec_z]

    base_vec_y, base_vec_x, base_vec_z = calc_basis_vectors(wheel_ucs_axis_z[2])
    right_base_vec = [base_vec_x base_vec_y base_vec_z]

    steering.base_vec_wheel_ucs = (left_base_vec, right_base_vec)    # (left, right)

    ############# COMPUTE TRACK LEVER MOUNTING POINT ################

    lower_end_of_rotational_component = (steering.wishbone_ucs_position[1].*[-1,-1,-1] - [0.0,0.0,steering.rotational_component.x_rotational_radius])
    
    # --- Define geometric constraints ---
    line = GeoSpatialRelations.Line(suspension.lowerwishbone[1].sphere_joint_neutral, left_base_vec[:,3])
    plane = GeoSpatialRelations.Plane(lower_end_of_rotational_component, [0.0,0.0,1.0])

    # --- Intersection point gives offset for wheel_ucs to track lever mounting point ---
    inter = GeoSpatialRelations.intersection(line, plane)
    vec_offset = inter - suspension.lowerwishbone[1].sphere_joint_neutral
    offset_length = norm(vec_offset)
    
    # --- Track lever mounting point in wheel_ucs
    track_lever_mount = [0.0; 0.0; offset_length]

    circle_joints_dict = Dict(:left_joint => Vector{<:Any}(), :right_joint => Vector{<:Any}()) 
    mount_dict = Dict(:left_mount => Vector{<:Any}(), :right_mount => Vector{<:Any}())

    for (circle_joint,mount,index,shift) in zip([:left_joint, :right_joint],[:left_mount, :right_mount], [1,2], [[1,1,1],[1,-1,1]])

        # --- Transform mounting point through coordinate systems ---
        track_lever_mount_IN_wheel_ucs = applyMatrixRotation(track_lever_mount,steering.base_vec_wheel_ucs[index], wheel_ucs)
        track_lever_mount_IN_wishbone_ucs = wheel_ucs_position[index] + track_lever_mount_IN_wheel_ucs
        track_lever_mount_IN_steering_ucs = steering.wishbone_ucs_position[index] + track_lever_mount_IN_wishbone_ucs.*(shift)

        #@eval $mount = $track_lever_mount_IN_steering_ucs                          # Not compatible with threading
        mount_dict[mount] = track_lever_mount_IN_steering_ucs

        # --- Create geometric primitives for intersection ---
        circ = GeoSpatialRelations.Circle(track_lever_mount_IN_steering_ucs,steering.track_lever.length,steering.base_vec_wheel_ucs[index][:,3])
        sphere = GeoSpatialRelations.Sphere(steering.sphere_joints[index],steering.tie_rod.length)

        circle_joints_1, circle_joints_2 = GeoSpatialRelations.intersection(circ, sphere)

        # --- Choose valid intersection based on x-distance ---
        if circle_joints_2[1] - track_lever_mount_IN_steering_ucs[1] < circle_joints_1[1] - track_lever_mount_IN_steering_ucs[1]                          
            #@eval $circle_joint = $circle_joints_2                                  # Not compatible with threading
            circle_joints_dict[circle_joint] = circle_joints_2
        else
            #@eval $circle_joint = $circle_joints_1                                  # Not compatible with threading
            circle_joints_dict[circle_joint] = circle_joints_1
        end
    end
    #steering.track_lever_mounting_points_ucs = (collect(left_mount), collect(right_mount))
    #steering.circle_joints = (collect(left_joint), collect(right_joint))
    steering.track_lever_mounting_points_ucs = (mount_dict[:left_mount], mount_dict[:right_mount])
    steering.circle_joints = (circle_joints_dict[:left_joint],circle_joints_dict[:right_joint])
    nothing
end


"""
    steeringkinematicsNEUTRAL!(θ::Tuple{T,T,T}, steering::Steering, suspension::Suspension) where {T<:Real}

For the rotation component in its rest position with the angles (θx, θz) = (0,0), the kinematics of the steering is calculated

# Arguments
- `θ::Tuple{T,T,T}`: angles (θx,θy,θz) in which the rotational component is rotated
        - `θx`: Angle of rotation of the rotation component around the x-axis
        - `θy`: Angle of rotation of the rotation component around the y-axis
        - `θz`: Angle of rotation of the rotation component around the z-axis
- `steering::Steering`: Instance of a specific steering
- `suspension::Suspension`: Instance of a specific suspension

# Returns:
- no returns because of in place programming
"""
function steeringkinematicsNEUTRAL!(θ::Tuple, steering::Steering, suspension::Suspension)
    # --- extract steering angles ---
    θx, θy, θz = θ

    # --- Convert angles to radians for computation ---
    θx = deg2rad(0.0)
    θy = deg2rad(θy)
    θz = deg2rad(0.0)

    ############# INITIAL NEUTRAL POSITIONS ################
    # --- Coordinate system basis vectors (identity matrix) ---
    rotational_component_ucs = [[1;0;0] [0;1;0] [0;0;1]]

    # --- Define neutral position vectors for rotational centers and joints ---
    vec_x_rotational_neutral = [0.0; 0.0; -steering.rotational_component.x_rotational_radius]
    vec_z_rotational_neutral = [-steering.rotational_component.z_rotational_radius; 0.0; 0.0] + vec_x_rotational_neutral 

    left_sphere_joints_neutral = [0.0; steering.rotational_component.to_joint_pivot_point; 0] + vec_z_rotational_neutral
    right_sphere_joints_neutral = [0.0; -steering.rotational_component.to_joint_pivot_point; 0] + vec_z_rotational_neutral

    steering.sphere_joints_neutral = (left_sphere_joints_neutral, right_sphere_joints_neutral)


    # --- Apply rotation around Y-axis (handlebar tilt) ---
    ~, vec_x_rotational_neutral = rotate3(vec_x_rotational_neutral, rotational_component_ucs[2,:], -θy)
    ~, vec_z_rotational_neutral = rotate3(vec_z_rotational_neutral, rotational_component_ucs[2,:], -θy)
    ~, left_sphere_joints_neutral = rotate3(left_sphere_joints_neutral, rotational_component_ucs[2,:], -θy)
    ~, right_sphere_joints_neutral = rotate3(right_sphere_joints_neutral, rotational_component_ucs[2,:], -θy)




    ############# APPLY ROTATIONS (X, THEN Z) ################
    ~, vec_x_rotational = rotate3(vec_x_rotational_neutral, rotational_component_ucs[1,:], θx)

    ~, vec_z_rotational = rotate3(vec_z_rotational_neutral, rotational_component_ucs[1,:], θx)
    ~, vec_z_rotational = rotate3(vec_z_rotational, vec_x_rotational, θz)
    
    ~, left_sphere_joints = rotate3(steering.sphere_joints_neutral[1], rotational_component_ucs[1,:], θx)
    ~, left_sphere_joints = rotate3(left_sphere_joints, vec_x_rotational, θz)

    ~, right_sphere_joints = rotate3(steering.sphere_joints_neutral[2], rotational_component_ucs[1,:], θx)
    ~, right_sphere_joints = rotate3(right_sphere_joints, vec_x_rotational, θz)

    # --- Save rotated joint vectors ---
    steering.sphere_joints_neutral = (left_sphere_joints, right_sphere_joints) 

    ############# SUSPENSION AND WHEEL GEOMETRY ################

    # --- Wheel coordinate system basis ---
    wheel_ucs = [[1.0;0.0;0.0] [0.0;1.0;0.0] [0.0;0.0;1.0]]

    # --- Update suspension state ---
    suspensionkinematics!(suspension)

    # --- Get wheel positions from suspension ---
    wheel_ucs_position = (suspension.lowerwishbone[1].sphere_joint, suspension.lowerwishbone[2].sphere_joint)
    steering.wheel_ucs_position = wheel_ucs_position
    
    # --- Compute z-axis of each wheel based on wishbone geometry ---
    left_axis_z = (suspension.upperwishbone[1].sphere_joint - suspension.lowerwishbone[1].sphere_joint) / norm(suspension.upperwishbone[1].sphere_joint - suspension.lowerwishbone[1].sphere_joint)
    right_axis_z = (suspension.upperwishbone[2].sphere_joint - suspension.lowerwishbone[2].sphere_joint) / norm(suspension.upperwishbone[2].sphere_joint - suspension.lowerwishbone[2].sphere_joint)
    wheel_ucs_axis_z = (left_axis_z,  right_axis_z)

    # --- Compute local basis vectors for wheel coordinate systems ---
    base_vec_y, base_vec_x, base_vec_z = calc_basis_vectors(wheel_ucs_axis_z[1])
    left_base_vec = [base_vec_x base_vec_y base_vec_z]

    base_vec_y, base_vec_x, base_vec_z = calc_basis_vectors(wheel_ucs_axis_z[2])
    right_base_vec = [base_vec_x base_vec_y base_vec_z]

    base_vec_wheel_ucs = (left_base_vec, right_base_vec)    # (left, right)

    ############# COMPUTE TRACK LEVER MOUNTING POINT ################
    lower_end_of_rotational_component = (steering.wishbone_ucs_position[1].*[-1,-1,-1] - [0.0,0.0,steering.rotational_component.x_rotational_radius])
    
    # --- Define geometric constraints ---
    line = GeoSpatialRelations.Line(suspension.lowerwishbone[1].sphere_joint_neutral, left_base_vec[:,3])
    plane = GeoSpatialRelations.Plane(lower_end_of_rotational_component, [0.0,0.0,1.0])

    # --- Intersection point gives offset for wheel_ucs to track lever mounting point ---
    inter = GeoSpatialRelations.intersection(line, plane)
    
    vec_offset = inter - suspension.lowerwishbone[1].sphere_joint_neutral
    offset_length = norm(vec_offset)
    
    # --- Track lever mounting point in wheel_ucs
    track_lever_mount = [0.0; 0.0; offset_length]

    
    circle_joints_dict = Dict(:left_joint => Vector{<:Any}(), :right_joint => Vector{<:Any}()) 
    mount_dict = Dict(:left_mount => Vector{<:Any}(), :right_mount => Vector{<:Any}())


    for (circle_joint,mount,index,shift) in zip([:left_joint, :right_joint],[:left_mount, :right_mount], [1,2], [[1,1,1],[1,-1,1]])

        # --- Transform mounting point through coordinate systems ---
        track_lever_mount_IN_wheel_ucs = applyMatrixRotation(track_lever_mount,base_vec_wheel_ucs[index], wheel_ucs)
        track_lever_mount_IN_wishbone_ucs = wheel_ucs_position[index] + track_lever_mount_IN_wheel_ucs
        track_lever_mount_IN_steering_ucs = steering.wishbone_ucs_position[index] + track_lever_mount_IN_wishbone_ucs.*(shift)

        #@eval $mount = $track_lever_mount_IN_steering_ucs                                  # Not compatible with threading
        mount_dict[mount] = track_lever_mount_IN_steering_ucs

        # --- Create geometric primitives for intersection ---
        circ = GeoSpatialRelations.Circle(track_lever_mount_IN_steering_ucs,steering.track_lever.length,base_vec_wheel_ucs[index][:,3])
        sphere = GeoSpatialRelations.Sphere(steering.sphere_joints_neutral[index],steering.tie_rod.length)

        circle_joints_1, circle_joints_2 = GeoSpatialRelations.intersection(circ, sphere)
        
        # --- Choose valid intersection based on x-distance ---
        if circle_joints_2[1] - track_lever_mount_IN_steering_ucs[1] < circle_joints_1[1] - track_lever_mount_IN_steering_ucs[1]                          
            #@eval $circle_joint = $circle_joints_2                                          # Not compatible with threading
            circle_joints_dict[circle_joint] = circle_joints_2
        else
            #@eval $circle_joint = $circle_joints_1                                          # Not compatible with threading
            circle_joints_dict[circle_joint] = circle_joints_1
        end
    end
    #steering.circle_joints_neutral = (collect(left_joint), collect(right_joint))                   # Not compatible with threading
    steering.circle_joints_neutral = (circle_joints_dict[:left_joint],circle_joints_dict[:right_joint])
    nothing
end



"""
    steeringkinematics!(θ::Tuple{T,T}, steering::Steering, suspension::Suspension) where {T<:Real}

To fully describe the kinematics of the steering system, both MOVED and NEUTRAL states must be calculated

# Arguments
- `θ::Tuple{T,T}`: angles (θx,θz) in which the rotational component is rotated
        - `θx`: Angle of rotation of the rotation component around the x-axis
        - `θz`: Angle of rotation of the rotation component around the z-axis
- `steering::Steering`: Instance of a specific steering
- `suspension::Suspension`: Instance of a specific suspension

# Returns:
- no returns because of in place programming
"""
function steeringkinematics!(args...)
    steeringkinematicsMOVED!(args...)
    steeringkinematicsNEUTRAL!(args...)
    nothing
end

"""
    steeringkinematics(θ::Tuple{T,T}, steering::Steering, suspension::Suspension) where {T<:Real}

To fully describe the kinematics of the steering system, both MOVED and NEUTRAL states must be calculated

# Arguments
- `θ::Tuple{T,T}`: angles (θx,θz) in which the rotational component is rotated
        - `θx`: Angle of rotation of the rotation component around the x-axis
        - `θz`: Angle of rotation of the rotation component around the z-axis
- `steering::Steering`: Instance of a specific steering
- `suspension::Suspension`: Instance of a specific suspension

# Returns:
- `steering::Steering`: Instance of a specific steering
"""
function steeringkinematics(θ::Tuple{T,T,T}, steering::Steering, suspension::Suspension) where {T<:Any}
    cpy_steering = deepcopy(steering)
    steeringkinematics!(θ,steering,suspension)
    return cpy_steering
end