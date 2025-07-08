"""
    kinematicsUNTILmountMOVED°!(angleConfig::Tuple{T,T,T}, steering::Steering, suspension::Suspension) where {T<:Real}

For the moving rotational component with the angles (θx, θz), the kinematics of the steering is calculated until `track_lever_mounting_points_ucs` 
! function°(): symbolises that this function should only be used within the optimisation !

# Arguments
- `angleConfig::Tuple{T,T}`: angles (θx,θz) in which the rotational component is rotated
        - `θx`: Angle of rotation of the rotation component around the x-axis
        - `θy`: Angle of rotation of the rotation component around the y-axis
        - `θz`: Angle of rotation of the rotation component around the z-axis
- `steering::Steering`: Instance of a specific steering
- `suspension::Suspension`: Instance of a specific suspension

# Returns:
- No return value due to in-place computation.
"""
function kinematicsUNTILmountMOVED°!(angleConfig::Tuple{T,T,T}, steering::Steering, suspension::Suspension) where {T<:Any}

    θx, θy, θz = angleConfig

    steering.θx = θx
    steering.θy = θy
    steering.θz = θz

    θx =    deg2rad(θx)
    θy =    deg2rad(θy)
    θz =    deg2rad(θz)

    #############
    rotational_component_ucs = [[1;0;0] [0;1;0] [0;0;1]]

    vec_x_rotational_neutral = [0.0; 0.0; -steering.rotational_component.x_rotational_radius]
    vec_z_rotational_neutral = [-steering.rotational_component.z_rotational_radius; 0.0; 0.0] + vec_x_rotational_neutral 

    left_sphere_joints_neutral = [0.0; steering.rotational_component.to_joint_pivot_point; 0] + vec_z_rotational_neutral
    right_sphere_joints_neutral = [0.0; -steering.rotational_component.to_joint_pivot_point; 0] + vec_z_rotational_neutral


    # tilting the handlebar around y-axes

    ~, vec_x_rotational_neutral = rotate3(vec_x_rotational_neutral, rotational_component_ucs[2,:], -θy)

    ~, vec_z_rotational_neutral = rotate3(vec_z_rotational_neutral, rotational_component_ucs[2,:], -θy)
    
    ~, left_sphere_joints_neutral = rotate3(left_sphere_joints_neutral, rotational_component_ucs[2,:], -θy)

    ~, right_sphere_joints_neutral = rotate3(right_sphere_joints_neutral, rotational_component_ucs[2,:], -θy)




    steering.sphere_joints_neutral = (left_sphere_joints_neutral, right_sphere_joints_neutral)

    ##############
    ~, vec_x_rotational = rotate3(vec_x_rotational_neutral, rotational_component_ucs[1,:], θx)

    ~, vec_z_rotational = rotate3(vec_z_rotational_neutral, rotational_component_ucs[1,:], θx)
    ~, vec_z_rotational = rotate3(vec_z_rotational, vec_x_rotational, θz)
    
    ~, left_sphere_joints = rotate3(steering.sphere_joints_neutral[1], rotational_component_ucs[1,:], θx)
    ~, left_sphere_joints = rotate3(left_sphere_joints, vec_x_rotational, θz)

    ~, right_sphere_joints = rotate3(steering.sphere_joints_neutral[2], rotational_component_ucs[1,:], θx)
    ~, right_sphere_joints = rotate3(right_sphere_joints, vec_x_rotational, θz)


    steering.sphere_joints = (left_sphere_joints, right_sphere_joints) 

    ###############
    wheel_ucs = [[1.0;0.0;0.0] [0.0;1.0;0.0] [0.0;0.0;1.0]]


    #suspension.kinematics!(suspension) #not necessary 


    wheel_ucs_position = (suspension.lowerwishbone[1].sphere_joint, suspension.lowerwishbone[2].sphere_joint)
    steering.wheel_ucs_position = wheel_ucs_position
    
    left_axis_z = (suspension.upperwishbone[1].sphere_joint - suspension.lowerwishbone[1].sphere_joint) / norm(suspension.upperwishbone[1].sphere_joint - suspension.lowerwishbone[1].sphere_joint)
    right_axis_z = (suspension.upperwishbone[2].sphere_joint - suspension.lowerwishbone[2].sphere_joint) / norm(suspension.upperwishbone[2].sphere_joint - suspension.lowerwishbone[2].sphere_joint)
    wheel_ucs_axis_z = (left_axis_z,  right_axis_z)

    base_vec_y, base_vec_x, base_vec_z = calc_basis_vectors(wheel_ucs_axis_z[1])
    left_base_vec = [base_vec_x base_vec_y base_vec_z]

    base_vec_y, base_vec_x, base_vec_z = calc_basis_vectors(wheel_ucs_axis_z[2])
    right_base_vec = [base_vec_x base_vec_y base_vec_z]

    steering.base_vec_wheel_ucs = (left_base_vec, right_base_vec)    # (left, right)

    ##############
    lower_end_of_rotational_component = (steering.wishbone_ucs_position[1].*[-1,-1,-1] - [0.0,0.0,steering.rotational_component.x_rotational_radius])
    
    #line
    line = Line(suspension.lowerwishbone[1].sphere_joint_neutral, left_base_vec[:,3])
 
    #plane
    plane = Plane(lower_end_of_rotational_component, [0.0,0.0,1.0])

    inter = intersection(line, plane)
    
    vec_offset = inter - suspension.lowerwishbone[1].sphere_joint_neutral
    offset_length = norm(vec_offset)
    ##############
    
    track_lever_mount = [0.0; 0.0; offset_length]
  
    mount_dict = Dict(:left_mount => Vector{<:Any}(), :right_mount => Vector{<:Any}()) 

    for (mount,index,shift) in zip([:left_mount, :right_mount], [1,2], [[1,1,1],[1,-1,1]])

        track_lever_mount_IN_wheel_ucs = applyMatrixRotation(track_lever_mount,steering.base_vec_wheel_ucs[index], wheel_ucs)

        track_lever_mount_IN_wishbone_ucs = wheel_ucs_position[index] + track_lever_mount_IN_wheel_ucs
        track_lever_mount_IN_steering_ucs = steering.wishbone_ucs_position[index] + track_lever_mount_IN_wishbone_ucs.*(shift)

        #@eval $mount = $track_lever_mount_IN_steering_ucs                                 # Not compatible with threading
        mount_dict[mount] = track_lever_mount_IN_steering_ucs
    end
    #steering.track_lever_mounting_points_ucs = (collect(left_mount), collect(right_mount))
    steering.track_lever_mounting_points_ucs = (mount_dict[:left_mount], mount_dict[:right_mount])
    nothing
end



"""
    kinematicsUNTILmountNEUTRAL°!(angleConfig::Tuple{T,T,T}, steering::Steering) where {T<:Any}

For the rotation component with the neutral position, the steering kinematics are calculated until 'track_lever_mounting_points_ucs'. 
! function°(): symbolises that this function should only be used within the optimisation !

# Arguments
- `angleConfig::Tuple{T,T}`: angles (θx,θz) in which the rotational component is rotated
        - `θx`: Angle of rotation of the rotation component around the x-axis (always = 0.0)
        - `θy`: Angle of rotation of the rotation component around the y-axis
        - `θz`: Angle of rotation of the rotation component around the z-axis (always = 0.0)
- `steering::Steering`: Instance of a specific steering 

# Returns:
- No return value due to in-place computation.
"""
function kinematicsUNTILmountNEUTRAL°!(angleConfig::Tuple{T,T,T}, steering::Steering) where {T<:Any}

    θx, θy ,θz = angleConfig

    θx =    deg2rad(0.0)
    θy =    deg2rad(θy)
    θz =    deg2rad(0.0)

    #############
    rotational_component_ucs = [[1;0;0] [0;1;0] [0;0;1]]

    vec_x_rotational_neutral = [0.0; 0.0; -steering.rotational_component.x_rotational_radius]
    vec_z_rotational_neutral = [-steering.rotational_component.z_rotational_radius; 0.0; 0.0] + vec_x_rotational_neutral 

    left_sphere_joints_neutral = [0.0; steering.rotational_component.to_joint_pivot_point; 0] + vec_z_rotational_neutral
    right_sphere_joints_neutral = [0.0; -steering.rotational_component.to_joint_pivot_point; 0] + vec_z_rotational_neutral


    # tilting the handlebar around y-axes

    ~, vec_x_rotational_neutral = rotate3(vec_x_rotational_neutral, rotational_component_ucs[2,:], -θy)

    ~, vec_z_rotational_neutral = rotate3(vec_z_rotational_neutral, rotational_component_ucs[2,:], -θy)
        
    ~, left_sphere_joints_neutral = rotate3(left_sphere_joints_neutral, rotational_component_ucs[2,:], -θy)
    
    ~, right_sphere_joints_neutral = rotate3(right_sphere_joints_neutral, rotational_component_ucs[2,:], -θy)


    steering.sphere_joints_neutral = (left_sphere_joints_neutral, right_sphere_joints_neutral)

    ##############
    ~, vec_x_rotational = rotate3(vec_x_rotational_neutral, rotational_component_ucs[1,:], θx)

    ~, vec_z_rotational = rotate3(vec_z_rotational_neutral, rotational_component_ucs[1,:], θx)
    ~, vec_z_rotational = rotate3(vec_z_rotational, vec_x_rotational, θz)
    
    ~, left_sphere_joints = rotate3(steering.sphere_joints_neutral[1], rotational_component_ucs[1,:], θx)
    ~, left_sphere_joints = rotate3(left_sphere_joints, vec_x_rotational, θz)

    ~, right_sphere_joints = rotate3(steering.sphere_joints_neutral[2], rotational_component_ucs[1,:], θx)
    ~, right_sphere_joints = rotate3(right_sphere_joints, vec_x_rotational, θz)


    steering.sphere_joints_neutral = (left_sphere_joints, right_sphere_joints) 
    nothing
end

"""
    kinematicsUNTILmount°!(angleConfig::Tuple{T,T,T}, steering::Steering, suspension::Suspension) where {T<:Real}

For the moving rotation component with the angles (θx, θz) and the neutral position of the rotation component, the steering kinematics are calculated until `track_lever_mounting_points_ucs`.    
    ! function°(): symbolises that this function should only be used within the optimisation !

# Arguments
- `angleConfig::Tuple{T,T}`: angles (θx,θz) in which the rotational component is rotated
        - `θx`: Angle of rotation of the rotation component around the x-axis
        - `θy`: Angle of rotation of the rotation component around the y-axis
        - `θz`: Angle of rotation of the rotation component around the z-axis
- `steering::Steering`: Instance of a specific steering 
- `suspension::Suspension`: Instance of a specific suspension

# Returns:
- No return value due to in-place computation.
"""
function kinematicsUNTILmount°!(angleConfig::Tuple{T,T,T}, steering::Steering, suspension::Suspension) where {T<:Any}
    kinematicsUNTILmountMOVED°!(angleConfig, steering, suspension)
    kinematicsUNTILmountNEUTRAL°!(angleConfig, steering)
    nothing
end

"""
    kinematicsUNTILmount°(angleConfig::Tuple{T,T,T}, steering::Steering, suspension::Suspension) where {T<:Real}

For the moving rotation component with the angles (θx, θz) and the neutral position of the rotation component, the steering kinematics are calculated until `track_lever_mounting_points_ucs`.    
! function°(): symbolises that this function should only be used within the optimisation !

# Arguments
- `angleConfig::Tuple{T,T}`: angles (θx,θz) in which the rotational component is rotated
        - `θx`: Angle of rotation of the rotation component around the x-axis
        - `θy`: Angle of rotation of the rotation component around the y-axis
        - `θz`: Angle of rotation of the rotation component around the z-axis
- `steering::Steering`: Instance of a specific steering 
- `suspension::Suspension`: Instance of a specific suspension

# Returns:
- `steering::Steering`: Instance of a steering with calculated kinematics until `track_lever_mounting_points_ucs`.
"""
function kinematicsUNTILmount°(angleConfig::Tuple{T,T,T}, steering::Steering, suspension::Suspension) where {T<:Any}
    cpy_steering = deepcopy(steering)
    kinematicsUNTILmount°!(angleConfig, cpy_steering, suspension)
    return cpy_steering
end


"""
    kinematicsASOFmount°!(steering::Steering)

For the moving rotational component with angles (θx, θz) and the neutral position of the rotational component, the steering kinematics are calculated as of `track_lever_mounting_points_ucs`. 
! function°(): symbolises that this function should only be used within the optimisation !
! function kinematicsUNTILmountMOVED°! should already be used on the steering instance !

# Arguments
- `steering::Steering`: Instance of a specific steering where the function was previously called

# Returns:
- No return value due to in-place computation.
"""
function kinematicsASOFmountMOVED°!(steering::Steering)

    circle_joints_dict = Dict(:left_joint => Vector{<:Any}(), :right_joint => Vector{<:Any}()) 

    for (circle_joint,index,shift) in zip([:left_joint, :right_joint], [1,2], [[1,1,1],[1,-1,1]])
        mount = steering.track_lever_mounting_points_ucs[index]

        #Circle 
        #@show steering.base_vec_wheel_ucs
        circ = GeoSpatialRelations.Circle(mount,steering.track_lever.length,steering.base_vec_wheel_ucs[index][:,3])

        #Sphere
        sphere = GeoSpatialRelations.Sphere(steering.sphere_joints[index],steering.tie_rod.length)


        circle_joints_1, circle_joints_2 = GeoSpatialRelations.intersection(circ, sphere)
        

        if circle_joints_2[1] - mount[1] < circle_joints_1[1] - mount[1]                          
            #@eval $circle_joint = $circle_joints_2                         # Not compatible with threading
            circle_joints_dict[circle_joint] = circle_joints_2
        else
            #@eval $circle_joint = $circle_joints_1                         # Not compatible with threading
            circle_joints_dict[circle_joint] = circle_joints_1
        end
    end
    #steering.circle_joints = (collect(left_joint), collect(right_joint))
    steering.circle_joints = (circle_joints_dict[:left_joint],circle_joints_dict[:right_joint])
    nothing
end

"""
    kinematicsASOFmountNEUTRAL°!(steering::Steering)

for the moving rotational component with the angles (θx, θz), the kinematics of the steering is calculated as of `track_lever_mounting_points_ucs` 
! function°(): symbolises that this function should only be used within the optimisation !
! function kinematicsUNTILmountMOVED°! should already be used on the steering instance !

# Arguments
-`steering::Steering`: Instance of a specific steering where the function `kinematicsUNTILmount°!` was previously called

# Returns:
- No return value due to in-place computation.
"""
function kinematicsASOFmountNEUTRAL°!(steering::Steering)

    circle_joints_dict = Dict(:left_joint => Vector{<:Any}(), :right_joint => Vector{<:Any}()) 

    for (circle_joint,index,shift) in zip([:left_joint, :right_joint], [1,2], [[1,1,1],[1,-1,1]])

        # in Neutral and Moved always the same
        mount = steering.track_lever_mounting_points_ucs[index]

        #Circle 
        circ = GeoSpatialRelations.Circle(mount,steering.track_lever.length,steering.base_vec_wheel_ucs[index][:,3])

        #Sphere
        sphere = GeoSpatialRelations.Sphere(steering.sphere_joints_neutral[index],steering.tie_rod.length)

        circle_joints_1, circle_joints_2 = GeoSpatialRelations.intersection(circ, sphere)
        
        if circle_joints_2[1] - mount[1] < circle_joints_1[1] - mount[1]
            #@eval $circle_joint = circle_joints_2                                      # Not compatible with threading
            circle_joints_dict[circle_joint] = circle_joints_2
        else
            #@eval $circle_joint = circle_joints_1                                      # Not compatible with threading
            circle_joints_dict[circle_joint] = circle_joints_1                          
        end
    end
    #steering.circle_joints_neutral = (collect(left_joint), collect(right_joint))       
    steering.circle_joints_neutral = (circle_joints_dict[:left_joint],circle_joints_dict[:right_joint])
    nothing
end



"""
    kinematicsASOFmount°!(steering::Steering)

For the moving rotational component with angles (θx, θz) and the neutral position of the rotational component, the steering kinematics are calculated from `track_lever_mounting_points_ucs`. 
! function°(): symbolises that this function should only be used within the optimisation !
! function kinematicsUNTILmountMOVED°! should already be used on the steering instance !

# Arguments
- `steering::Steering`: Instance of a specific steering where the function `kinematicsUNTILmount°!` was previously called

# Returns:
- No return value due to in-place computation.
"""
function kinematicsASOFmount°!(steering::Steering)
    kinematicsASOFmountMOVED°!(steering)
    kinematicsASOFmountNEUTRAL°!(steering)
    nothing
end



"""
    update°!(steering::Steering)

    Determines the further kinematic calculation and determines the turning angles of the wheels
! function°(): symbolises that this function should only be used within the optimisation !
! function kinematicsUNTILmountMOVED°! should already be used on the steering instance !

# Arguments
- `steering::Steering`: Instance of a specific steering where the function `kinematicsUNTILmount°!` was previously called

# Returns:
- No return value due to in-place computation.
"""
function update°!(steering::Steering)
    kinematicsASOFmount°!(steering)
    angle_δi!(steering)
    angle_δo!(steering)
    nothing
end