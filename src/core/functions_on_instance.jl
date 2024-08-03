"""
    copy(steering::Steering)

creates a copy of the steering instance

#Arguments:
-`steering::Steering`: instance of a Steering

#Returns:
-`copy::Steering`: copy of the given instance 

"""
function copy(steering::Steering)
    comp = (steering.rotational_component.x_rotational_radius,steering.rotational_component.z_rotational_radius, steering.track_lever.length, steering.tie_rod.length)
    
    copy = Steering(comp...)

    copy.θx = steering.θx
    copy.θz = steering.θz

    copy.δi = steering.δi
    copy.δo = steering.δo

    copy.θx_max = steering.θx_max
    copy.θz_max = steering.θz_max
    
    copy.δi_max = steering.δi_max
    copy.δo_max = steering.δo_max



    copy.sphere_joints = steering.sphere_joints
    copy.sphere_joints_neutral = steering.sphere_joints_neutral

    copy.circle_joints = steering.circle_joints
    copy.circle_joints_neutral = steering.circle_joints_neutral

    copy.wishbone_ucs_position = steering.wishbone_ucs_position
    copy.track_lever_mounting_points_ucs = steering.track_lever_mounting_points_ucs

    return copy 
end


#for (func,index) in zip([:angle_δo, :angle_δi], [1,2])
#        """
#            $func(steering::Steering)
#    
#            calculates inner/outer steering angle
#    
#        """
#    @eval function $func(steering::Steering)
#
#        a = steering.circle_joints[$index] - steering.track_lever_mounting_points_ucs[$index]
#        
#        b = steering.circle_joints_neutral[$index] - steering.track_lever_mounting_points_ucs[$index]
#        
#        δ = acosd(dot(a,b)/(norm(a) * norm(b)))
#
#        return δ
#    end
#
#end


"""
    angle_δo!(steering::Steering)
    
calculates the outer steering angle of the wheel
!only posible after using function kinematics!

#Arguments
-`steering::Steering`: Instance of a specific steering

#Returns:
- no returns becouse of in place programming

    
"""
function angle_δo!(steering::Steering)
    steering.δo =  angle_δo(steering)
end


"""
    angle_δo(steering::Steering)
    
calculates the outer steering angle of the wheel
!only posible after using function kinematics!

#Arguments
-`steering::Steering`: Instance of a specific steering

#Returns 
-`δ`: inner steering angle  of the wheel 
    
"""
function angle_δo(steering::Steering)

    a = steering.circle_joints[1] - steering.track_lever_mounting_points_ucs[1]
        
    b = steering.circle_joints_neutral[1] - steering.track_lever_mounting_points_ucs[1]
    
    δ = acosd(dot(a,b)/(norm(a) * norm(b)))

    return δ
end




"""
    angle_δi!(steering::Steering)

calculates the inner steering angle of the wheel
!only posible after using function kinematics!

#Arguments
-`steering::Steering`: Instance of a specific steering

#Returns:
- no returns becouse of in place programming
    
"""
function angle_δi!(steering::Steering)
    steering.δi =  angle_δi(steering)
end



"""
    angle_δi(steering::Steering)

calculates the inner steering angle of the wheel
!only posible after using function kinematics!

#Arguments
-`steering::Steering`: Instance of a specific steering

#Returns 
-`δ`: inner steering angle  of the wheel 
    
"""
function angle_δi(steering::Steering)

    a = steering.circle_joints[2] - steering.track_lever_mounting_points_ucs[2] 

    b = steering.circle_joints_neutral[2] - steering.track_lever_mounting_points_ucs[2]
    
    δ = acosd(dot(a,b)/(norm(a) * norm(b)))

    return δ
end

"""
    update!(angleConfig::Tuple{T,T}, steering::Steering, suspension::Suspension) where {T<:Real}

updates the kinematics of the given steering instance on the new angles and suspension

#Arguemnts
-`angleConfig::Tuple{T,T}`: angles (θx,θz) in which the rotational component ist rotated
        -`θx`: Angle of rotation of the rotation component around the x-axis
        -`θz`: Angle of rotation of the rotation component around the z-axis
-`steering::Steering`: Instance of a specific steering
-`suspension::Suspension`: Instance of a specific suspension

#Returns:
- no returns becouse of in place programming
"""
function update!(args...)
    steeringkinematicsNEUTRAL!(args...)
    steeringkinematicsMOVED!(args...)
    angle_δi!(args[2])
    angle_δo!(args[2])
    nothing
end 

"""
    update!(angleConfig::Tuple{T,T}, steering::Steering, suspension::Suspension) where {T<:Real}

updates the kinematics of the given steering instance on the new angles and suspension

#Arguemnts
-`angleConfig::Tuple{T,T}`: angles (θx,θz) in which the rotational component ist rotated
        -`θx`: Angle of rotation of the rotation component around the x-axis
        -`θz`: Angle of rotation of the rotation component around the z-axis
-`steering::Steering`: Instance of a specific steering
-`suspension::Suspension`: Instance of a specific suspension

#Returns
-`steering::Steering`: Instance of a specific steering

"""
function update(args...)
    update!(args...)
    return steering
end 