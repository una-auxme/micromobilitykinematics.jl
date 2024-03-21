
"""
    suspensionkinematicsMOVED!(suspension::Suspension)

calculates all the movement-dependent positions of the joints in MOVED-Position

#Arguments:
-`suspension::Suspension`: Suspension of the micromobility vihicle


"""
function suspensionkinematicsMOVED!(suspension::Suspension)
    
    #left

    dampel_lower_fixure = circcirc(suspension.damper[1].upper_fixture, 
                                    suspension.damper[1].length, 
                                    suspension.lowerwishbone[1].rotation_axis, 
                                    [suspension.damper[1].upper_fixture[1]; suspension.lowerwishbone[1].bearing_rear[2]; suspension.lowerwishbone[1].bearing_rear[3]], 
                                    suspension.lowerwishbone[1].distance_rotation_axis_to_lower_damper_fixure)

    dampel_lower_fixure = dampel_lower_fixure[1]
    suspension.damper[1].lower_fixture = dampel_lower_fixure
    suspension.lowerwishbone[1].lower_fixture = dampel_lower_fixure

    rotation_axis_TO_sphere_joint = [0;suspension.damper[1].lower_fixture[2]; suspension.damper[1].lower_fixture[3]]
    rotation_axis_TO_sphere_joint =  (-suspension.lowerwishbone[1].bearing_rear + rotation_axis_TO_sphere_joint) / norm(-suspension.lowerwishbone[1].bearing_rear + rotation_axis_TO_sphere_joint)
    suspension.lowerwishbone[1].sphere_joint = suspension.lowerwishbone[1].bearing_rear + suspension.lowerwishbone[1].rotation_axis * suspension.lowerwishbone[1].distance_to_joint_x + rotation_axis_TO_sphere_joint * suspension.lowerwishbone[1].distance_to_joint_y

    pos_circ = suspension.upperwishbone[1].bearing_rear + suspension.upperwishbone[1].distance_to_joint_x * ((suspension.upperwishbone[1].bearing_front - suspension.upperwishbone[1].bearing_rear) / norm(suspension.upperwishbone[1].bearing_front - suspension.upperwishbone[1].bearing_rear))

    suspension.upperwishbone[1].sphere_joint = circsphere(pos_circ, 
                                                            suspension.upperwishbone[1].distance_to_joint_y,          
                                                            suspension.upperwishbone[1].rotation_axis, 
                                                            suspension.lowerwishbone[1].sphere_joint, 
                                                            suspension.wheelmount.length)[1]


    #right

    dampel_lower_fixure = circcirc(suspension.damper[2].upper_fixture, 
                                    suspension.damper[2].length, 
                                    suspension.lowerwishbone[2].rotation_axis, 
                                    [suspension.damper[2].upper_fixture[1]; suspension.lowerwishbone[2].bearing_rear[2]; suspension.lowerwishbone[2].bearing_rear[3]], 
                                    suspension.lowerwishbone[2].distance_rotation_axis_to_lower_damper_fixure)

    dampel_lower_fixure = dampel_lower_fixure[1]
    suspension.damper[2].lower_fixture = dampel_lower_fixure
    suspension.lowerwishbone[2].lower_fixture = dampel_lower_fixure

    rotation_axis_TO_sphere_joint = [0;suspension.damper[2].lower_fixture[2]; suspension.damper[2].lower_fixture[3]]
    rotation_axis_TO_sphere_joint =  (-suspension.lowerwishbone[2].bearing_rear + rotation_axis_TO_sphere_joint) / norm(-suspension.lowerwishbone[2].bearing_rear + rotation_axis_TO_sphere_joint)
    suspension.lowerwishbone[2].sphere_joint = suspension.lowerwishbone[2].bearing_rear + suspension.lowerwishbone[2].rotation_axis * suspension.lowerwishbone[2].distance_to_joint_x + rotation_axis_TO_sphere_joint * suspension.lowerwishbone[2].distance_to_joint_y

    pos_circ = suspension.upperwishbone[2].bearing_rear + suspension.upperwishbone[2].distance_to_joint_x * ((suspension.upperwishbone[2].bearing_front - suspension.upperwishbone[2].bearing_rear) / norm(suspension.upperwishbone[2].bearing_front - suspension.upperwishbone[2].bearing_rear))

    suspension.upperwishbone[2].sphere_joint = circsphere(pos_circ, 
                            suspension.upperwishbone[2].distance_to_joint_y,          
                            suspension.upperwishbone[2].rotation_axis, 
                            suspension.lowerwishbone[2].sphere_joint, 
                            suspension.wheelmount.length)[1]

end


"""
    suspensionkinematicsNEUTRAL!(suspension::Suspension)

calculates all the movement-dependent positions of the joints in NEUTRAL-Position

#Arguemnts:
-`suspension::Suspension`: Suspension of the micromobility vihicle


"""
function suspensionkinematicsNEUTRAL!(suspension::Suspension)
    
    

    dampel_lower_fixure = circcirc(suspension.damper[1].upper_fixture, 
                                    suspension.damper[1].length_neutral, 
                                    suspension.lowerwishbone[1].rotation_axis, 
                                    [suspension.damper[1].upper_fixture[1]; suspension.lowerwishbone[1].bearing_rear[2]; suspension.lowerwishbone[1].bearing_rear[3]], 
                                    suspension.lowerwishbone[1].distance_rotation_axis_to_lower_damper_fixure)

    dampel_lower_fixure = dampel_lower_fixure[1]
    suspension.damper[1].lower_fixture_neutral = dampel_lower_fixure
    suspension.lowerwishbone[1].lower_fixture_neutral = dampel_lower_fixure

    rotation_axis_TO_sphere_joint = [0;suspension.damper[1].lower_fixture_neutral[2]; suspension.damper[1].lower_fixture_neutral[3]]
    rotation_axis_TO_sphere_joint =  (-suspension.lowerwishbone[1].bearing_rear + rotation_axis_TO_sphere_joint) / norm(-suspension.lowerwishbone[1].bearing_rear + rotation_axis_TO_sphere_joint)
    suspension.lowerwishbone[1].sphere_joint_neutral = suspension.lowerwishbone[1].bearing_rear + suspension.lowerwishbone[1].rotation_axis * suspension.lowerwishbone[1].distance_to_joint_x + rotation_axis_TO_sphere_joint * suspension.lowerwishbone[1].distance_to_joint_y

    pos_circ = suspension.upperwishbone[1].bearing_rear + suspension.upperwishbone[1].distance_to_joint_x * ((suspension.upperwishbone[1].bearing_front - suspension.upperwishbone[1].bearing_rear) / norm(suspension.upperwishbone[1].bearing_front - suspension.upperwishbone[1].bearing_rear))

    suspension.upperwishbone[1].sphere_joint_neutral = circsphere(pos_circ, 
                                                            suspension.upperwishbone[1].distance_to_joint_y,          
                                                            suspension.upperwishbone[1].rotation_axis, 
                                                            suspension.lowerwishbone[1].sphere_joint_neutral, 
                                                            suspension.wheelmount.length)[1]


    #right

    dampel_lower_fixure = circcirc(suspension.damper[2].upper_fixture, 
                                    suspension.damper[2].length_neutral, 
                                    suspension.lowerwishbone[2].rotation_axis, 
                                    [suspension.damper[2].upper_fixture[1]; suspension.lowerwishbone[2].bearing_rear[2]; suspension.lowerwishbone[2].bearing_rear[3]], 
                                    suspension.lowerwishbone[2].distance_rotation_axis_to_lower_damper_fixure)

    dampel_lower_fixure = dampel_lower_fixure[1]
    suspension.damper[2].lower_fixture_neutral = dampel_lower_fixure
    suspension.lowerwishbone[2].lower_fixture_neutral = dampel_lower_fixure

    rotation_axis_TO_sphere_joint = [0;suspension.damper[2].lower_fixture_neutral[2]; suspension.damper[2].lower_fixture_neutral[3]]
    rotation_axis_TO_sphere_joint =  (-suspension.lowerwishbone[2].bearing_rear + rotation_axis_TO_sphere_joint) / norm(-suspension.lowerwishbone[2].bearing_rear + rotation_axis_TO_sphere_joint)
    suspension.lowerwishbone[2].sphere_joint_neutral = suspension.lowerwishbone[2].bearing_rear + suspension.lowerwishbone[2].rotation_axis * suspension.lowerwishbone[2].distance_to_joint_x + rotation_axis_TO_sphere_joint * suspension.lowerwishbone[2].distance_to_joint_y

    pos_circ = suspension.upperwishbone[2].bearing_rear + suspension.upperwishbone[2].distance_to_joint_x * ((suspension.upperwishbone[2].bearing_front - suspension.upperwishbone[2].bearing_rear) / norm(suspension.upperwishbone[2].bearing_front - suspension.upperwishbone[2].bearing_rear))

    suspension.upperwishbone[2].sphere_joint_neutral = circsphere(pos_circ, 
                            suspension.upperwishbone[2].distance_to_joint_y,          
                            suspension.upperwishbone[2].rotation_axis, 
                            suspension.lowerwishbone[2].sphere_joint_neutral, 
                            suspension.wheelmount.length)[1]

end

"""
    suspensionkinematics!(suspension::Suspension)

    suspensionkinematicsNEUTRAL!(suspension::Suspension)

    calculates all the movement-dependent positions of the joints in NEUTRAL- and MOVED-Postion
    
#Arguments:
-`suspension::Suspension`: Suspension of the micromobility vihicle
"""
function suspensionkinematics!(suspension::Suspension)
    suspensionkinematicsNEUTRAL!(suspension)
    suspensionkinematicsMOVED!(suspension)
end