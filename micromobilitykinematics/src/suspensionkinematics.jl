
"""
    kinematics!(lowerwishbone::LowerWishbone, upperwishbone::UpperWishbone, damper::Damper, wheelmount::WheelMount)

calculates all the movement-dependent positions of the joints
!each transfer parameter must be on the same side of the steering system

#Arguments:
-`lowerwishbone::LowerWishbone`: instant of an lowerwishbone, which is on the same steering side as the others
-`upperwishbone::UpperWishbone`: instant of an lowerwishbone, which is on the same steering side as the others
-`damper::Damper`: instant of an damper, which is on the same steering side as the others
-`wheelmount::WheelMount`: instant of an wheelmout, which is on the same steering side as the others

#Returns:
-``:
"""
function suspensionkinematics!(lowerwishbone::LowerWishbone,upperwishbone::UpperWishbone, damper::Damper, wheelmount::WheelMount)

    DamperLowerFixure = circcirc(damper.upper_fixture, 
                                    damper.length, 
                                    lowerwishbone.rotation_axis, 
                                    [damper.upper_fixture[1]; lowerwishbone.bearing_rear[2]; lowerwishbone.bearing_rear[3]], 
                                    lowerwishbone.distance_rotation_axis_to_lower_damper_fixure)

    DamperLowerFixure = DamperLowerFixure[1]
    damper.lower_fixture = DamperLowerFixure
    lowerwishbone.lower_fixture = DamperLowerFixure

    lowerwishbone.rotation_axis_TO_sphere_joint = [0;damper.lower_fixture[2];damper.lower_fixture[3]]
    lowerwishbone.rotation_axis_TO_sphere_joint =  (-lowerwishbone.bearing_rear + lowerwishbone.rotation_axis_TO_sphere_joint) / norm(-lowerwishbone.bearing_rear + lowerwishbone.rotation_axis_TO_sphere_joint)
    lowerwishbone.sphere_joint = lowerwishbone.bearing_rear + lowerwishbone.rotation_axis * lowerwishbone.distance_to_joint_x + lowerwishbone.rotation_axis_TO_sphere_joint * lowerwishbone.distance_to_joint_y

    pos_circ = upperwishbone.bearing_rear + upperwishbone.distance_to_joint_x * ((upperwishbone.bearing_front - upperwishbone.bearing_rear) / norm(upperwishbone.bearing_front - upperwishbone.bearing_rear))

    upperwishbone.sphere_joint = circsphere(pos_circ, 
                                            upperwishbone.distance_to_joint_y,          
                                            upperwishbone.rotation_axis, 
                                            lowerwishbone.sphere_joint, 
                                            wheelmount.length)[1]
end
