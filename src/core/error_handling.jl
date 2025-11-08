
function capture_error!(steering::Steering, err::Any, bt::Any)
    # Backtrace as StackFrame-list

    bt_frames = Base.stacktrace(bt)   
    steering.err_info.type =  typeof(err)
    steering.err_info.msg = err.msg#sprint(showerror, err, bt)
    steering.err_info.backtrace = bt_frames
end



function capture_error!(suspension::Suspension, err::Any, bt::Any)
    # Backtrace as StackFrame-list
    bt_frames = Base.stacktrace(bt)  
    suspension.err_info.type =  typeof(err)
    suspension.err_info.msg = err.msg#sprint(showerror, err, bt)
    suspension.err_info.backtrace = bt_frames
end