wc = rws.getRobWorkStudio():getWorkCell()
state = wc:getDefaultState()
device = wc:findDevice("UR-6-85-5-A")
gripper = wc:findFrame("Tool")
bottle = wc:findFrame("Cylinder")
table = wc:findFrame("Table")

function setQ(q)
qq = rw.Q(#q,q[1],q[2],q[3],q[4],q[5],q[6])
device:setQ(qq,state)
rws.getRobWorkStudio():setState(state)
rw.sleep(0.1)
end

function attach(obj, tool)
rw.gripFrame(obj, tool, state)
rws.getRobWorkStudio():setState(state)
rw.sleep(0.1)
end

setQ({2.5 , -2.099 , -1.593 , -0.991 , 1.571 , 0})
attach(bottle, gripper)
setQ({2.26287 , -1.13761 , 0.0799033 , -1.87487 , 0.338616 , 0.413398})
setQ({2.10874 , -0.512706 , 1.16729 , -2.44938 , -0.462431 , 0.682106})
setQ({1.26303 , -1.06403 , -0.0760229 , -0.611309 , -0.973252 , 0.468657})
setQ({0.512132 , -1.2793 , -1.15998 , 0.0341033 , 0.596476 , -0.790009})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
