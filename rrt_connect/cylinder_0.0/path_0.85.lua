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

setQ({2.185 , -1.795 , -1.987 , -0.915 , 1.571 , 0})
attach(bottle, gripper)
setQ({2.3946 , -1.6984 , -1.30609 , -0.665813 , 1.57785 , -0.378761})
setQ({1.92535 , -1.69488 , -1.34674 , -0.684651 , 1.5325 , -0.373131})
setQ({1.08317 , -1.68855 , -1.4197 , -0.718461 , 1.45109 , -0.363028})
setQ({0.240992 , -1.68223 , -1.49266 , -0.752272 , 1.36969 , -0.352925})
setQ({-0.601187 , -1.6759 , -1.56562 , -0.786082 , 1.28828 , -0.342822})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
