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

setQ({1.693 , -1.728 , -2.068 , -0.932 , 1.571 , 0})
attach(bottle, gripper)
setQ({1.25174 , -1.41857 , -1.03409 , -1.76594 , 1.93596 , 0.935102})
setQ({0.721712 , -1.40191 , -1.20602 , -1.45504 , 1.37723 , 0.797286})
setQ({-0.357557 , -1.36797 , -1.55612 , -0.821981 , 0.239494 , 0.516657})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
