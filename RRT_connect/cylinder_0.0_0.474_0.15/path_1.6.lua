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
setQ({2.03337 , -1.50192 , -1.98515 , -1.86166 , 0.9085 , -1.05643})
setQ({1.23427 , -1.76281 , -1.32414 , -0.693714 , 0.720065 , -0.926679})
setQ({1.09666 , -1.80774 , -1.21032 , -0.492591 , 0.687616 , -0.904336})
setQ({0.279151 , -1.66487 , -1.98947 , -1.35111 , 1.27312 , -0.475006})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
