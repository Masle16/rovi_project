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
setQ({2.47997 , -1.56204 , -0.604423 , -1.43827 , 0.457286 , -0.950566})
setQ({2.34053 , -1.57104 , -0.536983 , -1.35838 , 0.454107 , -0.907572})
setQ({0.866646 , -1.66617 , 0.175824 , -0.514052 , 0.420509 , -0.453145})
setQ({-0.607236 , -1.76131 , 0.88863 , 0.330279 , 0.386911 , 0.00128276})
setQ({-1.04396 , -1.60571 , -0.610924 , -0.245841 , 0.826643 , 0.789399})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
