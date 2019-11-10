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
setQ({2.3136 , -0.628221 , -0.730946 , -1.29121 , 0.948832 , 1.6916})
setQ({1.7633 , -0.824837 , -0.541298 , -1.12531 , 1.31001 , 1.38277})
setQ({-0.0381628 , -1.46848 , 0.0795346 , -0.582205 , 2.49238 , 0.371793})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
