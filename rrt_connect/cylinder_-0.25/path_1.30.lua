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
setQ({2.1377 , -1.58312 , -1.53452 , -1.07936 , 1.46837 , -1.12732})
setQ({1.277 , -1.63931 , -1.6544 , -1.04135 , 1.49521 , -0.832553})
setQ({0.0614996 , -1.71865 , -1.8237 , -0.987676 , 1.5331 , -0.416276})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
