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
setQ({2.36187 , -0.224194 , -1.63451 , -3.26038 , 1.42104 , -1.23763})
setQ({1.60667 , -0.514501 , -1.48494 , -3.11683 , 0.869085 , -0.744593})
setQ({-0.558905 , -1.34697 , -1.05605 , -2.70518 , -0.713652 , 0.669207})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
