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
setQ({2.80882 , -1.0263 , -0.664992 , -1.26152 , 0.579063 , 0.275007})
setQ({1.33591 , -1.2705 , -0.774458 , -0.739853 , 0.673936 , 0.26629})
setQ({-0.333452 , -1.54727 , -0.898525 , -0.148608 , 0.781462 , 0.256412})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
