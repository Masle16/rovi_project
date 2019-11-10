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
setQ({2.73459 , -1.00776 , 0.0824066 , -1.69327 , -0.556512 , 1.34822})
setQ({2.36748 , -1.12453 , 0.122784 , -1.45023 , -0.108391 , 1.35253})
setQ({0.475337 , -1.7264 , 0.330895 , -0.197566 , 2.20128 , 1.37479})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
