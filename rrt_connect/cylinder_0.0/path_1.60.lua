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
setQ({1.947 , -1.12988 , -1.48876 , -0.17518 , 1.96039 , 1.05536})
setQ({1.6919 , -1.18484 , -1.53024 , -0.237603 , 1.92835 , 0.96854})
setQ({0.268951 , -1.49142 , -1.76162 , -0.585802 , 1.74968 , 0.48427})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
