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
setQ({2.31686 , -1.43228 , -0.916512 , -2.50234 , 1.36469 , 1.25602})
setQ({1.6477 , -1.59509 , -0.726909 , -2.27089 , 1.37208 , 1.04223})
setQ({-0.237867 , -2.05385 , -0.192641 , -1.61869 , 1.39288 , 0.439795})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
