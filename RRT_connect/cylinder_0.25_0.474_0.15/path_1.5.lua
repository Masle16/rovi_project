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
setQ({2.08833 , -0.910392 , -1.53794 , -0.0698878 , 1.66217 , -0.62667})
setQ({1.81782 , -1.04636 , -1.47131 , -0.0067561 , 1.79729 , -0.311025})
setQ({0.948735 , -1.4832 , -1.25724 , 0.196074 , 2.23143 , 0.703085})
setQ({-0.029375 , -2.09564 , -1.37572 , -0.225759 , 1.6233 , 0.106078})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
