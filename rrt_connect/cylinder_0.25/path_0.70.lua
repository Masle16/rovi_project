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
setQ({1.84457 , -1.50851 , -1.52617 , -1.21406 , 1.54995 , -0.212748})
setQ({1.20155 , -1.63177 , -1.39332 , -1.29123 , 1.46843 , -0.30278})
setQ({0.542904 , -1.75804 , -1.25724 , -1.37026 , 1.38493 , -0.395})
setQ({-0.115743 , -1.8843 , -1.12116 , -1.4493 , 1.30142 , -0.48722})
setQ({-0.541352 , -1.74559 , -1.51261 , -1.50568 , 1.10659 , -0.178618})
setQ({-1.06613 , -1.69077 , -1.92462 , -1.36905 , 1.15551 , -0.322805})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
