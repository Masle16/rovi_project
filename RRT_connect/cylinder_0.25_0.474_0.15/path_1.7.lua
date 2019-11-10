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
setQ({2.01867 , -1.29862 , -1.50968 , -1.39092 , 0.229366 , -0.526552})
setQ({1.85699 , -1.32407 , -1.53431 , -1.36764 , 0.297737 , -0.499718})
setQ({0.351493 , -1.56103 , -1.76366 , -1.15082 , 0.934369 , -0.249859})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
