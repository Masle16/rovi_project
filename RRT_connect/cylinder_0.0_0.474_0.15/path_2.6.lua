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
setQ({1.53013 , -1.34003 , -1.69863 , -2.32271 , 0.0504045 , -1.32179})
setQ({1.04782 , -1.00495 , -1.48624 , -3.35947 , -1.06949 , -2.29526})
setQ({-0.0392778 , -2.01918 , -1.34436 , -1.92782 , 0.341757 , -1.59657})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
