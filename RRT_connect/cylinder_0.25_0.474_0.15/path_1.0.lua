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
setQ({1.20873 , -1.58786 , -1.52605 , -0.752938 , 1.91221 , -0.551045})
setQ({0.694915 , -1.56899 , -1.43551 , -0.964791 , 1.41459 , -0.450553})
setQ({0.0174135 , -1.54411 , -1.31613 , -1.24414 , 0.758446 , -0.318046})
setQ({-0.517704 , -1.7785 , -1.84441 , -1.43673 , 1.04005 , 0.195012})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
