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
setQ({1.58157 , -1.19737 , -1.65145 , -2.22751 , 1.06212 , 1.26301})
setQ({1.49163 , -0.769091 , -1.31525 , -3.27312 , 0.651409 , 2.28239})
setQ({0.171727 , -1.76631 , -0.972741 , -2.35491 , 0.100973 , 2.28143})
setQ({-0.559738 , -2.17665 , -1.17307 , -2.13677 , 0.637168 , 0.715896})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
