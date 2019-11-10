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
setQ({1.79698 , -1.51164 , -1.76977 , -1.22736 , 1.2591 , -0.398645})
setQ({1.57276 , -1.5334 , -1.78673 , -1.20507 , 1.2828 , -0.368355})
setQ({0.89107 , -1.59955 , -1.8383 , -1.1373 , 1.35485 , -0.276267})
setQ({0.20938 , -1.6657 , -1.88987 , -1.06953 , 1.4269 , -0.184178})
setQ({-0.47231 , -1.73185 , -1.94143 , -1.00177 , 1.49895 , -0.0920888})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
