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
setQ({2.35497 , -1.48907 , -1.93106 , -0.463646 , 1.37216 , 0.418356})
setQ({1.78803 , -1.49418 , -1.77654 , -0.525612 , 1.36407 , 0.308112})
setQ({0.844939 , -1.50269 , -1.51949 , -0.62869 , 1.35061 , 0.124724})
setQ({-0.0981509 , -1.5112 , -1.26245 , -0.731768 , 1.33715 , -0.0586632})
setQ({-0.856381 , -1.9903 , -1.28439 , -0.796745 , 1.21923 , -0.47931})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
