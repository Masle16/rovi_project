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
setQ({2.52704 , -1.40147 , -1.09569 , -1.0974 , 0.890619 , -0.0341753})
setQ({1.98212 , -1.46017 , -1.22853 , -1.07321 , 0.99134 , -0.0291161})
setQ({0.936743 , -1.57278 , -1.48335 , -1.02681 , 1.18456 , -0.0194107})
setQ({-0.108628 , -1.68539 , -1.73818 , -0.980404 , 1.37778 , -0.00970536})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
