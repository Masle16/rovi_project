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
setQ({2.40204 , -1.62124 , -0.888928 , -1.73183 , 0.372753 , 0.783542})
setQ({1.65822 , -1.92801 , -0.601719 , -1.87011 , 0.281158 , 0.720353})
setQ({-0.0466812 , -2.63117 , 0.0565897 , -2.18706 , 0.0712131 , 0.575519})
setQ({-0.586563 , -2.04891 , -1.05042 , -2.53155 , 1.48454 , 0.408604})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
