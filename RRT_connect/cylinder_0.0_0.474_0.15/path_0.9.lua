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
setQ({1.75537 , -1.33313 , -1.70358 , -0.53161 , 1.37344 , -0.381764})
setQ({1.44843 , -1.38217 , -1.73412 , -0.574061 , 1.39428 , -0.341489})
setQ({0.580956 , -1.52078 , -1.82041 , -0.694041 , 1.45319 , -0.227659})
setQ({-0.286522 , -1.65939 , -1.90671 , -0.81402 , 1.51209 , -0.11383})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
