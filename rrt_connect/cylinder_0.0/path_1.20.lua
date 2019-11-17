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
setQ({2.32722 , -1.44883 , -1.35271 , -1.11078 , 0.867909 , 0.604106})
setQ({2.25168 , -1.45641 , -1.3666 , -1.10694 , 0.883165 , 0.590998})
setQ({1.11645 , -1.57027 , -1.5754 , -1.0493 , 1.11244 , 0.393998})
setQ({-0.0187745 , -1.68414 , -1.7842 , -0.991648 , 1.34172 , 0.196999})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
