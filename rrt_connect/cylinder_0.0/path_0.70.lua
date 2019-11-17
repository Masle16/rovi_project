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
setQ({1.8306 , -1.60122 , -1.4993 , -1.02949 , 1.43526 , 0.23972})
setQ({1.58997 , -1.61709 , -1.5391 , -1.02179 , 1.4462 , 0.220393})
setQ({0.90398 , -1.66232 , -1.65258 , -0.999845 , 1.4774 , 0.165295})
setQ({0.217987 , -1.70754 , -1.76605 , -0.977897 , 1.5086 , 0.110197})
setQ({-0.468007 , -1.75277 , -1.87953 , -0.955948 , 1.5398 , 0.0550983})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
