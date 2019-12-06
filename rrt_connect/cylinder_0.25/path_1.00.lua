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
setQ({1.87134 , -1.70265 , -1.86309 , -1.67856 , 1.02041 , -0.255063})
setQ({1.2193 , -1.69811 , -1.28502 , -1.75958 , 0.635463 , -0.548187})
setQ({1.09655 , -1.69726 , -1.17619 , -1.77483 , 0.562995 , -0.603369})
setQ({0.787755 , -1.74428 , -1.64058 , -1.62405 , 1.17337 , -0.063489})
setQ({-0.166855 , -1.78096 , -1.44457 , -1.57602 , 1.30814 , 0.105283})
setQ({-1.12146 , -1.81764 , -1.24855 , -1.52799 , 1.4429 , 0.274056})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
