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
setQ({2.52914 , -1.706 , -1.13923 , -0.863563 , 1.83703 , -0.437936})
setQ({1.93368 , -1.72088 , -1.27726 , -0.874951 , 1.79402 , -0.367134})
setQ({1.16176 , -1.74016 , -1.4562 , -0.889713 , 1.73827 , -0.27535})
setQ({0.389839 , -1.75944 , -1.63513 , -0.904475 , 1.68251 , -0.183567})
setQ({-0.38208 , -1.77872 , -1.81407 , -0.919238 , 1.62676 , -0.0917835})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
