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
setQ({2.53668 , -1.75576 , -1.74609 , -1.32222 , 1.3509 , -0.348936})
setQ({2.42524 , -1.75205 , -1.73929 , -1.29233 , 1.33893 , -0.335182})
setQ({1.80648 , -1.73143 , -1.70154 , -1.12636 , 1.27247 , -0.258814})
setQ({1.18773 , -1.71082 , -1.66378 , -0.960402 , 1.206 , -0.182447})
setQ({0.568974 , -1.6902 , -1.62603 , -0.794439 , 1.13954 , -0.106079})
setQ({-0.0497806 , -1.66958 , -1.58828 , -0.628477 , 1.07307 , -0.0297121})
setQ({-0.668535 , -1.64897 , -1.55053 , -0.462514 , 1.00661 , 0.0466553})
setQ({-0.796151 , -1.71544 , -1.65206 , -0.785531 , 1.38113 , -0.336593})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
