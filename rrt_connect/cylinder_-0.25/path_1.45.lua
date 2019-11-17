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
setQ({2.23465 , -1.86038 , -1.59641 , -2.20865 , 1.49976 , 0.69814})
setQ({1.96346 , -1.75438 , -1.57312 , -2.25258 , 1.17714 , 0.688972})
setQ({1.06469 , -1.40307 , -1.49594 , -2.3982 , 0.107929 , 0.658586})
setQ({0.165922 , -1.05177 , -1.41876 , -2.54381 , -0.961284 , 0.6282})
setQ({-0.74908 , -1.29825 , -1.96858 , -1.85178 , -0.312173 , 0.58399})
setQ({-1.33313 , -1.58423 , -1.88172 , -0.692943 , 0.236906 , 0.41763})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
