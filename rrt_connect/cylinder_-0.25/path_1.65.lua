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
setQ({2.16833 , -1.54769 , -1.14676 , -0.281422 , 1.68673 , 1.26195})
setQ({1.66757 , -1.5186 , -1.16012 , -0.438038 , 1.31076 , 1.32289})
setQ({0.39472 , -1.44466 , -1.19407 , -0.836128 , 0.355104 , 1.4778})
setQ({-0.618946 , -1.19928 , -1.32775 , -1.72 , 1.00494 , 0.834848})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
