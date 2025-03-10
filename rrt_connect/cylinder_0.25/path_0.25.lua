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
setQ({1.70855 , -1.67468 , -2.08725 , -1.05823 , 1.36365 , -0.0108839})
setQ({1.71951 , -1.6387 , -1.87639 , -1.00298 , 1.26652 , -0.0751801})
setQ({1.54649 , -1.70756 , -1.78391 , -1.01205 , 1.26951 , 0.0633096})
setQ({1.37347 , -1.77642 , -1.69143 , -1.02112 , 1.2725 , 0.201799})
setQ({1.20046 , -1.84528 , -1.59895 , -1.03019 , 1.27549 , 0.340289})
setQ({1.19291 , -1.84829 , -1.59492 , -1.03059 , 1.27562 , 0.346327})
setQ({1.20632 , -1.75068 , -1.78298 , -1.15589 , 1.27276 , 0.304862})
setQ({1.1485 , -1.75347 , -1.90469 , -1.13363 , 1.38295 , 0.126823})
setQ({1.08474 , -1.70155 , -2.06671 , -1.03115 , 1.40884 , -0.00851203})
setQ({0.835991 , -1.71227 , -2.05852 , -1.02036 , 1.42686 , -0.00756625})
setQ({0.587242 , -1.72298 , -2.05033 , -1.00956 , 1.44487 , -0.00662047})
setQ({0.338493 , -1.7337 , -2.04214 , -0.998769 , 1.46289 , -0.00567469})
setQ({0.0897444 , -1.74442 , -2.03395 , -0.987974 , 1.48091 , -0.00472891})
setQ({-0.159004 , -1.75513 , -2.02576 , -0.977179 , 1.49893 , -0.00378313})
setQ({-0.407753 , -1.76585 , -2.01757 , -0.966385 , 1.51695 , -0.00283734})
setQ({-0.656502 , -1.77657 , -2.00938 , -0.95559 , 1.53496 , -0.00189156})
setQ({-0.905251 , -1.78728 , -2.00119 , -0.944795 , 1.55298 , -0.000945781})
setQ({-1.154 , -1.798 , -1.993 , -0.934 , 1.571 , 0})
