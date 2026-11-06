-- Controller callbacks execute at MuJoCo's physics-step rate. Units: m, rad, N, Nm.
sim = require 'sim'
simIK = require 'simIK'
local cfg, command, groups, force, torques

local function cross(a,b)
    return {a[2]*b[3]-a[3]*b[2],a[3]*b[1]-a[1]*b[3],a[1]*b[2]-a[2]*b[1]}
end
local function rotate(m,v)
    return {m[1]*v[1]+m[2]*v[2]+m[3]*v[3],m[5]*v[1]+m[6]*v[2]+m[7]*v[3],m[9]*v[1]+m[10]*v[2]+m[11]*v[3]}
end
local function quatError(current,goal)
    local a,b,c,d=goal[4],goal[5],goal[6],goal[7]
    local x,y,z,w=-current[4],-current[5],-current[6],current[7]
    local q={d*x+a*w+b*z-c*y,d*y-a*z+b*w+c*x,d*z+a*y-b*x+c*w,d*w-a*x-b*y-c*z}
    if q[4]<0 then for i=1,4 do q[i]=-q[i] end end
    local n=math.sqrt(q[1]^2+q[2]^2+q[3]^2)
    local s=n>1e-9 and 2*math.atan2(n,q[4])/n or 2
    return {q[1]*s,q[2]*s,q[3]*s}
end
local function solveLinear(a,b)
    for i=1,6 do
        local pivot=i
        for j=i+1,6 do if math.abs(a[j][i])>math.abs(a[pivot][i]) then pivot=j end end
        a[i],a[pivot]=a[pivot],a[i]; b[i],b[pivot]=b[pivot],b[i]
        local d=a[i][i]
        for k=i,6 do a[i][k]=a[i][k]/d end
        b[i]=b[i]/d
        for j=1,6 do if j~=i then
            local f=a[j][i]
            for k=i,6 do a[j][k]=a[j][k]-f*a[i][k] end
            b[j]=b[j]-f*b[i]
        end end
    end
    return b
end
local function jacobian(tip)
    local p=sim.getObjectPosition(tip,sim.handle_world)
    local cols={}
    for i,j in ipairs(cfg.joints) do
        local m=sim.getObjectMatrix(j,sim.handle_world)
        local axis={m[3],m[7],m[11]}
        local v=cross(axis,{p[1]-m[4],p[2]-m[8],p[3]-m[12]})
        cols[i]={v[1],v[2],v[3],axis[1],axis[2],axis[3]}
    end
    return cols
end
function sysCall_init()
    cfg=sim.unpackTable(sim.readCustomDataBlock(sim.getObject('..'),'adapter-config'))
    command={kind='joint',q={},stiffness={3000,3000,2000,2000,1000,800,500},damping={.8,.8,.8,.8,.8,.8,.8}}
    force={0,0,0}; torques={0,0,0,0,0,0,0}; groups={}
    for i,j in ipairs(cfg.joints) do command.q[i]=sim.getJointPosition(j) end
    for _,name in ipairs({'tcp','flange'}) do
        local env=simIK.createEnvironment()
        local group=simIK.createGroup(env)
        simIK.setGroupCalculation(env,group,simIK.method_damped_least_squares,.02,100)
        local element,map=simIK.addElementFromScene(env,group,-1,cfg[name],cfg[name..'Target'],simIK.constraint_pose)
        simIK.setElementPrecision(env,group,element,{0.0002,0.002})
        local joints={}; for i,j in ipairs(cfg.joints) do joints[i]=map[j] end
        groups[name]={env=env,group=group,tip=map[cfg[name]],target=map[cfg[name..'Target']],joints=joints}
    end
end
function setCommand(value) command=value end
function sysCall_actuation() end
function setForce(value) force=value end
function inverse(pose,seed,which)
    local g=groups[which or 'tcp']
    for i,j in ipairs(g.joints) do simIK.setJointPosition(g.env,j,seed[i]) end
    simIK.setObjectPose(g.env,g.target,pose,-1)
    local result=simIK.handleGroup(g.env,g.group)
    local q={}; for i,j in ipairs(g.joints) do q[i]=simIK.getJointPosition(g.env,j) end
    local actual=simIK.getObjectPose(g.env,g.tip,-1)
    local e=quatError(actual,pose)
    local err=math.sqrt((actual[1]-pose[1])^2+(actual[2]-pose[2])^2+(actual[3]-pose[3])^2)
    local angle=math.sqrt(e[1]^2+e[2]^2+e[3]^2)
    return {q=q,positionError=err,angleError=angle,result=result,pose=actual}
end
function forward(q,which)
    local g=groups[which or 'tcp']
    for i,j in ipairs(g.joints) do simIK.setJointPosition(g.env,j,q[i]) end
    return simIK.getObjectPose(g.env,g.tip,-1)
end
function state()
    local q,dq,measured={},{},{}
    for i,j in ipairs(cfg.joints) do
        q[i]=sim.getJointPosition(j); dq[i]=sim.getJointVelocity(j); measured[i]=sim.getJointForce(j)
    end
    local valid,f,t=sim.readForceSensor(cfg.flange)
    if valid==0 then f={0,0,0}; t={0,0,0} end
    local m=sim.getObjectMatrix(cfg.flange,sim.handle_world)
    f=rotate(m,f); t=rotate(m,t)
    local tcp=sim.getObjectPose(cfg.tcp,sim.handle_world)
    local flange=sim.getObjectPose(cfg.flange,sim.handle_world)
    local shift=cross({tcp[1]-flange[1],tcp[2]-flange[2],tcp[3]-flange[3]},f)
    for i=1,3 do t[i]=t[i]-shift[i] end
    -- Remove the known tool's static weight; inertial residuals remain an approximation.
    f[3]=f[3]+cfg.toolMass*9.81
    local jac=jacobian(cfg.tcp); local external={}
    for i=1,7 do external[i]=0; for k=1,3 do external[i]=external[i]+jac[i][k]*f[k]+jac[i][k+3]*t[k] end end
    return {q=q,dq=dq,tcp=tcp,flange=flange,force=f,torque=t,measuredTorque=measured,externalTorque=external,time=sim.getSimulationTime(),target=sim.getObjectPose(cfg.tcpTarget,sim.handle_world)}
end
function sysCall_dyn(data)
    if not data.afterStep and cfg then
        sim.addForce(cfg.tool, {0,0,0}, force)
    end
end
function sysCall_joint(data)
    if not cfg then return end
    local index
    for i,j in ipairs(cfg.joints) do if data.handle==j then index=i; break end end
    if not index then return end
    local q,dq={},{}
    for i,j in ipairs(cfg.joints) do q[i]=sim.getJointPosition(j); dq[i]=sim.getJointVelocity(j) end
    local tau
    if command.kind=='cartesian' then
        if command.rotationWhich then
            local m=sim.getObjectMatrix(cfg[command.rotationWhich],sim.handle_world)
            command.rotation={m[1],m[2],m[3],m[5],m[6],m[7],m[9],m[10],m[11]}
        end
        local tip=cfg[command.which or 'tcp']
        local pose=sim.getObjectPose(tip,sim.handle_world)
        local rot=quatError(pose,command.pose)
        local err={command.pose[1]-pose[1],command.pose[2]-pose[2],command.pose[3]-pose[3],rot[1],rot[2],rot[3]}
        local jac=jacobian(tip)
        local wrench,velocity={},{}
        for k=1,6 do velocity[k]=0; for i=1,7 do velocity[k]=velocity[k]+jac[i][k]*dq[i] end end
        if command.rotation then
            local r=command.rotation
            local function localVector(v,o)
                return {r[1]*v[o+1]+r[4]*v[o+2]+r[7]*v[o+3],r[2]*v[o+1]+r[5]*v[o+2]+r[8]*v[o+3],r[3]*v[o+1]+r[6]*v[o+2]+r[9]*v[o+3]}
            end
            for _,v in ipairs({err,velocity}) do
                local a,b=localVector(v,0),localVector(v,3)
                for k=1,3 do v[k]=a[k];v[k+3]=b[k] end
            end
        end
        for k=1,6 do
            local stiffness=command.stiffness[k]
            -- Damping ratio converted using nominal task inertia, not KUKA's private model.
            local inertia=k<=3 and 3 or .15
            wrench[k]=stiffness*err[k]-2*command.damping[k]*math.sqrt(stiffness*inertia)*velocity[k]
            if command.maxForce then wrench[k]=math.max(-command.maxForce[k],math.min(command.maxForce[k],wrench[k])) end
        end
        if command.rotation then
            local r=command.rotation
            for _,o in ipairs({0,3}) do
                local x,y,z=wrench[o+1],wrench[o+2],wrench[o+3]
                wrench[o+1]=r[1]*x+r[2]*y+r[3]*z
                wrench[o+2]=r[4]*x+r[5]*y+r[6]*z
                wrench[o+3]=r[7]*x+r[8]*y+r[9]*z
            end
        end
        tau=0; for k=1,6 do tau=tau+jac[index][k]*wrench[k] end
        local posture={}; local kn=command.nullStiffness or 0; local dn=command.nullDamping or .3
        for i=1,7 do posture[i]=kn*(command.q[i]-q[i])-2*dn*math.sqrt(math.max(kn,.1))*dq[i] end
        local a,b={},{}
        for r=1,6 do
            a[r]={}; b[r]=0
            for i=1,7 do b[r]=b[r]+jac[i][r]*posture[i] end
            for c=1,6 do
                a[r][c]=r==c and .0001 or 0
                for i=1,7 do a[r][c]=a[r][c]+jac[i][r]*jac[i][c] end
            end
        end
        local solution=solveLinear(a,b); local projection=0
        for k=1,6 do projection=projection+jac[index][k]*solution[k] end
        tau=tau+posture[index]-projection
    else
        local k=command.stiffness[index]; local d=command.damping[index]
        -- A small viscous term retains damping in zero-stiffness hand-guiding mode.
        local damping=k==0 and 2*d or 2*d*math.sqrt(k*.2)
        tau=k*(command.q[index]-q[index])-damping*dq[index]
    end
    local low,high=cfg.limits[index][1],cfg.limits[index][2]
    if q[index]<low+.02 then tau=tau+300*(low+.02-q[index]) end
    if q[index]>high-.02 then tau=tau-300*(q[index]-high+.02) end
    tau=math.max(-cfg.maxTorque[index],math.min(cfg.maxTorque[index],tau)); torques[index]=tau
    return {vel=tau>=0 and 10000 or -10000,force=math.abs(tau)}
end
function sysCall_cleanup()
    for _,g in pairs(groups or {}) do simIK.eraseEnvironment(g.env) end
end
