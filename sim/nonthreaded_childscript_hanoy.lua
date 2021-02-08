function print(s)
    sim.addLog(sim.verbosity_scriptinfos, s)
end

-- moves object named disk above object named tower
function move(disk, tower)
    print("Movind disk "..disk.." to tower "..tower..".")

    disk = sim.getObjectHandle(disk)
    tower = sim.getObjectHandle(tower)

    sim.setObjectPosition(disk, tower, {0,0,.5})
end

-- lists children of an object
function getChildren(parentHandle)
    local children = {}
    local index = 0
    repeat
        local tmp = sim.getObjectChild(parentHandle, index)
        if -1 < tmp then children[index+1], index = tmp, index+1 end
    until tmp < 0
    return children
end

-- lists children by name of an object by name
function getNamedChildren(parentName)
    local parent = sim.getObjectHandle(parentName)
    local r = {}
    for k,v in pairs(getChildren(parent)) do
        local name = sim.getObjectName(v)
        r[name] = v
    end
    return r
end

-- finds the towers and add their coordinates to param server
function setupTowers(towers)
    local allNames = ""
    local nameSep = ""
    for k,v in pairs(towers) do
        allNames = allNames..nameSep..k
        nameSep = ";"

        local name = "/game/tower"..k
        local pos = sim.getObjectPosition(v, -1)
        simROS.setParamDouble(name.."/x", pos[1])
        simROS.setParamDouble(name.."/y", pos[2])
        simROS.setParamDouble(name.."/z", pos[3])
    end
    simROS.setParamString("/game/towers", allNames)
end

-- finds the disks and advertise their coordinates to topic
function setupDisks(disks)
    publishers = {}
    local allNames = ""
    local nameSep = ""
    for k,v in pairs(disks) do
        allNames = allNames..nameSep..k
        nameSep = ";"

        local name = "/game/sim/disk"..k
        publishers[k] = { disk=v, topic=name, pub=simROS.advertise(name, "geometry_msgs/Point") }
    end
    simROS.setParamString("/game/disks", allNames)
end

function sysCall_init()
    this = sim.getObjectAssociatedWithScript(sim.handle_self)

    -- towers are children of "Tower" object
    setupTowers(getNamedChildren("Towers"))
    -- disks are children of "Disks" object
    setupDisks(getNamedChildren("Disks"))

    -- setups signal callback (listen for string signal "move")
    sim.setStringSignal("move", "")
    print([[Use 'sim.setStringSignal("move", "3-B")' to move disk 3 to tower B.]])

    -- starts remote API server
    remoteSocketPort = 20202
    if -1 < simRemoteApi.start(remoteSocketPort) then
        print("Started remote API on 127.0.0.1 : "..remoteSocketPort)
    else
        print("Could not start remote API...")
    end
end

function sysCall_actuation()
    -- publishes disks coordinates to their respective topic
    for k,v in pairs(publishers) do
        sim.setObjectOrientation(v.disk, this, {0,0,0})
        local pos = sim.getObjectPosition(v.disk, -1)
        simROS.publish(v.pub, {
            x=pos[1],
            y=pos[2],
            z=pos[3],
        })
    end

    -- checks for string signal "move" of format "{disk}-{tower}"
    local input = sim.getStringSignal("move")
    if input ~= "" then
        local dash = string.find(input, '-')
        if dash and 2 < #input then
            move(string.sub(input, 1,dash-1), string.sub(input, dash+1))
        end
        sim.setStringSignal("move", "")
    end
end

function sysCall_cleanup()
    -- (not necessary) stops publisher
    for k,v in pairs(publishers) do
        simROS.shutdownPublisher(v.pub)
    end
    -- resets params giving towers and disks names
    simROS.setParamString("/game/towers", "")
    simROS.setParamString("/game/disks", "")
    -- closes API server
    simRemoteApi.stop(remoteSocketPort)
end
