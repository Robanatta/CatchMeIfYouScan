local workspace = '/Users/robanatta/Robotics/Assignment_2/robotics-lab-usi-robomaster'
local inputScene = workspace .. '/scenes/chaser_runner_test.ttt'
local outputScene = workspace .. '/scenes/big_path_planning_scene.ttt'

local function setAlias(handle, alias)
    pcall(function()
        sim.setObjectAlias(handle, alias)
    end)
end

local function setColor(handle, rgb)
    pcall(function()
        sim.setShapeColor(handle, '', sim.colorcomponent_ambient_diffuse, rgb)
    end)
end

local function setDetectableObstacle(handle)
    local props =
        sim.objectspecialproperty_collidable +
        sim.objectspecialproperty_measurable +
        sim.objectspecialproperty_detectable_all +
        sim.objectspecialproperty_renderable

    pcall(function()
        sim.setObjectSpecialProperty(handle, props)
    end)
end

local function setStaticPhysics(handle, respondable)
    pcall(function()
        sim.setObjectInt32Param(handle, sim.shapeintparam_static, 1)
        sim.setObjectInt32Param(handle, sim.shapeintparam_respondable, respondable and 1 or 0)
    end)
end

local function makeBox(alias, x, y, z, sx, sy, sz, color)
    local handle = sim.createPrimitiveShape(sim.primitiveshape_cuboid, {sx, sy, sz}, 2)
    setAlias(handle, alias)
    sim.setObjectPosition(handle, {x, y, z + sz / 2.0}, sim.handle_world)
    setColor(handle, color)
    setStaticPhysics(handle, true)
    setDetectableObstacle(handle)

    return handle
end

local function makeCylinder(alias, x, y, z, radius, height, color)
    local handle = sim.createPrimitiveShape(sim.primitiveshape_cylinder, {2.0 * radius, 2.0 * radius, height}, 2)
    setAlias(handle, alias)
    sim.setObjectPosition(handle, {x, y, z + height / 2.0}, sim.handle_world)
    setColor(handle, color)
    setStaticPhysics(handle, true)
    setDetectableObstacle(handle)

    return handle
end

local function makeMarker(alias, x, y, radius, color)
    local handle = sim.createPrimitiveShape(sim.primitiveshape_cylinder, {2.0 * radius, 2.0 * radius, 0.025}, 2)
    setAlias(handle, alias)
    sim.setObjectPosition(handle, {x, y, 0.013}, sim.handle_world)
    setColor(handle, color)
    setStaticPhysics(handle, false)

    return handle
end

sim.loadScene(inputScene)

-- Large map-sized floor. The planner currently assumes a 12m x 12m world.
makeBox('planning_floor_12m', 0.0, 0.0, -0.035, 12.0, 12.0, 0.04, {0.42, 0.46, 0.48})

-- Outer boundary walls: useful for visual orientation and ToF returns.
makeBox('wall_north', 0.0, 6.0, 0.0, 12.0, 0.18, 0.65, {0.20, 0.22, 0.24})
makeBox('wall_south', 0.0, -6.0, 0.0, 12.0, 0.18, 0.65, {0.20, 0.22, 0.24})
makeBox('wall_east', 6.0, 0.0, 0.0, 0.18, 12.0, 0.65, {0.20, 0.22, 0.24})
makeBox('wall_west', -6.0, 0.0, 0.0, 0.18, 12.0, 0.65, {0.20, 0.22, 0.24})

-- Obstacle layout: corridors, a central blocker, and several partial walls.
makeBox('obstacle_left_corridor_wall', -3.2, 1.8, 0.0, 0.35, 4.4, 0.70, {0.12, 0.35, 0.62})
makeBox('obstacle_right_corridor_wall', 3.0, -1.7, 0.0, 0.35, 4.7, 0.70, {0.12, 0.35, 0.62})
makeBox('obstacle_top_gate', 0.1, 3.35, 0.0, 4.4, 0.35, 0.70, {0.18, 0.42, 0.34})
makeBox('obstacle_bottom_gate', -0.1, -3.15, 0.0, 4.0, 0.35, 0.70, {0.18, 0.42, 0.34})
makeBox('obstacle_center_block', 0.0, 0.0, 0.0, 1.25, 1.05, 0.75, {0.55, 0.30, 0.18})
makeBox('obstacle_short_wall_a', -4.25, -2.3, 0.0, 2.0, 0.30, 0.65, {0.50, 0.26, 0.50})
makeBox('obstacle_short_wall_b', 4.15, 2.3, 0.0, 2.0, 0.30, 0.65, {0.50, 0.26, 0.50})
makeBox('obstacle_diagonal_hint_a', -1.9, -4.55, 0.0, 0.35, 1.9, 0.60, {0.38, 0.40, 0.17})
makeBox('obstacle_diagonal_hint_b', 1.9, 4.55, 0.0, 0.35, 1.9, 0.60, {0.38, 0.40, 0.17})

-- Round columns make the path less grid-like while still being easy for ToF to detect.
makeCylinder('obstacle_column_nw', -4.6, 4.3, 0.0, 0.35, 0.75, {0.58, 0.54, 0.22})
makeCylinder('obstacle_column_se', 4.65, -4.2, 0.0, 0.35, 0.75, {0.58, 0.54, 0.22})

-- Small floor markers to make the planned start/goal areas obvious in the GUI.
makeMarker('chaser_start_area_marker', -4.7, -4.8, 0.45, {0.05, 0.25, 0.85})
makeMarker('runner_goal_area_marker', 4.7, 4.7, 0.45, {0.95, 0.80, 0.10})

sim.saveScene(outputScene)
sim.addLog(sim.verbosity_scriptinfos, 'Saved big path-planning scene to: ' .. outputScene)
sim.quitSimulator()
