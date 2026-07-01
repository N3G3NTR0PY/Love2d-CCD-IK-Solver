----- COLORS
local color = {
    black = {0, 0, 0, 1},
    white = {1, 1, 1, 1},
    gray  = {0.5, 0.5, 0.5, 1},
    lightPurple = {0.2117, 0.2117, 0.2980, 1},
    darkPurple = {0.1021, 0.1021, 0.1505, 1},
    armShadow = {0.1450, 0.1450, 0.2078, 1},
    foregroundShadow = {0, 0, 0, 0.3}
}

----- WINDOW DEFAULTS
local window = {
	fullscreen = false,
    width = 800,
    height = 600,
    title = 'Inverse Kinematics Solver',
    color = color.black
}
local centerX, centerY = window.width / 2, window.height / 2

----- SCENE CONFIG
local scene = {
    backgroundColor = color.lightPurple,
    foregroundColor = color.darkPurple,
    separatorColor = color.white,
    separatorThickness = 20,
    separatorY = centerY + centerY / 2,

    separatorShadowYoffset = 9,
    armShadowXoffset = 11,
    armShadowYoffset = 8,
}

----- CONFIG TABLE (Example configuration)
local arm = {
    root = {centerX / 1.25, scene.separatorY - scene.separatorThickness / 2},

    linkLengths = {
        90,
        150,
        105,
        90,
        -- ADD MORE HERE
    },
    segmentModificationSpeed = 50,

    jointAngles = {
        -45,
        90,
        10,
        35,
        -- ADD MORE HERE
    },

    jointLimits = {
        {75, 75},
        {0, 135},
        {135, 105},
        {135, 135},
        -- ADD MORE HERE
    },

    angleModificationSpeed = math.pi / 4,

    jointRadius = 20,
    jointLineThickness = 5,
    jointRadiusDecreaseRate = 1.2,

    activeColor = color.white,
    inactiveColor = color.gray,

    lineThickness = 25,
    lineThicknessDecreaseRate = 1.2,

    baseConvergenceRate = 0.3
}


----- WINDOW CREATION
local function initWindow(width, height, title, color)
	love.window.setTitle(title)
	love.graphics.setBackgroundColor(color)
	love.window.setMode(width, height, {fullscreen = window.fullscreen, resizable = false})
end


----- CONVERT DEGS TO RADS
local function convertJointAngles()
    arm.jointAngles[1] = math.rad(arm.jointAngles[1]) + -math.pi / 2
    arm.jointLimits[1][1] = -math.pi / 2 - math.rad(arm.jointLimits[1][1])
    arm.jointLimits[1][2] = -math.pi / 2 + math.rad(arm.jointLimits[1][2])
    for angle = 2, #arm.jointAngles do
        arm.jointAngles[angle] = math.rad(arm.jointAngles[angle])
        arm.jointLimits[angle][1] = math.rad(arm.jointLimits[angle][1])
        arm.jointLimits[angle][2] = math.rad(arm.jointLimits[angle][2])
    end
end


----- WRAP ANGLES
local function wrapJointAngles()
    for angle = 1, #arm.jointAngles do
        if math.abs(arm.jointAngles[angle]) >= math.pi * 2 then
            arm.jointAngles[angle] = math.fmod(arm.jointAngles[angle], math.pi * 2)
        end
    end
end


------ CALCULATE RADIUS AND THICKNESS DECREASE
local function updateRadii()
    arm.radii = {}
    table.insert(arm.radii, arm.jointRadius)
    for link = 2, #arm.linkLengths do
        table.insert(arm.radii, arm.radii[link - 1] / arm.jointRadiusDecreaseRate)
    end
end

local function updateLinkThicknesses()
    arm.linkThicknesses = {}
    table.insert(arm.linkThicknesses, arm.lineThickness)
    for link = 2, #arm.linkLengths do
        table.insert(arm.linkThicknesses, arm.linkThicknesses[link - 1] / arm.lineThicknessDecreaseRate)
    end
end

----- FORWARD KINEMATICS
arm.jointPos = {}
local endEffector = {arm.root[1], arm.root[2]}

local function forwardKinematics()
    arm.jointPos = {}
    local absoluteAngle = arm.jointAngles[1]
    table.insert(
        arm.jointPos, {arm.root[1] + arm.linkLengths[1] * math.cos(absoluteAngle),
        arm.root[2] + arm.linkLengths[1] * math.sin(absoluteAngle)}
    )
    for segment = 2, #arm.linkLengths do
        absoluteAngle = 0
        for joint = 1, segment do
            absoluteAngle = absoluteAngle + arm.jointAngles[joint]
        end
        table.insert(arm.jointPos,
            {arm.jointPos[segment - 1][1] + arm.linkLengths[segment] * math.cos(absoluteAngle),
            arm.jointPos[segment - 1][2] + arm.linkLengths[segment] * math.sin(absoluteAngle)}
        )
    end
    endEffector = {arm.jointPos[#arm.jointPos][1], arm.jointPos[#arm.jointPos][2]}
end

----- CALCULATING ARM TARGET
local mousePos = {endEffector[1], endEffector[2]}
local target = {mousePos[1], mousePos[2]}
local reachable = true

local distance, workspaceRadius

local function setTarget(x, y)
    local dx, dy = x - arm.root[1], y - arm.root[2]
    local angle = math.atan2(dy, dx)
    distance = math.sqrt(dx ^ 2 + dy ^ 2)
    workspaceRadius = 0

    for segment = 1, #arm.linkLengths do
        workspaceRadius = workspaceRadius + arm.linkLengths[segment]
    end

    if distance < workspaceRadius then
        target = {x, y}
        reachable = true
    else
        target[1] = arm.root[1] + workspaceRadius * math.cos(angle)
        target[2] = arm.root[2] + workspaceRadius * math.sin(angle)
        reachable = false
    end
end

----- CCD STEP
local function ccdCorrectionStep(number, currentAngle, targetAngle, baseConvergenceRate)
    local delta = targetAngle - currentAngle
	local maxConvergenceRate = baseConvergenceRate / 30
    local convergenceRate = math.min(baseConvergenceRate / arm.linkLengths[number], maxConvergenceRate)
    while delta > math.pi do
        delta = delta - math.pi * 2
    end 
    while delta < -math.pi do
        delta = delta + math.pi * 2
    end
    if delta > 0 then
        arm.jointAngles[number] = arm.jointAngles[number] + convergenceRate
    elseif delta < 0 then
        arm.jointAngles[number] = arm.jointAngles[number] - convergenceRate
    end
end


----- ANGLE CLAMPER
local function clampAngles()
    if arm.jointAngles[1] < arm.jointLimits[1][1] then
        arm.jointAngles[1] = arm.jointLimits[1][1]
    elseif arm.jointAngles[1] > arm.jointLimits[1][2] then
        arm.jointAngles[1] = arm.jointLimits[1][2]
    end
    for i = 2, #arm.jointAngles do
        if arm.jointAngles[i] < -arm.jointLimits[i][1] then
            arm.jointAngles[i] = -arm.jointLimits[i][1] 
        elseif arm.jointAngles[i] > arm.jointLimits[i][2] then
            arm.jointAngles[i] = arm.jointLimits[i][2]
        end
    end
end


----- CCD INVERSE KINEMATICS SOLVER (Also known as pain and suffering, but at least no jacobian here)
local iterationAmount = 12
local autoMode = false
local function inverseKinematics()
    local dx, dy, targetAngle, currentAbsoluteAngle
    for i = 1, iterationAmount do
        for joint = 0, #arm.jointAngles - 2 do
            dx = target[1] - arm.jointPos[#arm.jointAngles - joint - 1][1]
            dy = target[2] - arm.jointPos[#arm.jointAngles - joint - 1][2]
            targetAngle = math.atan2(dy, dx)
            dx = endEffector[1] - arm.jointPos[#arm.jointPos - joint - 1][1]
            dy = endEffector[2] - arm.jointPos[#arm.jointPos - joint - 1][2]
            currentAbsoluteAngle = math.atan2(dy, dx)
            ccdCorrectionStep(#arm.jointAngles - joint, currentAbsoluteAngle, targetAngle, arm.baseConvergenceRate)
        end
            dx = target[1] - arm.root[1]
            dy = target[2] - arm.root[2]
            targetAngle = math.atan2(dy, dx)
            dx = endEffector[1] - arm.root[1]
            dy = endEffector[2] - arm.root[2]
            currentAbsoluteAngle = math.atan2(dy, dx)
            ccdCorrectionStep(1, currentAbsoluteAngle, targetAngle, arm.baseConvergenceRate)
        wrapJointAngles()
        clampAngles()
        forwardKinematics()
    end
end

----- INTERACTIVITY
local keyStates = {}
local function pressed(key)
    local down = love.keyboard.isDown(key)

    if down and not keyStates[key] then
        keyStates[key] = down
        return true
    else
        keyStates[key] = love.keyboard.isDown(key)
        return false
    end
end

local selectedSegment = 1
local function checkSegmentSelection()
    for key = 1, #arm.linkLengths do
        if pressed(tostring(key)) then
            selectedSegment = key
        end
    end
end

local angleModificationDirection = 'clockwise'
local function checkAngleModification()
    if love.keyboard.isDown('right') then
        angleModificationDirection = 'clockwise'
        return true
    elseif love.keyboard.isDown('left') then
        angleModificationDirection = 'counterclockwise'
        return true
    end
end

local lengthModificationType = 'extend'
local function checkLengthModification()
    if love.keyboard.isDown('up') then
        lengthModificationType = 'extend'
        return true
    elseif love.keyboard.isDown('down') then
        lengthModificationType = 'retract'
        return true
    end
end

local function modifyAngle(number, direction, speed, deltaTime)
    if direction == 'counterclockwise' then
        speed = -speed
    end
    arm.jointAngles[number] = arm.jointAngles[number] + speed * deltaTime
end

local function modifyLength(number, action, speed, deltaTime)
    if action == 'retract' then
        speed = -speed
    end
    arm.linkLengths[number] = arm.linkLengths[number] + speed * deltaTime
    if arm.linkLengths[number] <= 0 then
        arm.linkLengths[number] = 0.00001
    end
end

----- DRAWN A JOINT
local function drawJoint(x, y, radius, color, type)
    love.graphics.setColor(color)
    if type == 'filled' then
        love.graphics.circle('fill', x, y, radius)
    else
        love.graphics.circle('line', x, y, radius)
    end
end

----- GET MOUSE POS
function love.mousemoved(x, y)
    mousePos = {x, y}
    setTarget(x, y)
end





----- INIT
function love.load()
    initWindow(window.width, window.height, window.title, window.color)
    updateRadii()
    updateLinkThicknesses()
    convertJointAngles()
    forwardKinematics()
    setTarget(endEffector[1], endEffector[2])
end

----- DRAW
function love.draw()
    -- BACKGROUND
    love.graphics.setColor(scene.backgroundColor)
    love.graphics.rectangle("fill", 0, 0, window.width, scene.separatorY)
    -- SHADOW OF THE ARM
    love.graphics.setColor(color.armShadow)
    love.graphics.setLineWidth(arm.lineThickness)
    local dx = arm.jointPos[1][1] - arm.root[1]
    local dy = arm.jointPos[1][2] - arm.root[2]
    local currentAngle = math.pi / 2 - math.atan2(dx, dy)
    local startOffsetX = (arm.radii[1] - arm.jointLineThickness) * math.cos(currentAngle)
    local startOffsetY = (arm.radii[1] - arm.jointLineThickness) * math.sin(currentAngle)
    local endOffsetX = (arm.radii[2] - arm.jointLineThickness / 2) * math.cos(currentAngle)
    local endOffsetY = (arm.radii[2] - arm.jointLineThickness / 2) * math.sin(currentAngle)
    love.graphics.line(
        arm.root[1] + scene.armShadowXoffset + startOffsetX, arm.root[2] + scene.armShadowYoffset + startOffsetY,
        arm.jointPos[1][1] + scene.armShadowXoffset - endOffsetX, arm.jointPos[1][2]+ scene.armShadowYoffset - endOffsetY
    )
    if selectedSegment ~= 1 then
        love.graphics.setLineWidth(arm.lineThickness / 3)
        love.graphics.setColor(scene.backgroundColor)
        love.graphics.line(
            arm.root[1] + scene.armShadowXoffset + startOffsetX, arm.root[2] + scene.armShadowYoffset + startOffsetY,
            arm.jointPos[1][1] + scene.armShadowXoffset - endOffsetX, arm.jointPos[1][2]+ scene.armShadowYoffset - endOffsetY
        )
        love.graphics.setLineWidth(arm.lineThickness)
    end
    for segment = 2, #arm.jointPos do
        dx = arm.jointPos[segment][1] - arm.jointPos[segment - 1][1]
        dy = arm.jointPos[segment][2] - arm.jointPos[segment - 1][2]
        currentAngle = math.pi / 2 - math.atan2(dx, dy)
        startOffsetX = (arm.radii[segment - 1] - arm.jointLineThickness) * math.cos(currentAngle)
        startOffsetY = (arm.radii[segment - 1] - arm.jointLineThickness) * math.sin(currentAngle)
        endOffsetX = (arm.radii[segment] - arm.jointLineThickness) * math.cos(currentAngle)
        endOffsetY = (arm.radii[segment] - arm.jointLineThickness) * math.sin(currentAngle)
        love.graphics.setLineWidth(arm.linkThicknesses[segment])
        love.graphics.setColor(color.armShadow)
        love.graphics.line(
            arm.jointPos[segment - 1][1] + scene.armShadowXoffset + startOffsetX, arm.jointPos[segment - 1][2] + scene.armShadowYoffset + startOffsetY,
            arm.jointPos[segment][1] + scene.armShadowXoffset - endOffsetX, arm.jointPos[segment][2] + scene.armShadowYoffset - endOffsetY
        )
        if selectedSegment ~= segment then
            love.graphics.setLineWidth(arm.linkThicknesses[segment] / 3)
            love.graphics.setColor(scene.backgroundColor)
            love.graphics.line(
                arm.jointPos[segment - 1][1] + scene.armShadowXoffset + startOffsetX, arm.jointPos[segment - 1][2] + scene.armShadowYoffset + startOffsetY,
                arm.jointPos[segment][1]  + scene.armShadowXoffset - endOffsetX, arm.jointPos[segment][2] + scene.armShadowYoffset - endOffsetY
            )
            love.graphics.setLineWidth(arm.lineThickness)
        end
        love.graphics.setLineWidth(arm.jointLineThickness)
        drawJoint(arm.jointPos[segment - 1][1] + scene.armShadowXoffset, arm.jointPos[segment - 1][2] + scene.armShadowYoffset, arm.radii[segment], color.armShadow, 'hollow')
    end
    love.graphics.setLineWidth(arm.jointLineThickness)
    drawJoint(endEffector[1] + scene.armShadowXoffset, endEffector[2] + scene.armShadowYoffset, arm.radii[#arm.radii] / arm.jointRadiusDecreaseRate, color.armShadow, 'hollow')
    drawJoint(arm.root[1] + scene.armShadowXoffset, arm.root[2] + scene.armShadowYoffset, arm.radii[1], color.armShadow, 'filled')
    -- THE ARM ITSELF
    love.graphics.setLineWidth(arm.lineThickness)
    love.graphics.setColor(arm.activeColor)
    dx = arm.jointPos[1][1] - arm.root[1]
    dy = arm.jointPos[1][2] - arm.root[2]
    currentAngle = math.pi / 2 - math.atan2(dx, dy)
    startOffsetX = (arm.radii[1] - arm.jointLineThickness) * math.cos(currentAngle)
    startOffsetY = (arm.radii[1] - arm.jointLineThickness) * math.sin(currentAngle)
    endOffsetX = (arm.radii[2] - arm.jointLineThickness / 2) * math.cos(currentAngle)
    endOffsetY = (arm.radii[2] - arm.jointLineThickness / 2) * math.sin(currentAngle)
    love.graphics.line(
        arm.root[1] + startOffsetX, arm.root[2] + startOffsetY,
        arm.jointPos[1][1] - endOffsetX, arm.jointPos[1][2] - endOffsetY
    )
    if selectedSegment ~= 1 then
        love.graphics.setLineWidth(arm.lineThickness / 3)
        love.graphics.setColor(scene.backgroundColor)
        love.graphics.line(
            arm.root[1], arm.root[2],
            arm.jointPos[1][1], arm.jointPos[1][2]
        )
        love.graphics.setLineWidth(arm.lineThickness)
    end
    for segment = 2, #arm.jointPos do
        dx = arm.jointPos[segment][1] - arm.jointPos[segment - 1][1]
        dy = arm.jointPos[segment][2] - arm.jointPos[segment - 1][2]
        currentAngle = math.pi / 2 - math.atan2(dx, dy)
        startOffsetX = (arm.radii[segment - 1] - arm.jointLineThickness) * math.cos(currentAngle)
        startOffsetY = (arm.radii[segment - 1] - arm.jointLineThickness) * math.sin(currentAngle)
        endOffsetX = (arm.radii[segment] - arm.jointLineThickness) * math.cos(currentAngle)
        endOffsetY = (arm.radii[segment] - arm.jointLineThickness) * math.sin(currentAngle)
        love.graphics.setLineWidth(arm.linkThicknesses[segment])
        love.graphics.setColor(arm.activeColor)
        love.graphics.line(
            arm.jointPos[segment - 1][1] + startOffsetX, arm.jointPos[segment - 1][2] + startOffsetY,
            arm.jointPos[segment][1] - endOffsetX, arm.jointPos[segment][2] - endOffsetY
        )
        if selectedSegment ~= segment then
            love.graphics.setLineWidth(arm.linkThicknesses[segment] / 3)
            love.graphics.setColor(scene.backgroundColor)
            love.graphics.line(
                arm.jointPos[segment - 1][1] + startOffsetX, arm.jointPos[segment - 1][2] + startOffsetY,
                arm.jointPos[segment][1] - endOffsetX, arm.jointPos[segment][2] - endOffsetY
            )
            love.graphics.setLineWidth(arm.lineThickness)
        end
        love.graphics.setLineWidth(arm.jointLineThickness)
        drawJoint(arm.jointPos[segment - 1][1], arm.jointPos[segment - 1][2], arm.radii[segment], arm.activeColor, 'hollow')
    end
    love.graphics.setLineWidth(arm.jointLineThickness)
    drawJoint(endEffector[1], endEffector[2], arm.radii[#arm.radii] / arm.jointRadiusDecreaseRate, arm.activeColor, 'hollow')
    -- TARGET
    if not reachable then
        drawJoint(mousePos[1], mousePos[2], arm.radii[#arm.radii] / arm.jointRadiusDecreaseRate, arm.inactiveColor, 'hollow')
    end
    drawJoint(target[1], target[2], arm.radii[#arm.radii] / arm.jointRadiusDecreaseRate, arm.activeColor, 'hollow')
    -- FOREGROUND
    love.graphics.setLineWidth(arm.jointLineThickness)
    drawJoint(arm.root[1], arm.root[2], arm.radii[1], arm.activeColor, 'filled')
    love.graphics.setColor(scene.foregroundColor)
    love.graphics.rectangle("fill", 0, scene.separatorY, window.width, window.height - scene.separatorY)
    love.graphics.setColor(scene.separatorColor)
    love.graphics.setLineWidth(scene.separatorThickness)
    love.graphics.line(
        0, scene.separatorY,
        window.width, scene.separatorY
    )
    love.graphics.setColor(color.foregroundShadow)
    love.graphics.rectangle('fill', 0, scene.separatorY + scene.separatorThickness / 2, window.width, scene.separatorShadowYoffset)
end

----- UPDATE
function love.update(dt)
    checkSegmentSelection()
    if checkAngleModification() then
        modifyAngle(selectedSegment, angleModificationDirection, arm.angleModificationSpeed, dt)
        wrapJointAngles()
        setTarget(mousePos[1], mousePos[2])
        forwardKinematics()
    end
    if checkLengthModification() then
        modifyLength(selectedSegment, lengthModificationType, arm.segmentModificationSpeed, dt)
        wrapJointAngles()
        setTarget(mousePos[1], mousePos[2])
        forwardKinematics()
    end
    if pressed("space") then
        autoMode = not autoMode
    end
    if autoMode then
        inverseKinematics()
    end
end
