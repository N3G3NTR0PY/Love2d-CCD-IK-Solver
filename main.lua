----- COLORS
local color = {
    black = {0, 0, 0, 1},
    white = {1, 1, 1, 1},
    gray  = {0.5, 0.5, 0.5, 1}
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

----- CONFIG TABLE
local arm = {
    root = {centerX, centerY},
    
    linkLengths = {
        80,
        120,
        40,
        -- ADD MORE HERE
    },
    segmentModificationSpeed = 50,
    
    jointAngles = {
        -135,
        90,
        90,
        -- ADD MORE HERE
    },
    angleModificationSpeed = math.pi / 4,
    
    jointRadius = 10,
    jointRadiusDecreaseRate = 1.4,

    activeColor = color.white,
    inactiveColor = color.gray,

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
    for angle = 1, #arm.jointAngles do
        arm.jointAngles[angle] = math.rad(arm.jointAngles[angle])
    end
end


----- WARP
local function wrapJointAngles()
    for angle = 1, #arm.jointAngles do
        if math.abs(arm.jointAngles[angle]) >= math.pi * 2 then
            arm.jointAngles[angle] = math.fmod(arm.jointAngles[angle], math.pi * 2)
        end
    end
end


----- FORWARD KINEMATICS
arm.jointPos = {}
local endEffector = {arm.root[1], arm.root[2]}

local function forwardKinematics()
    arm.jointPos = {}
    local relativeAngle = arm.jointAngles[1]
    table.insert(
        arm.jointPos, {arm.root[1] + arm.linkLengths[1] * math.cos(relativeAngle),
        arm.root[2] + arm.linkLengths[1] * math.sin(relativeAngle),
        arm.jointRadius}
    )
    for segment = 2, #arm.linkLengths do
        relativeAngle = 0
        for joint = 1, segment do
            relativeAngle = relativeAngle + arm.jointAngles[joint]
        end
        table.insert(arm.jointPos,
            {arm.jointPos[segment - 1][1] + arm.linkLengths[segment] * math.cos(relativeAngle), 
            arm.jointPos[segment - 1][2] + arm.linkLengths[segment] * math.sin(relativeAngle),
            arm.jointPos[segment - 1][3] / arm.jointRadiusDecreaseRate}
        )
    end
    endEffector = {arm.jointPos[#arm.jointPos][1], arm.jointPos[#arm.jointPos][2]}
end

----- CALCULATING arm TARGET
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
    local convergenceRate = math.min(baseConvergenceRate / arm.linkLengths[number], baseConvergenceRate / 30)
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


----- CCD INVERSE KINEMATICS SOLVER (Also known as pain and suffering, but alteast no jacobian here)
local autoMode = false
local function inverseKinematics()
    local dx, dy, targetAngle, currentAbsoluteAngle
    for i = 1, 10 do
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

local function drawJoint(x, y, radius, color, type)
    if type == 'filled' then
        love.graphics.setColor(window.color)
        love.graphics.circle('fill', x, y, radius)
    end
    love.graphics.setColor(color)
    love.graphics.circle('line', x, y, radius)
end

----- GET MOUSE POS
function love.mousemoved(x, y)
    mousePos = {x, y}
    setTarget(x, y)
end





----- INIT
function love.load()
    initWindow(window.width, window.height, window.title, window.color)
    convertJointAngles()
    forwardKinematics()
    setTarget(endEffector[1], endEffector[2])
end

----- DRAW
function love.draw()
    if selectedSegment == 1 then
        love.graphics.setColor(arm.activeColor)
    else
        love.graphics.setColor(arm.inactiveColor)
    end
    love.graphics.line(arm.root[1], arm.root[2], arm.jointPos[1][1], arm.jointPos[1][2])
    drawJoint(arm.root[1], arm.root[2], arm.jointPos[1][3], arm.inactiveColor, 'filled')
    for segment = 2, #arm.jointPos do
        if selectedSegment == segment then
            love.graphics.setColor(arm.activeColor)
        else
            love.graphics.setColor(arm.inactiveColor)
        end
        love.graphics.line(
            arm.jointPos[segment - 1][1], arm.jointPos[segment - 1][2],
            arm.jointPos[segment][1], arm.jointPos[segment][2]
        )
        drawJoint(arm.jointPos[segment - 1][1], arm.jointPos[segment - 1][2], arm.jointPos[segment][3], arm.inactiveColor, 'filled')
    end
    drawJoint(endEffector[1], endEffector[2], arm.jointPos[#arm.jointPos][3] / arm.jointRadiusDecreaseRate, arm.inactiveColor, 'filled')
    if not reachable then
        drawJoint(mousePos[1], mousePos[2], arm.jointPos[#arm.jointPos][3] / arm.jointRadiusDecreaseRate, arm.inactiveColor, 'hollow')
    end
    drawJoint(target[1], target[2], arm.jointPos[#arm.jointPos][3] / arm.jointRadiusDecreaseRate, arm.activeColor, 'hollow')
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