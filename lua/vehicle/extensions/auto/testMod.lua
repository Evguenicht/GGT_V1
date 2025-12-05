-- lines modified : 203, 




local nodes = require("vehicleeditor.nodes")
groundModelsLut = {} 
beamstate = require("beamstate")
local slipspeedGlobal = 0
local slipDerivativeGlobal = 0
local angleRMemory = 0
local angleRPrev = 0
local angleRMemory2 = 0
local angleRPrev2 = 0
local angleRPrevD = 0
local angleRPrev2D = 0
local tyreGripGlobal = 0
local slipDerivativeP = 0
local activeBoost = 0
local velocityPrev = 0

local M = {}
local acc = 0
local targetStep = 1 / 20


local tyreGripTable = {}


local tyreData = {}
local wheelCache = {}

local totalTimeMod60 = 0

local degubStepFinished = true


local function GetGroundModelData(id)
    local materials, materialsMap = particles.getMaterialsParticlesTable()
    local matData = materials[id] or {}
    local name = matData.name or "DOESNT EXIST"
    local name = groundModelsLut[id] or "DOESNT EXIST"
    return name
end


local function CalcSlip(angleR, velocity, angularVel, radius)
    local dirFactor = math.max(0.01, math.abs(math.cos(math.rad(angleR))))
    return (math.abs(angularVel) * radius * dirFactor - velocity)
end

function sinusModifie(deg, tire)
    if deg <= 0 or deg >= 180 then
        return 0
    elseif deg > 0 and deg < tire then
        return math.sin(math.rad(90 * (deg / tire)))
	elseif deg >= tire and deg < 1.5*tire then
        return math.sin(math.rad(90 * ((1.2*tire - deg) / (tire/0.2))))
    elseif deg >= 1.5*tire and deg <= (180-1.5*tire) then
        return 
	elseif deg > (180-1.5*tire) and deg <= (180-tire) then
        return math.sin(math.rad(90 * (((180-1.2*tire)-deg) / (-tire*0.2))))
    elseif deg > (180-tire) and deg <= 180 then
        return math.sin(math.rad(90 * ((180 - deg) / tire)))
    end
end



local function CalculateGrip(wheelID, treadCoef, radius, angularVel, tyreWidth, 
	groundModelName, velocity, percentageLoad, angleR, propulsionTorque,slipspeedGlobal,dt,totalLoad,flancHeight,softnessCoef,angleTriche,slipDerivativeGlobal, 
	angleR2, treadCoef2, Tyreminheight, Tyremaxheight, AngleDif, AngleDif2, AngleMax2, AngleMax, downForceRaw, tyreWratio,TyremaxW,betaDeg)
    local data = tyreData[wheelID]
	local validSurfaceTypes = {
        DIRT = true, SAND = true, MUD = true, GRAVEL = true
    }

	local slipvariance=0
	local slipVelocity = CalcSlip(angleR,velocity,angularVel,radius)
	local slipspeed = slipVelocity / velocity
	local slipDerivative = (slipspeed - (slipspeedGlobal or 0)) / (dt)
	local tyreHcoef = flancHeight / Tyreminheight
	slipspeedGlobal = slipspeed
	slipDerivativeGlobal = slipDerivative + (slipDerivativeGlobal or 0)
	
	local tyreWcoef = math.min(tyreWidth/0.345,1)

	slipDerivativeDerivative = (slipDerivative - (slipDerivativeP or 0))/ dt
	slipDerivativeP = slipDerivative
	
	local dAngleR = (angleR - (angleRPrev or 0)) / dt
	angleRPrev = angleR
	local Angle2Coef = math.max(math.sin(math.pi * angleR2 / 4 , 0))
	local AngleCoef = math.max(math.sin(math.pi * angleR / 4 , 0))

	local Angle2Ratio = angleR2/math.max(AngleMax2,0.001)
	local AngleRatio = angleR/math.max(AngleMax,0.001)
	local dAngleR2 = (angleR2 - (angleRPrev2 or 0)) / dt
	angleRPrev2 = angleR2
	local maxanglegrip = 10e6 
	local ddAngleR2 = (dAngleR2 - (angleRPrev2D or 0)) / dt
	local ddAngleR = (dAngleR - (angleRPrevD or 0)) / dt
	local tyreWr = tyreWidth/TyremaxW
	angleRPrevD = dAngleR
	angleRPrev2D = dAngleR2
	local dvelocity= (velocity -( velocityPrev or 0))/dt
	local denomV=50
	local denomV2=50
	local coef,n1 = 0,1
	if tyreWr == 1 then n1 = 0.85
	end
	local tire = math.min(10,4*0.12/flancHeight)
	if (angleR-betaDeg)> 1 then
		if tyreWr==1 and dvelocity<0 then
			coef = sinusModifie(angleR,math.max(10*(70-velocity)/70,tire))*math.cos(math.rad(90*(math.min(velocity/70,1)))) + math.max(0.9*math.min(angleR/(5*0.45*0.12/(math.max(tyreWidth,0.2)*flancHeight)),1)+0.1,0)*math.sin(math.rad(90*(math.min(velocity/70,1))))
			if validSurfaceTypes[groundModelName] then
				coef = sinusModifie(angleR,math.max(10*(70-velocity)/70,tire))*math.cos(math.rad(90*(math.min(velocity/70,1))))
			end
		elseif tyreWr~=1 then
			if dvelocity<0 then
				coef =  sinusModifie(angleR,((math.max((30-(velocity or 0))/30,0))^0.5)*12) -- 
			else 
				coef =  sinusModifie(angleR,((math.max((30-(velocity or 0))/30,0))^0.5)*12)*(math.max((60-(velocity or 0))/60,0))
			end
		end
	else
		if tyreWr==1 and dvelocity > 0 then
			coef = math.max(0.9*math.min(angleR/(5*0.45*0.12/(math.max(tyreWidth,0.2)*flancHeight)),1)+0.1,0)*math.cos(math.rad(90*(math.min(velocity/35,1)))) + sinusModifie(angleR,tire)*math.sin(math.rad(90*(math.min(velocity/35,1))))
		elseif tyreWr~=1 and dvelocity > 0 then
			coef = math.max(0.8*math.min(angleR/(5*0.45*0.12/(math.max(tyreWidth,0.2)*flancHeight)),1.5)+0.1,0)*math.cos(math.rad(90*(math.min(velocity/35,1)))) + sinusModifie(angleR,tire)*math.sin(math.rad(90*(math.min(velocity/35,1))))
		elseif tyreWr==1 and dvelocity < 0 then
			coef = sinusModifie(angleR,tire)
		else coef = math.max(0.7*math.min(angleR/(5*0.45*0.12/(math.max(tyreWidth,0.2)*flancHeight)),1.5)+0.5,0)*math.cos(math.rad(90*(math.min(velocity/35,1)))) + math.sin(math.rad(90*(math.min(velocity/35,1))))*sinusModifie(angleR,tire)
		end
	end
	local straightBoost = 0
	local angleStraigthBoost = 8-((math.max((45-(velocity or 0))/45,0))^0.5)*6
	angleStraigthBoost = math.min(math.max(0.5, tyreWidth/0.420),1)*angleStraigthBoost
	if betaDeg < angleStraigthBoost then
		straightBoost = math.max(math.min(((1-((angleR-betaDeg)^0.5))/(1-angleStraigthBoost^0.5)),1),0)
	elseif dAngleR< 0 then straightBoost = 0
	else straightBoost = 0
	end
	local sbm = (1+straightBoost)/2 
	if tyreWratio==1 and dvelocity > 0 then
		straightBoost= tyreWidth((math.max((27-(velocity or 0))/20,0))^0.5)*straightBoost/3
	elseif tyreWratio==1 and dvelocity < 0 then
		sbm = 1
		straightBoost= 0 
	elseif dvelocity < 0 and tyreWratio ~=1 and ddAngleR>0 then
		straightBoost = ((math.max((25-(velocity or 0))/25,0))^0.5)*straightBoost
		if dAngleR < 0 then
			straightBoost = (math.max((math.min(velocity/10,0.5))^0.5,1))*straightBoost*(1.5-softnessCoef)
		end
	elseif dvelocity > 0 and tyreWratio ~=1 then
		coef = coef*(math.max((math.min(velocity/25,0.5))^0.5+0.2,1))
		straightBoost = ((math.max((25-(velocity or 0))/25,0.3))^0.5)*straightBoost
	end
	
	if betaDeg < tire then
		coef = math.sin(math.rad(90*(angleR/tire)))*coef*math.cos(math.rad(90*(math.min(velocity/30,1))))*n1+math.sin(math.rad(90*(math.min(velocity/30,1))))*math.sin(math.rad(90*((angleR-betaDeg)/tire)))*coef
	else coef = math.max(math.sin(math.rad(90*(((tire-angleR)/tire)))),0)*math.cos(math.rad(90*(math.min(velocity/30,1))))*coef+math.sin(math.rad(90*(math.min(velocity/30,1))))*math.sin(math.rad(90*((tire-(angleR-betaDeg))/tire)))*coef
	end

	if slipDerivative > 0 then
			local slipboost = (1+straightBoost*0.1)*sbm*(slipDerivative+slipDerivativeDerivative)/300 
			if slipDerivativeDerivative > 0 then
				activeBoost = math.min(maxanglegrip, activeBoost + slipboost * dt)
			else
				activeBoost = math.min(maxanglegrip, activeBoost - slipboost * dt)
			end
		else
			activeBoost = 0
	end
	
	
	if angleR>0 then
		
		if dAngleR2 > 0 then
			local contribution2 = angleTriche*dAngleR2/denomV2 
			local contribution3 = angleTriche*ddAngleR2/denomV2 
			local contribution = (contribution2 + contribution3)*coef*maxanglegrip*sbm*(1+straightBoost*0.1)
			if ddAngleR2 > 0 then
				
				angleRMemory2 =  math.min(maxanglegrip,angleRMemory2 + contribution * dt)
			else
				angleRMemory2 =  math.min(maxanglegrip,angleRMemory2 - contribution * dt)
			end
		else
			angleRMemory2 = 0
		end
		if dAngleR > 0 then
			local contribution2 = angleTriche*dAngleR2/denomV2 
			local contribution3 = angleTriche*ddAngleR2/denomV2 
			local contribution = (contribution2 + contribution3)*coef*maxanglegrip*sbm*(1+straightBoost*0.1)
			if ddAngleR > 0 then
				angleRMemory =  math.min(maxanglegrip,angleRMemory + contribution * dt)
			else
				angleRMemory =  math.min(maxanglegrip,angleRMemory - contribution * dt)
			end
		else
			angleRMemory = 0
		end
	end

	local tyreGrip = 0.98
	local correction2 =(math.min(math.max(velocity/(35*3)+0.73,1),1.3)*(0.667+treadCoef/1.5))
	
	local correction = 0
	if validSurfaceTypes[groundModelName] then
		tyreGrip = 0.98 * tyreGrip
		if slipspeed > 0.001 then
			local term1 = 0.06 - (0.06 - 0.059) / (1 + math.exp(((slipspeed * 50 - 35) * 1) / 2))
			local term2 = 0.06 - (0.06 - 0.059) / (1 + math.exp(((-slipspeed * 1000 + 8) * 1) / 2))
    
			slipvariance = (term1 + term2 - 0.118) * 300
		else slipvariance = 0.03
		end
		if angularVel == 0 then
			slipvariance = 2
		end
		correction = (1.0 + slipvariance)*(1+angleRMemory*correction2*tyreWr*math.min(1+0.2*0.35/tyreWidth,1.1)) * (1+angleRMemory2*math.min(1+0.2*0.35/tyreWidth,1.1)*correction2*tyreWr)*(1+activeBoost*tyreWr)
	else
		tyreGrip = 1 * tyreGrip 

		slipvariance= (math.exp(-slipspeed*20/1000+8)/3000)*0.64
		correction = (1.0 + slipvariance) *(1+activeBoost*tyreWr) *(1+angleRMemory*tyreWcoef*correction2*tyreWr) * (1+angleRMemory2*tyreWcoef*correction2*tyreWr)-- *(1+straightBoost*0.1*(1+ 0.1 *math.max((200-(velocity or 0))/200,0)))


	end
	tyreGrip = tyreGrip * correction
	tyreGripTable[wheelID] = tyreGrip

    return tyreGrip
end

local function getVehicleSlipAngle()
  local upVec = obj:getDirectionVectorUp()
  local dirVec = obj:getDirectionVector() 
  local worldVel = obj:getVelocity()     

  local rightVecX = dirVec.y * upVec.z - dirVec.z * upVec.y
  local rightVecY = dirVec.z * upVec.x - dirVec.x * upVec.z
  local rightVecZ = dirVec.x * upVec.y - dirVec.y * upVec.x

  local vx = worldVel.x * dirVec.x  + worldVel.y * dirVec.y  + worldVel.z * dirVec.z 
  local vy = worldVel.x * rightVecX + worldVel.y * rightVecY + worldVel.z * rightVecZ 

  local speed = math.sqrt(vx*vx + vy*vy)
  if speed < 0.5 then
    return 0 
  end

  local beta = math.atan2(vy, vx)

  return beta
end

local function update(dt,slipspeedGlobal,slipDerivativeGlobal)

	acc = acc + dt
	while acc >= targetStep do
	local beta = getVehicleSlipAngle()

    local stream = { data = {} }
	local Tyreminheight = 100
	local Tyremaxheight = 0
	local TyreminW = 100
	local TyremaxW = 0
	local AngleMin = 100
	local AngleMax = 0
	local AngleMin2 = 100
	local AngleMax2 = 0
	local totalLoad = 0
	for _, wd in pairs(wheels.wheelRotators) do
		totalLoad = totalLoad + (wd.downForceRaw or 0)
	end

	if totalLoad < 1 then totalLoad = 1 end

	
    for i, wd in pairs(wheels.wheelRotators) do
        local w = wheelCache[i] or {}
        w.name = wd.name
        w.radius = wd.radius
        w.width = wd.tireWidth
        w.wheelDir = wd.wheelDir
        w.angularVelocity = wd.angularVelocity
        w.downForce = wd.downForce
        w.contactMaterialID1 = wd.contactMaterialID1
        w.contactMaterialID2 = wd.contactMaterialID2
        w.treadCoef = wd.treadCoef
        w.softnessCoef = wd.softnessCoef
		w.velocity = wd.velocity
        w.downForceRaw = wd.downForceRaw
		w.betaDeg = math.deg(beta)

		w.loadPercent = (wd.downForceRaw or 0) / totalLoad
		w.flancHeight = wd.radius - wd.hubRadius
		if Tyreminheight>flancHeight then
			Tyreminheight=flancHeight
		end
		if Tyremaxheight<flancHeight then
			Tyremaxheight=flancHeight
		end
		if TyreminW>width then
			TyreminW=width
		end
		if TyremaxW<width then
			TyremaxW=width
		end
		local tyreWratioN = (TyreminW-420) / (TyremaxW-420)
		
		if tyreWratioN ==1 then
			w.tyreWratio = 1
		else	
			w.tyreWratio = tyreWratioN * math.min(math.max((width-273)/(235-273),1),1.666)
		end
        local vectorUp = obj:getDirectionVectorUp()
		
				-- Obtenir la vitesse du véhicule
		--local vel = obj:getVelocity()
		local vel= obj:getVelocity()
		local moveDir = vec3(velocity)
		local moveCarDir = vec3(vel.x, vel.y, vel.z)
		local nodeFrontID = v.data.refNodes[1]
		local vehForward = obj:getDirectionVector()
		local velFront = obj:getNodeVelocity(nodeFrontID)

		local speedFront = velFront:length()  -- en m/s
		local directionFront = velFront:normalized()  -- direction unitaire
		
		-- On ne calcule que si la voiture est réellement en mouvement
		if moveDir:length() > 0.1 then
			moveCarDir:normalize()
			moveDir:normalize()

			-- Construction de la direction de la roue via ses nœuds
			local vectorUp = obj:getDirectionVectorUp()
			local node1 = obj:getNodePosition(wd.node1)
			local node2 = obj:getNodePosition(wd.node2)

			-- Vecteur avant de la roue = axe de roue × verticale du châssis
			local wheelForward = (node2 - node1):cross(vectorUp)

			-- Sécurité si vecteur nul
			if wheelForward:length() > 0.001 then
				wheelForward:normalize()

				-- Projection au sol pour comparer dans le plan horizontal
				local wheelDir = vec3(wheelForward.x, wheelForward.y, wheelForward.z)
				if wheelDir:length() > 0.001 then
					wheelDir:normalize()
					w.angleTriche = vehForward:dot(wheelDir)
					-- Produit scalaire + angle sécurisé
					local dot = math.max(-1, math.min(1, wheelDir:dot(moveDir)))
					local dot2 = math.max(-1, math.min(1, moveDir:dot(directionFront)))
					local angle = math.acos(dot)
					local angle2 = math.acos(dot2)
					local angleDeg = math.abs(math.deg(angle))
					local angleDeg2 = math.abs(math.deg(angle2))

					-- Assignation
					w.AngleR = angleDeg
					w.AngleR2 = angleDeg2
				else
					w.AngleR = 0
					w.AngleR2 = 0
				end
			else
				w.AngleR = 0
				w.AngleR2 = 0
			end
		else
			w.AngleR = 0
			w.AngleR2 = 0
		end
		if AngleMin2>AngleR2 then
			AngleMin2=AngleR2
		end
		if AngleMax2<AngleR2 then
			AngleMax2=AngleR2
		end
		if AngleMin>AngleR then
			AngleMin=AngleR
		end
		if AngleMax<AngleR then
			AngleMax=AngleR
		end
		if softnessCoef<0.2 then
			targetStep = 1/200
		elseif softnessCoef < 0.6 then
			targetStep = 1/100
		elseif softnessCoef < 0.4 then
			targetStep = 1/60
		else targetStep = 1/30
		end
        wheelCache[i] = w
    end

    for i = 0, #wheels.wheelRotators do
        local wheel = obj:getWheel(i)
        if wheel then
            local groundModelName = GetGroundModelData(wheelCache[i].contactMaterialID1)



			local angularVel2 = math.abs(wheelCache[i].angularVelocity) 
			local speedGround = vec3(wheelCache[i].velocity):length()
			local treadCoef2 = wheelCache[i].treadCoef
            local treadCoef = 1.0 - wheelCache[i].treadCoef * 0.45
            local softnessCoef = wheelCache[i].softnessCoef
			local AngleDif = wheelCache[i].AngleMax - wheelCache[i].AngleMin
			local AngleDif2 = wheelCache[i].AngleMax2 - wheelCache[i].AngleMin2

            local tyreGrip = CalculateGrip(i, treadCoef, wheelCache[i].radius, angularVel2, wheelCache[i].width, wheelCache[i].groundModelName, 
				speedGround, wheelCache[i].loadPercent, wheelCache[i].AngleR, propulsionTorque,slipspeedGlobal,dt, totalLoad,wheelCache[i].flancHeight,wheelCache[i].softnessCoef,wheelCache[i].angleTriche,
				slipDerivativeGlobal, wheelCache[i].AngleR2, treadCoef2, wheelCache[i].Tyreminheight, wheelCache[i].Tyremaxheight, AngleDif, AngleDif2, wheelCache[i].AngleMax2, wheelCache[i].AngleMax, wheelCache[i].downForceRaw, wheelCache[i].tyreWratio,TyremaxW,wheelCache[i].betaDeg)--*wheelCache[i].finalGrip



            table.insert(stream.data, {
                name = wheelCache[i].name,
                tread_coef = treadCoef,
                tyreGrip = math.floor(tyreGrip * 1000) / 1000,
                contact_material = groundModelName,

            })


            wheel:setFrictionThermalSensitivity(
                -300,     -- frictionLowTemp              default: -300
                1e7,      -- frictionHighTemp             default: 1e7
                1e-10,    -- frictionLowSlope             default: 1e-10
                1e-10,    -- frictionHighSlope            default: 1e-10
                10,       -- frictionSlopeSmoothCoef      default: 10
                tyreGrip, -- frictionCoefLow              default: 1
                tyreGrip, -- frictionCoefMiddle           default: 1
                tyreGrip  -- frictionCoefHigh             default: 1
            )
        end
    end
    totalTimeMod60 = (totalTimeMod60 + dt) % 60 -- Loops every 60 seconds
    stream.total_time_mod_60 = totalTimeMod60
	acc = acc - targetStep
  end
end



local function onReset()

end

local function onInit()

end

local function vSettingsDebug()
    local count = 0
    htmlTools.dumpToFile(obj.partConfig, "obj")
end

local function onSettingsChanged()

end

local function onVehicleSpawned()
end

M.onSettingsChanged = onSettingsChanged
M.onInit = onInit
M.onReset = onReset
M.update = update
M.onVehicleSpawned = onVehicleSpawned
M.groundModelsCallback = groundModelsCallback

return M
