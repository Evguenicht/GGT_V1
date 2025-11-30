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
local tyre_dataS = require("tyreDataSlip")
local tyre_dataSD = require("tyreDataSlipDirt")


local M = {}
local acc = 0
local targetStep = 1 / 500


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
        -- Transition douce de 0 à 2 degrés (sinus croissant)
        return math.sin(math.rad(90 * (deg / tire)))
	elseif deg >= tire and deg < 1.5*tire then
        -- Transition douce de 0 à 2 degrés (sinus croissant)
        return math.sin(math.rad(90 * ((1.2*tire - deg) / (tire/0.2))))
    elseif deg >= 1.5*tire and deg <= (180-1.5*tire) then
        return 
	elseif deg > (180-1.5*tire) and deg <= (180-tire) then
        -- Transition douce de 0 à 2 degrés (sinus croissant)
        return math.sin(math.rad(90 * (((180-1.2*tire)-deg) / (-tire*0.2))))
    elseif deg > (180-tire) and deg <= 180 then
        -- Transition douce de 178 à 180 degrés (sinus décroissant)
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
	local T_opt, sigma, minAdherence, maxAdherence, minGrip, maxGrip

	local slipvariance
	local slipVelocity = CalcSlip(angleR,velocity,angularVel,radius)
	local slipspeed = slipVelocity * 1000 / velocity
	local slipDerivative = (slipspeed - (slipspeedGlobal or 0)) / (dt)
	local tyreHcoef = flancHeight / Tyreminheight
	slipspeedGlobal = slipspeed
	slipDerivativeGlobal = slipDerivative + (slipDerivativeGlobal or 0)
	--slipDerivativeGlobal = math.min(300, slipDerivativeGlobal)
	

	
	
	local tyreWcoef = math.min((0.190+tyreWidth/2)/0.345,1)
	--local linearCoefficientTimer = 50*0.345*0.12/(tyreWcoef*28*tyreHcoef*(flancHeight/0.12))
	--if slipDerivativeGlobal > 0 then
	--	slipDerivativeGlobal = math.max(0, slipDerivativeGlobal - linearCoefficientTimer * dt)
	--end
	slipDerivativeDerivative = (slipDerivative - (slipDerivativeP or 0))/ dt
	slipDerivativeP = slipDerivative
	
	--local boostFactor = slipDerivativeGlobal / 300

	--local maxBoost = 0.3*(math.min(tyreWidth/0.345,(0.2*tyreWidth+0.069)) * math.min(1, 0.5+treadCoef/2)--*slipDerivativeDerivative
	
	--local activeBoost = maxBoost * boostFactor
	--local tyreWspeedCoef = math.min(1, 0.7 + 0.3 * math.sin(math.pi * velocity / (2*30*tyreWidth))
	--local tyreWspeedCoefInv = math.min(1, 1 - 0.5 * math.sin(math.pi * velocity / (2*290*tyreWidth))




	local dAngleR = (angleR - (angleRPrev or 0)) / dt
	angleRPrev = angleR
	local Angle2Coef = math.max(math.sin(math.pi * angleR2 / 4 , 0))
	local AngleCoef = math.max(math.sin(math.pi * angleR / 4 , 0))

	local Angle2Ratio = angleR2/math.max(AngleMax2,0.001)
	local AngleRatio = angleR/math.max(AngleMax,0.001)
	local dAngleR2 = (angleR2 - (angleRPrev2 or 0)) / dt
	angleRPrev2 = angleR2
	local maxanglegrip = math.min(math.max(tyreWidth/(0.480),0.5),1)*(1.5-softnessCoef)*(tyreWratio^0.5)/0.10
	local ddAngleR2 = (dAngleR2 - (angleRPrev2D or 0)) / dt
	local ddAngleR = (dAngleR - (angleRPrevD or 0)) / dt
	local tyreWr = tyreWidth/TyremaxW
	angleRPrevD = dAngleR
	angleRPrev2D = dAngleR2
	local dvelocity= (velocity -( velocityPrev or 0))/dt
	local denomV=50
	local denomV2=50
	local coef = 0
	--local tire = math.min(10,4*0.12/((math.min(1,treadCoef+0.1))^2*flancHeight))
	if (angleR-betaDeg)> 1 then
		if tyreWr==1 and dvelocity<0 then
			coef = sinusModifie(angleR,10)
			if validSurfaceTypes[groundModelName] then
				coef = sinusModifie(angleR,4)
			end
		elseif tyreWr=/=1 and dvelocity<0 then
			coef =  sinusModifie(angleR,((math.max((30-(velocity or 0))/30,0))^0.5)*12) -- 
		-- elseif tyreWr==1 and dvelocity>0 then
			-- coef = sinusModifie(angleR,10)
		else tyreWr=/=1 and dvelocity>0 then
			coef =  sinusModifie(angleR,((math.max((30-(velocity or 0))/30,0))^0.5)*12)*(math.max((60-(velocity or 0))/60,0))
		end
	else
		coef = math.max(0.6*math.min(angleR/(5*0.45*0.12/(math.max(tyreWidth,0.2)*flancHeight)),1.5),0.4)
	end
	local straightBoost = 0
	local angleStraigthBoost = 8-((math.max((45-(velocity or 0))/45,0))^0.5)*6
	angleStraigthBoost = math.min(math.max(0.5, tyreWidth/0.420),1)*angleStraigthBoost
	if betaDeg < angleStraigthBoost then
		straightBoost = math.max(math.min(((1-((angleR-betaDeg)^0.5))/(1-angleStraigthBoost^0.5)),1),0)
	elseif dAngleR< 0 then straightBoost = 0
	else straightBoost = 0
	end
	local sbm = (1+straightBoost)/2 --straightBoostMultiplier
	if tyreWratio==1 and dvelocity > 0 then
		straightBoost= ((math.max((27-(velocity or 0))/20,0))^0.5)*straightBoost/3
	elseif tyreWratio==1 and dvelocity < 0 then
		sbm = 1
		straightBoost= 0 --straightBoost/3
	elseif dvelocity < 0 and tyreWratio =/=1 then
		straightBoost = ((math.max((50-(velocity or 0))/50,0))^0.5)*straightBoost
		if dAngleR < 0 then
			straightBoost = (math.max((math.min(velocity/10,0.5))^0.5+0.5,1))*straightBoost*(1.5-softnessCoef)
		end
	elseif dvelocity > 0 and tyreWratio =/=1 then
		coef = coef*(math.max((math.min(velocity/30,0.5))^0.5+0.2,1))
		straightBoost = ((math.max((40-(velocity or 0))/30,0))^0.5)*straightBoost
	end
	

	if slipDerivative > 0 then
			local slipboost = sbm*(slipDerivative+slipDerivativeDerivative)/300 -- montée lente pour petites dérivées
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
			local contribution2 = angleTriche*dAngleR2/denomV2 -- montée lente pour petites dérivées boostAngle * rotateAngleCoef *
			local contribution3 = angleTriche*ddAngleR2/denomV2 -- montée lente pour petites dérivées boostAngle * rotateAngleCoef *
			local contribution = (contribution2 + contribution3)*coef*maxanglegrip*sbm
			if ddAngleR2 > 0 then
				--local contribution2 = math.min( 0.3 * math.sin(dAngleR2 * math.pi/2), 0.3)
				angleRMemory2 =  math.min(maxanglegrip,angleRMemory2 + contribution * dt)
			else
				angleRMemory2 =  math.min(maxanglegrip,angleRMemory2 - contribution * dt)
			end
		else
			angleRMemory2 = 0
		end
		if dAngleR > 0 then
			local contribution2 = angleTriche*dAngleR2/denomV2 -- montée lente pour petites dérivées boostAngle * rotateAngleCoef *
			local contribution3 = angleTriche*ddAngleR2/denomV2 -- montée lente pour petites dérivées boostAngle * rotateAngleCoef *
			local contribution = (contribution2 + contribution3)*coef*maxanglegrip*sbm
			if ddAngleR > 0 then
				--local contribution2 = math.min( 0.3 * math.sin(dAngleR2 * math.pi/2), 0.3)
				angleRMemory =  math.min(maxanglegrip,angleRMemory + contribution * dt)
			else
				angleRMemory =  math.min(maxanglegrip,angleRMemory - contribution * dt)
			end
		else
			angleRMemory = 0
		end
	end

	local tyreGrip = (1 + (((1-tyreWcoef)/10)^2)*8)*(1-math.min(velocity/250))
	local correction2 =(math.min(math.max(velocity/(35*3)+0.73,1),1.3)*(0.667+treadCoef/1.5))-- (1.0 + slipvariance) *(1.0 + 0.6*activeBoost*(math.min(tyreWidth/0.345,1))*math.min(1, 0.67+(1-softnessCoef)/3)) * (1+angleRMemory*tyreWcoef) * (1+angleRMemory2*tyreWcoef*tyreHcoef) * (1+0.05*Angle2Coef) * (1+Angle2Ratio*0.5)
	
	local correction = 0
	if validSurfaceTypes[groundModelName] then
		tyreGrip = 0.947 * tyreGrip

		slipvariance = tyre_dataSD.SlipDirtToGrip.slicks[math.floor(slipspeed)] or 0.686
		slipvariance= slipvariance / 10
		if angularVel == 0 then
			slipvariance = 2 -- previous value : 0.3
		end
		correction = (1.0 + slipvariance)*(1+angleRMemory*tyreWcoef*correction2*tyreWr) * (1+angleRMemory2*tyreWcoef*correction2*tyreWr)*(1+activeBoost*tyreWr)*(1+straightBoost*0.1*(1+ 0.1 *math.max((50-(velocity or 0))/50,0))) --* (1+0.05*Angle2Coef) * (1+Angle2Ratio*0.5) --* (1+angleRMemory2*tyreWcoef*tyreHcoef*(0.5+Angle2Ratio/2)) * (1+0.2*Angle2Coef) * (1+Angle2Ratio*0.7)

		--tyreGrip = tyreGrip * correction -- (1.0 + slipvariance) --* adherence 
	else
		--tyreGrip = 0.9433962
		tyreGrip = 0.98 * tyreGrip -- * math.max(0.9,math.min(1,downForceRaw/(2*tyreWidth*flancHeight/0.12)))
		slipvariance = tyre_dataS.SlipToGrip.slicks[math.floor(slipspeed)] or 0
		slipvariance = slipvariance * 0.8
		correction = (1.0 + slipvariance)*(1+straightBoost*0.1*(1+ 0.1 *math.max((200-(velocity or 0))/200,0))) *(1+activeBoost*tyreWr) *(1+angleRMemory*tyreWcoef*correction2*tyreWr) * (1+angleRMemory2*tyreWcoef*correction2*tyreWr)
--		correction = (1.0 + slipvariance) *(1.0 + 0.5*activeBoost*tyreWcoef*(0.67+(1-softnessCoef)/3)*tyreWcoef*correction2*tyreWr) *(1+angleRMemory*tyreWcoef*correction2*tyreWr) * (1+angleRMemory2*tyreWcoef*correction2*tyreWr)

		--(1.0 + slipvariance) *((1.0 + (0.4+math.min(0.3, 0.3 * math.sin(math.pi * velocity / (2*30*tyreWidth))))*activeBoost*(math.min(tyreWidth/0.345,1))*math.min(1, 0.67+(1-softnessCoef)/3)) * (1+angleRMemory*tyreWspeedCoef*tyreWcoef*(0.5+AngleRatio/2)) * (1+angleRMemory2*tyreWcoef*tyreHcoef*(0.5+Angle2Ratio/2)) *tyreWcoef --(1+0.05*Angle2Coef) * (1+Angle2Ratio*0.5))--*tyreHcoef -- * angleGrip  * adherence
		--tyreGrip = tyreGrip * correction -- * angleGrip  * adherence

	end
	tyreGrip = tyreGrip * correction

	


		
	tyreGripTable[wheelID] = tyreGrip

    return tyreGrip
end

local function getVehicleSlipAngle()
  -- Vecteurs de repère véhicule
  local upVec = obj:getDirectionVectorUp()
  local dirVec = obj:getDirectionVector() -- avant de la voiture
  local worldVel = obj:getVelocity()      -- vitesse en monde

  -- Produit vectoriel pour avoir le vecteur "droite"
  local rightVecX = dirVec.y * upVec.z - dirVec.z * upVec.y
  local rightVecY = dirVec.z * upVec.x - dirVec.x * upVec.z
  local rightVecZ = dirVec.x * upVec.y - dirVec.y * upVec.x

  -- Projection de la vitesse dans le repère voiture
  local vx = worldVel.x * dirVec.x  + worldVel.y * dirVec.y  + worldVel.z * dirVec.z  -- avant
  local vy = worldVel.x * rightVecX + worldVel.y * rightVecY + worldVel.z * rightVecZ -- latéral (droite +)

  local speed = math.sqrt(vx*vx + vy*vy)
  if speed < 0.5 then
    return 0 -- à très basse vitesse, on considère 0 pour éviter les trucs débiles
  end

  -- Slip angle véhicule (β), en radians
  local beta = math.atan2(vy, vx) -- signe : >0 si glisse vers la droite

  return beta
end

local function limitThrottle()
    -- Lire la valeur d'accélérateur du joueur (0 → 1)
    local throttle = electrics.values.throttle or 0

    -- La limiter à 50%
    local limited = math.min(throttle, 0.5)

    -- Réécrire la valeur de throttle dans le véhicule
    electrics.values.throttle = limited
end



local function update(dt,slipspeedGlobal,slipDerivativeGlobal)
--local function updateGFX(dt,slipspeedGlobal,slipDerivativeGlobal)
	--local rawInput = electrics.values.rawThrottle  -- ce que le joueur a VRAIMENT mis
	acc = acc + dt
	while acc >= targetStep do
    limitThrottle()

	-- Total load
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

	-- Sécurité pour éviter division par zéro
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
        w.isBroken = wd.isBroken
		w.velocity = wd.velocity
        w.downForceRaw = wd.downForceRaw
		w.betaDeg = math.deg(beta)

		w.noLoad = wd.noLoadCoef or 1.0
		w.loadSlope = wd.loadSensitivitySlope or 0.0001
		
		--applyCustomTC(wd,betaDeg, dt)
		
		
		
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
		
		if tyreWratioN =1 then
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
