-- This script runs in a thread. You can also use a non-threaded script instead
-- Following commands are implemented:
-- k3Handle=simExtK3_create(table_2 wheelMotorHandles,table_2 colorSensorHandles,table_9 IrSensorHandles,table_5 usSensorHandles,table_6 armMotorHandles,table_3 fingerMotorHandles,table_2 gripperDistSensHandles,table_2 gripperColSensHandles,number uiHandle) 
-- boolean result=simExtK3_destroy(number k3Handle)
-- distance_inMeters=simExtK3_getInfrared(k3Handle,index_of_ir_sensor_0_to_8)
-- distance_inMeters=simExtK3_getUltrasonic(k3Handle,index_of_us_sensor_0_to_5)
-- intensity_0_to_1=simExtK3_getLineSensor(k3Handle,index_of_line_sensor_0_to_1)
-- distance_inMeters=simExtK3_getGripperProxSensor(k3Handle,index_of_finger_prox_sensor_0_to_1)
-- boolean result=simExtK3_setVelocity(k3Handle,velocityLeft_radPerSec,velocityRight_radPerSec)
-- boolean result=simExtK3_setArmPosition(k3Handle,position_300_to_900)
-- boolean result=simExtK3_setGripperGap(k3Handle,gap_0_to_170)

maxVel=2*math.pi

threadFunction=function()
local irSensor={-1,-1,-1,-1,-1,-1,-1,-1,-1}
local usSensor={-1,-1,-1,-1,-1}
    while simGetSimulationState()~=sim_simulation_advancing_abouttostop do
		for i=0,4,1 do
			usSensor[i+1]=simExtK3_getUltrasonic(k3Handle,i)
			simAddStatusbarMessage('US'..i..': '..usSensor[i+1])
		end
		
		if (usSensor[1]>1 and usSensor[3]>1) then
			velLeft=maxVel
			 velRight=maxVel
		elseif (usSensor[1]<1 and usSensor[3]>1) then
			velLeft=maxVel
			velRight=-maxVel
		elseif (usSensor[1]>1 and usSensor[3]<1) then
			velLeft=-maxVel
			velRight=maxVel
		elseif (usSensor[1]<1 and usSensor[3]<1) then
			velLeft=-maxVel
			velRight=-maxVel
		end		
		
		-- for i=0,8,1 do
			-- irSensor[i+1]=simExtK3_getInfrared(k3Handle,i)
			-- simAddStatusbarMessage('IR'..i..': '..irSensor[i+1])
		-- end
		-- if (irSensor[2]>0.2 and irSensor[3]>0.2) then
			-- velLeft=maxVel
			-- velRight=maxVel
		-- elseif (irSensor[2]<0.2 and irSensor[3]>0.2) then
			-- velLeft=maxVel
			-- velRight=-maxVel
		-- elseif (irSensor[2]>0.2 and irSensor[3]<0.2) then
			-- velLeft=-maxVel
			-- velRight=maxVel
		-- elseif (irSensor[2]<0.2 and irSensor[3]<0.2) then
			-- velLeft=-maxVel
			-- velRight=-maxVel
		-- end

        simExtK3_setVelocity(k3Handle,velLeft,velRight) -- Set desired left and right motor velocities
    end
end

-- Put some initialization code here:
-- Check if the required plugin is there:
-- ************************************************
moduleName=0
moduleVersion=0
index=0
kheperaModuleNotFound=true
while moduleName do
    moduleName,moduleVersion=simGetModuleName(index)
    if (moduleName=='K3') then
        kheperaModuleNotFound=false
    end
    index=index+1
end
if (kheperaModuleNotFound) then
    simDisplayDialog('Error','Khepera3 plugin was not found. (v_repExtK3.dll)&&nSimulation will not run properly',sim_dlgstyle_ok,true,nil,{0.8,0,0,0,0,0},{0.5,0,0,1,1,1})
end
-- ************************************************

disableUltrasonicSensors=false
if (disableUltrasonicSensors) then
    for i=1,5,1 do
        simSetExplicitHandling(simGetObjectHandle('K3_ultrasonicSensor'..i),1)
    end
end

-- Create the K3 object:
local wheelMotorHandles={simGetObjectHandle('K3_leftWheelMotor'),simGetObjectHandle('K3_rightWheelMotor')}
local colorSensorHandles={simGetObjectHandle('K3_colorSensorLeft'),simGetObjectHandle('K3_colorSensorRight')}
local irSensorHandles={}
for i=1,9,1 do
    irSensorHandles[#irSensorHandles+1]=simGetObjectHandle('K3_infraredSensor'..i)
end
local usSensorHandles={}
for i=1,5,1 do
    usSensorHandles[#usSensorHandles+1]=simGetObjectHandle('K3_ultrasonicSensor'..i)
end

local armMotorHandles={-1,-1,-1,-1,-1,-1}
armMotorHandles[1]=simGetObjectHandle('K3_gripper_armJoint1')
armMotorHandles[2]=simGetObjectHandle('K3_gripper_armJoint2')
armMotorHandles[3]=simGetObjectHandle('K3_gripper_armAuxJoint1')
armMotorHandles[4]=simGetObjectHandle('K3_gripper_armAuxJoint2')
armMotorHandles[5]=simGetObjectHandle('K3_gripper_armAuxJoint3')
armMotorHandles[6]=simGetObjectHandle('K3_gripper_armAuxJoint4')
local fingerMotorHandles={-1,-1,-1}
fingerMotorHandles[1]=simGetObjectHandle('K3_gripper_fingers')
fingerMotorHandles[2]=simGetObjectHandle('K3_gripper_fingersAux')
fingerMotorHandles[3]=simGetObjectHandle('K3_gripper_fingersAux0')
local gripperDistSensHandles={simGetObjectHandle('K3_gripper_leftDistanceSensor'),simGetObjectHandle('K3_gripper_rightDistanceSensor')}
local gripperColSensHandles={simGetObjectHandle('K3_gripper_leftColorSensor'),simGetObjectHandle('K3_gripper_rightColorSensor')}
local uiHandle=simGetUIHandle('K3_stateVisualization')

k3Handle=simExtK3_create(wheelMotorHandles,colorSensorHandles,irSensorHandles,usSensorHandles,armMotorHandles,fingerMotorHandles,gripperDistSensHandles,gripperColSensHandles,uiHandle)

simExtK3_setArmPosition(k3Handle,900)

-- Here we execute the regular thread code:
res,err=xpcall(threadFunction,function(err) return debug.traceback(err) end)
if not res then
    simAddStatusbarMessage('Lua runtime error: '..err)
end

-- Clean-up:
-- Destroy the K3 object:
simExtK3_destroy(k3Handle)

