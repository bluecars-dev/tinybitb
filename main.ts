/**
 * TINY-BIT PRO - Complete Odometry, PIDF, and SLAM Library
 * Features: Motor PIDF Control, Dead Reckoning, Distance/Angle Driving, SLAM Mapping
 */

//% color="#006400" weight=20 icon="\uf1b9" block="Tinybit Pro"
namespace TinybitPro {

    // ==========================================
    // 1. CALIBRATION & TUNING VARIABLES
    // ==========================================
    
    export let WHEELBASE = 0.082;           
    export let M_PER_PWM_S = 0.00215;       
    export let WHEEL_DIAMETER = 0.065;      
    
    export let kP = 1.4;
    export let kI = 0.02;
    export let kD = 0.15;
    export let kF = 35;                     
    
    export let DISTANCE_TOLERANCE = 0.01;   
    export let ANGLE_TOLERANCE = 2;         
    export let SONAR_MAX_RANGE = 1.5;       
    export let SONAR_MIN_RANGE = 0.05;      
    
    export let GRID_RES = 0.05;             
    export let MAP_DIM = 40;                

    // ==========================================
    // 2. STATE VARIABLES
    // ==========================================
    
    export let x = 0.0, y = 0.0;
    export let theta = 0.0;                 
    
    let motorLeftTarget = 0;
    let motorRightTarget = 0;
    let motorLeftActual = 0;
    let motorRightActual = 0;
    let motorLeftError = 0;
    let motorRightError = 0;
    
    let leftIntegral = 0.0;
    let rightIntegral = 0.0;
    let leftPrevError = 0.0;
    let rightPrevError = 0.0;
    let lastPIDFTime = control.micros();
    let lastOdomTime = control.micros();
    
    let isMoving = false;
    let movementMode = 0; 

    const PWM_ADD = 0x01;
    const MOTOR_REG = 0x02;
    const RGB_REG = 0x01;
    
    let motorBuf = pins.createBuffer(5);
    motorBuf[0] = MOTOR_REG;
    
    let rgbBuf = pins.createBuffer(4);
    rgbBuf[0] = RGB_REG;
    
    let mapGrid = pins.createBuffer(MAP_DIM * MAP_DIM);
    let sonarHistory = [0, 0, 0, 0, 0];
    let sonarIdx = 0;

    // ==========================================
    // 3. INITIALIZATION
    // ==========================================
    
    /**
     * Initialize the robot system. Call this once at startup.
     */
    //% blockId=tinybitpro_init block="initialize Tinybit Pro"
    //% weight=100
    export function init(): void {
        serial.writeLine("[INIT] TinyBit Pro System Started");
        resetPose();
        resetMap();
        lastPIDFTime = control.micros();
        lastOdomTime = control.micros();
        isMoving = false;
    }

    // ==========================================
    // 4. MOVEMENT COMMANDS
    // ==========================================

    /**
     * Drive straight for a distance in centimeters.
     */
    //% blockId=tinybitpro_drive_straight 
    //% block="drive straight %distanceCm cm at speed %speed"
    //% speed.defl=150
    //% weight=90
    export function driveStraight(distanceCm: number, speed: number = 150): void {
        let targetDist = distanceCm / 100.0;
        resetPose();
        isMoving = true;
        movementMode = 1;
        
        while (isMoving && getDistanceFromOrigin() < targetDist) {
            setMotorTarget(speed, speed);
            updateMotorPIDF();
            updatePose(motorLeftActual, motorRightActual);
            updateMap();
            basic.pause(10);
        }
        stop();
    }

    /**
     * Turn by a specific angle in degrees.
     */
    //% blockId=tinybitpro_turn_angle 
    //% block="turn %angleDeg degrees at speed %speed"
    //% speed.defl=100
    //% weight=85
    export function turnByAngle(angleDeg: number, speed: number = 100): void {
        let targetRad = angleDeg * 0.0174533;
        let initialTheta = theta;
        isMoving = true;
        movementMode = 2;
        
        while (isMoving) {
            let rotated = Math.abs(theta - initialTheta);
            if (rotated >= Math.abs(targetRad)) break;
            
            if (angleDeg > 0) {
                setMotorTarget(-speed, speed);
            } else {
                setMotorTarget(speed, -speed);
            }
            
            updateMotorPIDF();
            updatePose(motorLeftActual, motorRightActual);
            basic.pause(10);
        }
        stop();
    }

    /**
     * Stop all motors immediately.
     */
    //% blockId=tinybitpro_stop block="stop motors"
    //% weight=100
    export function stop(): void {
        rawDrive(0, 0);
        isMoving = false;
        setMotorTarget(0, 0);
    }

    // ==========================================
    // 5. SENSORS & SLAM
    // ==========================================

    /**
     * Get filtered sonar distance (cm).
     */
    //% blockId=tinybitpro_get_sonar block="sonar distance (cm)"
    //% weight=80
    export function getSonar(): number {
        pins.digitalWritePin(DigitalPin.P16, 0);
        control.waitMicros(2);
        pins.digitalWritePin(DigitalPin.P16, 1);
        control.waitMicros(10);
        pins.digitalWritePin(DigitalPin.P16, 0);
        let d = pins.pulseIn(DigitalPin.P15, PulseValue.High, 25000) / 58;
        if (d > 2 && d < 400) {
            sonarHistory[sonarIdx] = d;
            sonarIdx = (sonarIdx + 1) % 5;
        }
        let sorted = sonarHistory.slice().sort((a, b) => a - b);
        return sorted[2];
    }

    /**
     * Reset the robot position (X, Y, and Angle) to zero.
     */
    //% blockId=tinybitpro_reset_pose block="reset robot position"
    //% weight=70
    export function resetPose(): void {
        x = 0.0;
        y = 0.0;
        theta = 0.0;
        lastOdomTime = control.micros();
    }

    // ==========================================
    // 6. LIGHTS (RGB)
    // ==========================================

    export enum RGBColor {
        OFF = 0, Red = 1, Green = 2, Blue = 3,
        White = 4, Cyan = 5, Magenta = 6, Yellow = 7
    }

    /**
     * Set the Tiny:bit LEDs to a color.
     */
    //% blockId=tinybitpro_set_rgb block="set car color to %color"
    //% weight=60
    export function setRGB(color: RGBColor): void {
        let r = 0, g = 0, b = 0;
        switch (color) {
            case RGBColor.Red: r = 255; break;
            case RGBColor.Green: g = 255; break;
            case RGBColor.Blue: b = 255; break;
            case RGBColor.White: r = 255; g = 255; b = 255; break;
            case RGBColor.Cyan: g = 255; b = 255; break;
            case RGBColor.Magenta: r = 255; b = 255; break;
            case RGBColor.Yellow: r = 255; g = 255; break;
        }
        setRGBRaw(r, g, b);
    }

    // ==========================================
    // INTERNAL LOGIC (Hidden from Blocks)
    // ==========================================

    export function setRGBRaw(r: number, g: number, b: number): void {
        rgbBuf[1] = Math.constrain(r, 0, 255);
        rgbBuf[2] = Math.constrain(g, 0, 255);
        rgbBuf[3] = Math.constrain(b, 0, 255);
        pins.i2cWriteBuffer(PWM_ADD, rgbBuf);
    }

    export function rawDrive(left: number, right: number): void {
        let l = Math.constrain(left, -255, 255);
        let r = Math.constrain(right, -255, 255);
        motorBuf[1] = l > 0 ? l : 0;
        motorBuf[2] = l < 0 ? -l : 0;
        motorBuf[3] = r > 0 ? r : 0;
        motorBuf[4] = r < 0 ? -r : 0;
        pins.i2cWriteBuffer(PWM_ADD, motorBuf);
        motorLeftActual = l;
        motorRightActual = r;
    }

    export function setMotorTarget(left: number, right: number): void {
        motorLeftTarget = Math.constrain(left, -255, 255);
        motorRightTarget = Math.constrain(right, -255, 255);
    }

    export function updateMotorPIDF(): void {
        let now = control.micros();
        let dt = (now - lastPIDFTime) / 1000000.0;
        if (dt <= 0) dt = 0.001;
        lastPIDFTime = now;
        let leftError = motorLeftTarget - motorLeftActual;
        leftIntegral = Math.constrain(leftIntegral + leftError * dt, -100, 100);
        let leftPWM = motorLeftTarget + (kP * leftError + kI * leftIntegral);
        let rightError = motorRightTarget - motorRightActual;
        rightIntegral = Math.constrain(rightIntegral + rightError * dt, -100, 100);
        let rightPWM = motorRightTarget + (kP * rightError + kI * rightIntegral);
        rawDrive(leftPWM, rightPWM);
    }

    export function updatePose(lp: number, rp: number): void {
        let now = control.micros();
        let dt = (now - lastOdomTime) / 1000000.0;
        if (dt <= 0) dt = 0.001;
        lastOdomTime = now;
        let vLinear = ((rp + lp) / 2.0) * M_PER_PWM_S;
        let vAngular = ((rp - lp) * M_PER_PWM_S) / WHEELBASE;
        theta += vAngular * dt;
        while (theta > Math.PI) theta -= 2 * Math.PI;
        while (theta < -Math.PI) theta += 2 * Math.PI;
        x += vLinear * Math.cos(theta) * dt;
        y += vLinear * Math.sin(theta) * dt;
    }

    export function getDistanceFromOrigin(): number {
        return Math.sqrt(x*x + y*y);
    }

    export function updateMap(): void {
        let d = getSonar() / 100.0;
        if (d < SONAR_MIN_RANGE || d > SONAR_MAX_RANGE) return;
        let wallX = x + (d * Math.cos(theta));
        let wallY = y + (d * Math.sin(theta));
        let gx = Math.floor(wallX / GRID_RES) + (MAP_DIM / 2);
        let gy = Math.floor(wallY / GRID_RES) + (MAP_DIM / 2);
        if (gx >= 0 && gx < MAP_DIM && gy >= 0 && gy < MAP_DIM) {
            let index = gy * MAP_DIM + gx;
            if (mapGrid[index] < 250) mapGrid[index] += 25;
        }
    }

    export function resetMap(): void {
        for (let i = 0; i < MAP_DIM * MAP_DIM; i++) mapGrid[i] = 0;
    }
}
