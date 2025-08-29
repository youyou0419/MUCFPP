
"use strict";

let AttitudeCommand = require('./AttitudeCommand.js');
let RawImu = require('./RawImu.js');
let ControllerState = require('./ControllerState.js');
let YawrateCommand = require('./YawrateCommand.js');
let MotorCommand = require('./MotorCommand.js');
let VelocityXYCommand = require('./VelocityXYCommand.js');
let Altimeter = require('./Altimeter.js');
let MotorStatus = require('./MotorStatus.js');
let ServoCommand = require('./ServoCommand.js');
let Supply = require('./Supply.js');
let HeadingCommand = require('./HeadingCommand.js');
let RawRC = require('./RawRC.js');
let RuddersCommand = require('./RuddersCommand.js');
let HeightCommand = require('./HeightCommand.js');
let MotorPWM = require('./MotorPWM.js');
let ThrustCommand = require('./ThrustCommand.js');
let Compass = require('./Compass.js');
let RawMagnetic = require('./RawMagnetic.js');
let PositionXYCommand = require('./PositionXYCommand.js');
let RC = require('./RC.js');
let VelocityZCommand = require('./VelocityZCommand.js');

module.exports = {
  AttitudeCommand: AttitudeCommand,
  RawImu: RawImu,
  ControllerState: ControllerState,
  YawrateCommand: YawrateCommand,
  MotorCommand: MotorCommand,
  VelocityXYCommand: VelocityXYCommand,
  Altimeter: Altimeter,
  MotorStatus: MotorStatus,
  ServoCommand: ServoCommand,
  Supply: Supply,
  HeadingCommand: HeadingCommand,
  RawRC: RawRC,
  RuddersCommand: RuddersCommand,
  HeightCommand: HeightCommand,
  MotorPWM: MotorPWM,
  ThrustCommand: ThrustCommand,
  Compass: Compass,
  RawMagnetic: RawMagnetic,
  PositionXYCommand: PositionXYCommand,
  RC: RC,
  VelocityZCommand: VelocityZCommand,
};
