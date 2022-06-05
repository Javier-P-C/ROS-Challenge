
"use strict";

let SetLoad = require('./SetLoad.js')
let MoveAxisAngle = require('./MoveAxisAngle.js')
let PlayTraj = require('./PlayTraj.js')
let SetFloat32 = require('./SetFloat32.js')
let SetToolModbus = require('./SetToolModbus.js')
let MoveVelocity = require('./MoveVelocity.js')
let GetErr = require('./GetErr.js')
let GetInt32 = require('./GetInt32.js')
let TCPOffset = require('./TCPOffset.js')
let SetDigitalIO = require('./SetDigitalIO.js')
let GripperState = require('./GripperState.js')
let SetInt16 = require('./SetInt16.js')
let SetAxis = require('./SetAxis.js')
let GripperMove = require('./GripperMove.js')
let GetAnalogIO = require('./GetAnalogIO.js')
let SetString = require('./SetString.js')
let Move = require('./Move.js')
let GripperConfig = require('./GripperConfig.js')
let GetControllerDigitalIO = require('./GetControllerDigitalIO.js')
let MoveVelo = require('./MoveVelo.js')
let ClearErr = require('./ClearErr.js')
let SetControllerAnalogIO = require('./SetControllerAnalogIO.js')
let SetMultipleInts = require('./SetMultipleInts.js')
let GetDigitalIO = require('./GetDigitalIO.js')
let GetFloat32List = require('./GetFloat32List.js')
let ConfigToolModbus = require('./ConfigToolModbus.js')

module.exports = {
  SetLoad: SetLoad,
  MoveAxisAngle: MoveAxisAngle,
  PlayTraj: PlayTraj,
  SetFloat32: SetFloat32,
  SetToolModbus: SetToolModbus,
  MoveVelocity: MoveVelocity,
  GetErr: GetErr,
  GetInt32: GetInt32,
  TCPOffset: TCPOffset,
  SetDigitalIO: SetDigitalIO,
  GripperState: GripperState,
  SetInt16: SetInt16,
  SetAxis: SetAxis,
  GripperMove: GripperMove,
  GetAnalogIO: GetAnalogIO,
  SetString: SetString,
  Move: Move,
  GripperConfig: GripperConfig,
  GetControllerDigitalIO: GetControllerDigitalIO,
  MoveVelo: MoveVelo,
  ClearErr: ClearErr,
  SetControllerAnalogIO: SetControllerAnalogIO,
  SetMultipleInts: SetMultipleInts,
  GetDigitalIO: GetDigitalIO,
  GetFloat32List: GetFloat32List,
  ConfigToolModbus: ConfigToolModbus,
};
