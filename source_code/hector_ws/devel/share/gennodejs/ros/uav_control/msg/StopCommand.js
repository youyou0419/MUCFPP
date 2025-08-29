// Auto-generated. Do not edit!

// (in-package uav_control.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class StopCommand {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.stop = null;
      this.blocker_id = null;
    }
    else {
      if (initObj.hasOwnProperty('stop')) {
        this.stop = initObj.stop
      }
      else {
        this.stop = false;
      }
      if (initObj.hasOwnProperty('blocker_id')) {
        this.blocker_id = initObj.blocker_id
      }
      else {
        this.blocker_id = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type StopCommand
    // Serialize message field [stop]
    bufferOffset = _serializer.bool(obj.stop, buffer, bufferOffset);
    // Serialize message field [blocker_id]
    bufferOffset = _serializer.string(obj.blocker_id, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type StopCommand
    let len;
    let data = new StopCommand(null);
    // Deserialize message field [stop]
    data.stop = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [blocker_id]
    data.blocker_id = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.blocker_id);
    return length + 5;
  }

  static datatype() {
    // Returns string type for a message object
    return 'uav_control/StopCommand';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1ee931530b17c39eb5a3b97eb6933cbd';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool stop
    string blocker_id  # 导致死锁的无人机ID
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new StopCommand(null);
    if (msg.stop !== undefined) {
      resolved.stop = msg.stop;
    }
    else {
      resolved.stop = false
    }

    if (msg.blocker_id !== undefined) {
      resolved.blocker_id = msg.blocker_id;
    }
    else {
      resolved.blocker_id = ''
    }

    return resolved;
    }
};

module.exports = StopCommand;
