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

class DeadlockInfo {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.blocker_id = null;
      this.blocked_id = null;
      this.is_bilateral = null;
    }
    else {
      if (initObj.hasOwnProperty('blocker_id')) {
        this.blocker_id = initObj.blocker_id
      }
      else {
        this.blocker_id = '';
      }
      if (initObj.hasOwnProperty('blocked_id')) {
        this.blocked_id = initObj.blocked_id
      }
      else {
        this.blocked_id = '';
      }
      if (initObj.hasOwnProperty('is_bilateral')) {
        this.is_bilateral = initObj.is_bilateral
      }
      else {
        this.is_bilateral = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DeadlockInfo
    // Serialize message field [blocker_id]
    bufferOffset = _serializer.string(obj.blocker_id, buffer, bufferOffset);
    // Serialize message field [blocked_id]
    bufferOffset = _serializer.string(obj.blocked_id, buffer, bufferOffset);
    // Serialize message field [is_bilateral]
    bufferOffset = _serializer.bool(obj.is_bilateral, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DeadlockInfo
    let len;
    let data = new DeadlockInfo(null);
    // Deserialize message field [blocker_id]
    data.blocker_id = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [blocked_id]
    data.blocked_id = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [is_bilateral]
    data.is_bilateral = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.blocker_id);
    length += _getByteLength(object.blocked_id);
    return length + 9;
  }

  static datatype() {
    // Returns string type for a message object
    return 'uav_control/DeadlockInfo';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '38f3084008c30c3c1c815f245eba16b8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string blocker_id    # 造成阻塞的无人机ID
    string blocked_id   # 被阻塞的无人机ID
    bool is_bilateral   # true表示双向死锁，false表示单向阻塞
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DeadlockInfo(null);
    if (msg.blocker_id !== undefined) {
      resolved.blocker_id = msg.blocker_id;
    }
    else {
      resolved.blocker_id = ''
    }

    if (msg.blocked_id !== undefined) {
      resolved.blocked_id = msg.blocked_id;
    }
    else {
      resolved.blocked_id = ''
    }

    if (msg.is_bilateral !== undefined) {
      resolved.is_bilateral = msg.is_bilateral;
    }
    else {
      resolved.is_bilateral = false
    }

    return resolved;
    }
};

module.exports = DeadlockInfo;
