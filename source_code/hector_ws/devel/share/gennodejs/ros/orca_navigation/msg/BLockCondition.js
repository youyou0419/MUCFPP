// Auto-generated. Do not edit!

// (in-package orca_navigation.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class BLockCondition {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.blocker_id = null;
      this.blocked_id = null;
      this.stamp = null;
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
      if (initObj.hasOwnProperty('stamp')) {
        this.stamp = initObj.stamp
      }
      else {
        this.stamp = {secs: 0, nsecs: 0};
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type BLockCondition
    // Serialize message field [blocker_id]
    bufferOffset = _serializer.string(obj.blocker_id, buffer, bufferOffset);
    // Serialize message field [blocked_id]
    bufferOffset = _serializer.string(obj.blocked_id, buffer, bufferOffset);
    // Serialize message field [stamp]
    bufferOffset = _serializer.time(obj.stamp, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type BLockCondition
    let len;
    let data = new BLockCondition(null);
    // Deserialize message field [blocker_id]
    data.blocker_id = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [blocked_id]
    data.blocked_id = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [stamp]
    data.stamp = _deserializer.time(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.blocker_id);
    length += _getByteLength(object.blocked_id);
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'orca_navigation/BLockCondition';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '159eaf14713728dcfaf35256e7ad2445';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string blocker_id
    string blocked_id
    time stamp
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new BLockCondition(null);
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

    if (msg.stamp !== undefined) {
      resolved.stamp = msg.stamp;
    }
    else {
      resolved.stamp = {secs: 0, nsecs: 0}
    }

    return resolved;
    }
};

module.exports = BLockCondition;
