// Auto-generated. Do not edit!

// (in-package uav_control.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class LDRInfo {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.uav_ids = null;
      this.positions = null;
      this.min_x = null;
      this.max_x = null;
      this.min_y = null;
      this.max_y = null;
    }
    else {
      if (initObj.hasOwnProperty('uav_ids')) {
        this.uav_ids = initObj.uav_ids
      }
      else {
        this.uav_ids = [];
      }
      if (initObj.hasOwnProperty('positions')) {
        this.positions = initObj.positions
      }
      else {
        this.positions = [];
      }
      if (initObj.hasOwnProperty('min_x')) {
        this.min_x = initObj.min_x
      }
      else {
        this.min_x = 0.0;
      }
      if (initObj.hasOwnProperty('max_x')) {
        this.max_x = initObj.max_x
      }
      else {
        this.max_x = 0.0;
      }
      if (initObj.hasOwnProperty('min_y')) {
        this.min_y = initObj.min_y
      }
      else {
        this.min_y = 0.0;
      }
      if (initObj.hasOwnProperty('max_y')) {
        this.max_y = initObj.max_y
      }
      else {
        this.max_y = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type LDRInfo
    // Serialize message field [uav_ids]
    bufferOffset = _arraySerializer.string(obj.uav_ids, buffer, bufferOffset, null);
    // Serialize message field [positions]
    // Serialize the length for message field [positions]
    bufferOffset = _serializer.uint32(obj.positions.length, buffer, bufferOffset);
    obj.positions.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Point.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [min_x]
    bufferOffset = _serializer.float64(obj.min_x, buffer, bufferOffset);
    // Serialize message field [max_x]
    bufferOffset = _serializer.float64(obj.max_x, buffer, bufferOffset);
    // Serialize message field [min_y]
    bufferOffset = _serializer.float64(obj.min_y, buffer, bufferOffset);
    // Serialize message field [max_y]
    bufferOffset = _serializer.float64(obj.max_y, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type LDRInfo
    let len;
    let data = new LDRInfo(null);
    // Deserialize message field [uav_ids]
    data.uav_ids = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [positions]
    // Deserialize array length for message field [positions]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.positions = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.positions[i] = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [min_x]
    data.min_x = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [max_x]
    data.max_x = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [min_y]
    data.min_y = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [max_y]
    data.max_y = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.uav_ids.forEach((val) => {
      length += 4 + _getByteLength(val);
    });
    length += 24 * object.positions.length;
    return length + 40;
  }

  static datatype() {
    // Returns string type for a message object
    return 'uav_control/LDRInfo';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7f8c645ab0d7e706bc2196262140227e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string[] uav_ids
    geometry_msgs/Point[] positions
    float64 min_x
    float64 max_x
    float64 min_y
    float64 max_y
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new LDRInfo(null);
    if (msg.uav_ids !== undefined) {
      resolved.uav_ids = msg.uav_ids;
    }
    else {
      resolved.uav_ids = []
    }

    if (msg.positions !== undefined) {
      resolved.positions = new Array(msg.positions.length);
      for (let i = 0; i < resolved.positions.length; ++i) {
        resolved.positions[i] = geometry_msgs.msg.Point.Resolve(msg.positions[i]);
      }
    }
    else {
      resolved.positions = []
    }

    if (msg.min_x !== undefined) {
      resolved.min_x = msg.min_x;
    }
    else {
      resolved.min_x = 0.0
    }

    if (msg.max_x !== undefined) {
      resolved.max_x = msg.max_x;
    }
    else {
      resolved.max_x = 0.0
    }

    if (msg.min_y !== undefined) {
      resolved.min_y = msg.min_y;
    }
    else {
      resolved.min_y = 0.0
    }

    if (msg.max_y !== undefined) {
      resolved.max_y = msg.max_y;
    }
    else {
      resolved.max_y = 0.0
    }

    return resolved;
    }
};

module.exports = LDRInfo;
