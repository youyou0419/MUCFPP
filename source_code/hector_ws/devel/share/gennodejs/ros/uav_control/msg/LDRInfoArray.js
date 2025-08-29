// Auto-generated. Do not edit!

// (in-package uav_control.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let LDRInfo = require('./LDRInfo.js');

//-----------------------------------------------------------

class LDRInfoArray {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.ldr_infos = null;
    }
    else {
      if (initObj.hasOwnProperty('ldr_infos')) {
        this.ldr_infos = initObj.ldr_infos
      }
      else {
        this.ldr_infos = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type LDRInfoArray
    // Serialize message field [ldr_infos]
    // Serialize the length for message field [ldr_infos]
    bufferOffset = _serializer.uint32(obj.ldr_infos.length, buffer, bufferOffset);
    obj.ldr_infos.forEach((val) => {
      bufferOffset = LDRInfo.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type LDRInfoArray
    let len;
    let data = new LDRInfoArray(null);
    // Deserialize message field [ldr_infos]
    // Deserialize array length for message field [ldr_infos]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.ldr_infos = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.ldr_infos[i] = LDRInfo.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.ldr_infos.forEach((val) => {
      length += LDRInfo.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'uav_control/LDRInfoArray';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9e49ba5991a82a17025c4ca6b278a4e1';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uav_control/LDRInfo[] ldr_infos
    ================================================================================
    MSG: uav_control/LDRInfo
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
    const resolved = new LDRInfoArray(null);
    if (msg.ldr_infos !== undefined) {
      resolved.ldr_infos = new Array(msg.ldr_infos.length);
      for (let i = 0; i < resolved.ldr_infos.length; ++i) {
        resolved.ldr_infos[i] = LDRInfo.Resolve(msg.ldr_infos[i]);
      }
    }
    else {
      resolved.ldr_infos = []
    }

    return resolved;
    }
};

module.exports = LDRInfoArray;
